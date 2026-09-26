#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成 filters 单元测试所用的数据文件。

为什么用 Python 生成：
  测试要验证的是 C++ 实现，如果期望值也由同一份 C++ 代码算出来就没有意义。
  本脚本用**独立实现**的参考算法算出 expected 列，C++ 侧再把滤波器输出与它逐点
  比对，形成交叉校验（C++ vs Python）。

生成的 CSV 格式（UTF-8，`,` 分隔）：
  - `# key=value` 行：元数据，声明用哪个滤波器、参数是多少；
  - 第一行非注释行：列名（index,truth,raw,expected）；
  - 其余行：数值。

数据文件已经提交到仓库，构建时不需要 Python；修改算法或数据后重新执行本脚本即可：
    python3 src/test/test_data/generate_filters_data.py
"""

import math
import os

# ---------------------------------------------------------------------------
# 确定性伪随机噪声：自己实现 LCG，避免不同 Python 版本 random() 的差异，
# 保证任何人重新生成都得到逐位相同的数据文件。
# ---------------------------------------------------------------------------
class Lcg:
    def __init__(self, seed):
        self.state = seed & 0x7FFFFFFF

    def uniform(self):
        """返回 [0, 1) 上的伪随机数。"""
        self.state = (1103515245 * self.state + 12345) & 0x7FFFFFFF
        return self.state / float(0x80000000)

    def noise(self, amplitude):
        """返回 [-amplitude, amplitude] 上的零均值噪声。"""
        return amplitude * (2.0 * self.uniform() - 1.0)


# ---------------------------------------------------------------------------
# 参考实现（必须与 C++ 侧语义一致）
# ---------------------------------------------------------------------------
def ref_moving_average(raw, window):
    """滑动窗口均值：窗口未满时对已有样本求平均。"""
    out = []
    data = []
    for value in raw:
        data.append(value)
        if len(data) > window:
            data.pop(0)
        total = 0.0
        for item in data:
            total += item
        out.append(total / len(data))
    return out


def ref_median(raw, window):
    """滑动窗口中值：奇数取中间值，偶数取中间两值平均。"""
    out = []
    data = []
    for value in raw:
        data.append(value)
        if len(data) > window:
            data.pop(0)
        ordered = sorted(data)
        count = len(ordered)
        middle = count // 2
        if count % 2 == 1:
            out.append(ordered[middle])
        else:
            out.append((ordered[middle - 1] + ordered[middle]) / 2.0)
    return out


def ref_low_pass(raw, alpha):
    """一阶低通：第一个样本直接作为初值。"""
    out = []
    value = 0.0
    initialized = False
    for sample in raw:
        if not initialized:
            value = sample
            initialized = True
        else:
            value = alpha * sample + (1.0 - alpha) * value
        out.append(value)
    return out


def ref_kalman(raw, q, r, x0, p0):
    """一维卡尔曼：常值状态 + 随机游走过程模型。"""
    out = []
    x = x0
    p = p0
    for sample in raw:
        p = p + q                      # 预测
        gain = p / (p + r)             # 更新
        x = x + gain * (sample - x)
        p = (1.0 - gain) * p
        out.append(x)
    return out


# ---------------------------------------------------------------------------
# 写出 CSV
# ---------------------------------------------------------------------------
def write_csv(path, meta, index, truth, raw, expected):
    with open(path, "w", encoding="utf-8", newline="\n") as handle:
        for key in meta:
            handle.write("# {}={}\n".format(key, meta[key]))
        handle.write("index,truth,raw,expected\n")
        for i in range(len(raw)):
            handle.write("{},{:.17g},{:.17g},{:.17g}\n".format(i, truth[i], raw[i], expected[i]))
    print("生成 {}（{} 行）".format(path, len(raw)))


def write_signal_csv(path, meta, truth, raw):
    """只含 truth/raw 的信号文件，用于降噪指标测试（无 golden 期望列）。"""
    with open(path, "w", encoding="utf-8", newline="\n") as handle:
        for key in meta:
            handle.write("# {}={}\n".format(key, meta[key]))
        handle.write("index,truth,raw\n")
        for i in range(len(raw)):
            handle.write("{},{:.17g},{:.17g}\n".format(i, truth[i], raw[i]))
    print("生成 {}（{} 行）".format(path, len(raw)))


# ---------------------------------------------------------------------------
# 各个数据用例
# ---------------------------------------------------------------------------
def case_moving_average(out_dir):
    count = 40
    rng = Lcg(20240501)
    truth = [2.0 * math.sin(2.0 * math.pi * i / 20.0) for i in range(count)]
    raw = [truth[i] + rng.noise(0.5) for i in range(count)]
    window = 3
    expected = ref_moving_average(raw, window)
    write_csv(os.path.join(out_dir, "moving_average_w3.csv"),
              {"filter": "moving_average", "window": window,
               "desc": "sine+noise, moving average window=3"}, range(count), truth, raw, expected)


def case_median(out_dir):
    count = 40
    rng = Lcg(20240502)
    # 前 20 个样本为 0，之后阶跃到 5
    truth = [0.0 if i < 20 else 5.0 for i in range(count)]
    raw = [truth[i] + rng.noise(0.2) for i in range(count)]
    # 人为注入三个脉冲野值
    spikes = {8: 8.0, 25: -8.0, 33: 6.0}
    for index, delta in spikes.items():
        raw[index] += delta
    window = 3
    expected = ref_median(raw, window)
    write_csv(os.path.join(out_dir, "median_w3.csv"),
              {"filter": "median", "window": window, "spikes": "8:8,25:-8,33:6",
               "desc": "step+noise+spikes, median window=3"}, range(count), truth, raw, expected)


def case_low_pass(out_dir):
    count = 50
    rng = Lcg(20240503)
    truth = [0.0 if i < 10 else 10.0 for i in range(count)]
    raw = [truth[i] + rng.noise(1.0) for i in range(count)]
    alpha = 0.3
    expected = ref_low_pass(raw, alpha)
    write_csv(os.path.join(out_dir, "lowpass_alpha0.3.csv"),
              {"filter": "lowpass", "alpha": alpha,
               "desc": "step+noise, first-order low pass alpha=0.3"}, range(count), truth, raw, expected)


def case_kalman(out_dir):
    count = 50
    rng = Lcg(20240504)
    truth = [0.0 if i < 10 else 5.0 for i in range(count)]
    raw = [truth[i] + rng.noise(1.0) for i in range(count)]
    q, r, x0, p0 = 1e-4, 1e-2, 0.0, 1.0
    expected = ref_kalman(raw, q, r, x0, p0)
    write_csv(os.path.join(out_dir, "kalman_step.csv"),
              {"filter": "kalman", "q": q, "r": r, "x0": x0, "p0": p0,
               "desc": "step+noise, 1D Kalman q=1e-4 r=1e-2"}, range(count), truth, raw, expected)


def case_sine_noisy(out_dir):
    count = 200
    rng = Lcg(20240505)
    truth = [3.0 * math.sin(2.0 * math.pi * i / 50.0) for i in range(count)]
    raw = [truth[i] + rng.noise(1.0) for i in range(count)]
    write_signal_csv(os.path.join(out_dir, "sine_noisy.csv"),
                     {"desc": "3*sin(i/50*2pi)+uniform noise, metric only"}, truth, raw)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    out_dir = os.path.join(here, "filters")
    if not os.path.isdir(out_dir):
        os.makedirs(out_dir)

    print("输出目录: {}".format(out_dir))
    case_moving_average(out_dir)
    case_median(out_dir)
    case_low_pass(out_dir)
    case_kalman(out_dir)
    case_sine_noisy(out_dir)
    print("完成。")


if __name__ == "__main__":
    main()
