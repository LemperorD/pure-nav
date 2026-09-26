# test_data

测试用数据。**这里只放数据，不放测试目标**（测试目标在 `src/test/CMakeLists.txt` 注册）。

## filters/

供 `test_filters_data` 使用的滤波器测试数据，由 `generate_filters_data.py` 生成。

### 文件格式

UTF-8、`,` 分隔、LF 换行：

```
# key=value        ← 元数据，可有多行，读取时被跳过并解析成配置
# filter=kalman
# q=0.0001
index,truth,raw,expected
0,0,-0.0238,-0.0238
...
```

| 列 | 含义 |
|---|---|
| `index` | 样本序号 |
| `truth` | 无噪声的真实信号 |
| `raw` | 带噪声的观测（滤波器的输入） |
| `expected` | 参考实现算出的期望输出（golden），可能缺省 |

`# key=value` 元数据里，`filter` 决定用哪个滤波器，其余键是对应构造参数
（`window` / `alpha` / `q`,`r`,`x0`,`p0`）。

### 现有数据

| 文件 | 滤波器 | 说明 |
|---|---|---|
| `moving_average_w3.csv` | `SlidingWindowFilter(3)` | 正弦+噪声，验证滑动平均 |
| `median_w3.csv` | `MedianFilter(3)` | 阶跃+噪声+3 个脉冲野值，验证中值滤波 |
| `lowpass_alpha0.3.csv` | `FirstOrderLowPassFilter(0.3)` | 阶跃+噪声，验证一阶低通与稳态误差 |
| `kalman_step.csv` | `KalmanFilter1D(1e-4, 1e-2)` | 阶跃+噪声，验证一维卡尔曼与稳态误差 |
| `sine_noisy.csv` | 全部四种 | 仅 `truth`/`raw`，用于对比降噪指标 |

### 为什么需要 Python 参考实现

如果 `expected` 也由同一份 C++ 代码产生，测试就变成"自己验证自己"。因此
`generate_filters_data.py` 用**独立实现**的参考算法算出 `expected`，C++ 侧逐点比对，
形成交叉校验（实测最大误差 ≤ 9e-16，即机器精度）。

### 重新生成

数据文件已提交，**构建时不需要 Python**。修改算法或数据后：

```bash
python3 src/test/test_data/generate_filters_data.py
```

脚本用固定种子的自实现 LCG 产生噪声，保证任何人重新生成都得到逐位相同的文件。

### 关于指标

测试用**粗糙度**（一阶差分的标准差）衡量噪声抑制，而不是"与真值的 RMSE"：任何
平滑滤波器都有相位滞后，阶跃/正弦信号上 filtered 的 RMSE 反而可能大于 raw，用
RMSE 会把正确的滤波器判成失败。阶跃类数据另外校验末尾若干点的稳态误差。
