# pure-nav 代码编写规范

> 面向对象：本仓库所有 C++ 代码的作者与 AI Agent。
> 适用范围：`src/` 下的项目自有代码（`src/thirdparty/`、`src/odometry/Super-LIO`、`src/perception/M-detector` 等引入的第三方与参考实现不追溯）。
> 效力：新增代码必须遵守；改动存量文件时，改动范围内的注释按本规范整理（过渡办法见第 7 节）。
> 格式依据：仓库根目录 `.clang-format` 是格式问题的唯一裁决者。本文件不重复其内容，只规定格式工具管不到的注释、命名与文档约定。

---

## 0. 总则

**功能写进文件名与 README，原理写进紧挨代码的一行注释，接口写进 Doxygen，格式交给 clang-format。**

四句话分别对应第 2、3、4、1 节。出现冲突时，优先级为：`.clang-format`（机器可判定）> 本规范 > 个人习惯。

---

## 1. 格式：以 `.clang-format` 为唯一依据

- 提交前对改动的文件执行 `clang-format -i`；只格式化自己改动的代码，可配合 `git clang-format`。
- 关键约定速查（摘自 `.clang-format`，**冲突时以配置文件为准**）：
  - 基线为 Google 风格；缩进 2 空格，不使用制表符；列宽 80；续行缩进 4 空格。
  - `BreakBeforeBraces: Attach`：左大括号不换行；构造函数初始化列表在冒号前换行。
  - `PointerAlignment: Left`：指针与引用左贴，写作 `int* p`、`const T& value`。
  - `IncludeBlocks: Regroup`：include 分组并排序，标准库 / 第三方 / 本项目各成一组。
  - `AllowShortFunctionsOnASingleLine: None`、`AllowShortIfStatementsOnASingleLine: Never`、`AllowShortLoopsOnASingleLine: false`：函数体、if、循环一律换行并加花括号。
  - `ReflowComments: Always`：说明文字按列宽自动重排（配合方式见 4.3）。
- 不手工对齐、不为对齐补空格、不用注释画表格或分隔横幅。
- 不为了迁就个别文件去修改 `.clang-format`。
- 静态检查以 `.clang-tidy` 为准，保持零新增告警。

---

## 2. 文件组织与命名：功能归属

### 2.1 文件名体现功能

- 源码文件顶部不写大段功能说明注释。**看文件名就应该知道这个文件做什么。**
- 一个文件只承担一件内聚的事，文件名用能自解释的名词短语：`filters.hpp` 合格；`utils.hpp`、`common.hpp`、`misc.hpp`、`tools.hpp` 不合格。
- 头文件与实现文件同名成对：`xxx.hpp` 放声明与接口契约，`xxx.cpp` 放实现。头文件统一用 `.hpp`。
- 文件名一旦不能自解释，就必须在所属模块 README 的文件清单里补一行说明。

### 2.2 README 承载功能说明

- 每个模块目录（`src/<模块>/`）与每个较大的子库提供一份 `README.md`。
- README 至少写清：职责、对外接口与入口、输入与输出、依赖、最小可运行示例、注意事项与已知限制。
- 需要展开的原理、公式推导、数据流、配置项说明写进 README 或 `assets/docs/`，**不要塞进源码文件头**。
- 新增文件或改动模块行为时，同步更新该模块 README。

### 2.3 命名

命名不在 clang-format 管辖范围内，沿用仓库现有约定；新增代码与所在模块保持一致。

| 对象 | 约定 | 示例 |
|---|---|---|
| 类型（class / struct / enum / 别名） | 大驼峰 | `ShmPublisher`、`PointCloudFrame`、`PointDataType` |
| 函数与方法 | 小驼峰；既有下划线式（如 `filters.hpp` 的 `window_size()`）保持原样 | `publish()`、`fromString()` |
| 成员变量 | 小驼峰 + 尾部下划线 | `service_`、`window_size_` |
| 常量 / `constexpr` | `k` + 大驼峰 | `kMaxTopicSegmentLength`、`kLivoxGravity` |
| 枚举值 | `k` + 大驼峰 | `kMid360`、`kCartesianHigh` |
| 命名空间 | 小写下划线 | `pure::common`、`pure::driver` |
| 宏 | 自有代码尽量避免宏；必要时全大写下划线并加项目前缀 | `PURE_NAV_XXX` |

- 同一模块内不混用命名风格；不做无意义的重命名。
- 标识符中可以沿用领域通用缩写（`imu`、`lidar`、`shm`），注释里必须给出全称（见第 5 节）。

---

## 3. `.cpp` 注释：只注释重要的原理性语句

### 3.1 两条硬性规则

1. **只有重要的、体现原理的语句才写注释。** 常规赋值、循环、函数调用、日志输出不写注释。
2. **每行代码最多搭配一行注释**，注释与被注释的代码紧邻（写在其正上方，或同一行行尾）。禁止用多行注释段落去解释一行代码。

### 3.2 注释写"为什么 / 原理"，不写"做什么"

合格：

```cpp
// 窗口满了就丢掉最旧的样本，保持 sum_ 与窗口内容一致
if (window_.size() > window_size_) {
    sum_ -= window_.front();
    window_.pop_front();
}

// 中位数取中间位置；偶数个样本时取中间两者平均，故先复制再排序，避免破坏时间顺序
std::vector<T> sorted(window_.begin(), window_.end());
const std::size_t middle = sorted.size() / 2;
```

不合格：

```cpp
// 如果窗口内样本数大于窗口容量，就弹出队首元素
if (window_.size() > window_size_) {
    sum_ -= window_.front();
    window_.pop_front();
}

// 下面开始处理数据
// 第一步先复制
// 第二步再排序
std::vector<T> sorted(window_.begin(), window_.end());
```

### 3.3 值得写注释的原理性语句

- 数学与算法依据：卡尔曼增益、外积方向、坐标变换顺序、四元数归一化、数值稳定的等价变形。
- 单位与坐标系：`m` 还是 `mm`、度还是弧度、机体系还是世界系、时间基准。
- 边界与退化处理：除零保护、窗口未满、时间戳回退、`epsilon` 取值的由来。
- 并发与内存：线程归属、为何无锁、零拷贝样本的生命周期、对齐与平凡复制要求。
- 魔法数与硬编码常量：协议规定、标定结果、实测标定值。
- 性能取舍：预分配、原地操作、复杂度选择及其代价。

### 3.4 禁止的注释

- 文件顶部或函数上方的大段横幅式说明（`// ===== 模块说明 =====`）：功能归属文件名与 README。
- 注释掉的死代码：删除，历史交给 git。
- 变更记录（作者、日期、"修改了什么"）：写进提交信息。
- 复述代码的注释，以及给显而易见语句加的注释。
- 未展开的英文缩写（见第 5 节）。
- 行尾注释只用于足够短的短语；整句说明放到代码上方单独一行。

---

## 4. `.hpp` 注释：用 Doxygen 标注每个函数

### 4.1 覆盖范围

- 每个类 / 结构体 / 枚举：`@brief` 一句话说明"是什么"。
- **每个公开函数与方法：必须写 `@brief`**，并按需补齐：

| 标签 | 何时使用 |
|---|---|
| `@brief` | 全部公开接口必写，一句话说清职责 |
| `@param` | 有参数时，逐个说明含义、单位、取值范围 |
| `@return` | 有返回值时，说明返回内容与特殊情况下的取值 |
| `@tparam` | 模板参数，说明允许的实参类型或约束 |
| `@throws` | 会抛异常时，写明异常类型与触发条件 |
| `@pre` / `@note` / `@warning` | 前置条件、补充说明、危险或易错用法（如线程约束、生命周期） |

- 每个公开成员变量与常量：行尾 `///<` 一行说明。
- 私有辅助函数：鼓励写 `@brief` 解释原理；至少保证读者不用猜其职责。
- `@brief` 不要复述函数名已经表达的信息（`publish()` 写"@brief 发布"是废话，应写清发布语义，如"把消息原地构造在共享内存中并对外提供"）。

### 4.2 模板

```cpp
/// @brief 滑动窗口均值滤波器，输出窗口内样本的算术平均。
/// @tparam T 标量类型，当前支持 float 与 double。
template <typename T>
class SlidingWindowFilter : public Filter<T> {
public:
    /// @brief 构造滑动窗口均值滤波器。
    /// @param window_size 窗口容量（样本数），必须大于 0。
    /// @throws std::invalid_argument 当 window_size 为 0 时抛出。
    explicit SlidingWindowFilter(std::size_t window_size);

    /// @brief 在线滤波：喂入一个新样本，返回当前窗口内的算术平均。
    /// @param value 新样本。
    /// @return 当前窗口内样本的算术平均；窗口未填满时按已有样本求平均。
    /// @note 非线程安全，同一实例只由一个线程调用。
    T update(T value) override;

    std::size_t window_size_;  ///< 窗口容量（样本数）
};
```

书写形式统一为 `///` 单行注释与 `///<` 行尾注释，不使用 `/** ... */` 大块形式，避免与列宽重排互相干扰。

### 4.3 与 clang-format 的配合

- 说明性文字不必手工折行，`ReflowComments: Always` 会按 80 列重排。
- 每个 `@` 命令独占一行、命令之间不空行；clang-format 保留以 `@` 开头的行，而会合并普通文字行。
- Doxygen 块紧贴声明上方，不留空行；相邻函数之间用一个空行分隔。
- 若某个 Doxygen 块被重排得难以阅读，调整写法，而不是修改 `.clang-format`。

### 4.4 `.hpp` 里不写什么

- 不写文件头的大段介绍（同 3.4）。
- 不重复 `.cpp` 中的实现细节注释：**接口契约写 `.hpp`，实现原理写 `.cpp` 的那一行注释**。
- 不写"本文件包含……"式的目录注释：文件清单写在 README。

---

## 5. 术语与英文缩写

- 注释与文档使用中文，标识符保持英文。
- **阐明原理时尽量少用英文缩写。** 首次出现给出中文全称，必要时在括号内附英文全称或缩写。

| 不推荐 | 推荐写法 |
|---|---|
| EMA | 指数加权移动平均（exponential moving average） |
| FIFO | 先进先出（first in first out） |
| IMU | 惯性测量单元 |
| IPC / SHM | 进程间通信 / 共享内存 |
| LPF / HPF | 低通滤波 / 高通滤波 |
| KF / EKF | 卡尔曼滤波 / 扩展卡尔曼滤波 |
| tmp / buf / cfg / msg | 临时值 / 缓冲区 / 配置 / 消息 |

- 例外：已成为专有名词或工具名的缩写可直接使用，如 `C++`、`CMake`、`Doxygen`、`OpenCV`、`Eigen`、`iceoryx`、`API`、`RAII`、`POD`、`SDK`、`TCP`、`UDP`；数学函数名 `sin` / `cos` / `atan2` 与单位 `m/s`、`rad/s` 也直接使用。
- 判据：不了解该缩写的同伴读到这里，是否需要额外查资料？需要就展开。
- 标识符不受此限制（`imu_callback`、`shm_publisher` 可以保留缩写），但同一注释里出现的缩写要能对应到标识符或已展开的全称。

---

## 6. 提交前自检清单

- [ ] 已对改动文件执行 `clang-format -i`，`git diff` 中没有纯格式噪声。
- [ ] 文件顶部没有大段功能说明；功能能从文件名或 README 中查到。
- [ ] 新增文件、改动行为已同步到所属模块 README。
- [ ] `.cpp` 中没有注释掉的死代码、没有变更记录、没有复述代码的注释。
- [ ] `.cpp` 中每条原理性注释都是单行且紧贴被注释的代码。
- [ ] `.hpp` 中每个公开函数都有 Doxygen（`@brief` + 必要的 `@param` / `@return` / `@tparam` / `@throws` / `@note`）。
- [ ] 注释中没有未展开的英文缩写。
- [ ] `.clang-tidy` 无新增告警。

---

## 7. 存量代码的过渡

- 本规范发布后新增的代码直接遵守。
- 存量文件（例如 `src/common/common_libs/include/filters.hpp`、`src/common/shm/include/shm.hpp`）顶部的横幅式说明注释不符合 3.4：**在因其他原因改动这些文件时顺手删除**，把其中有价值的内容下沉到对应 README。
- 不专门发起"只改注释格式"的提交，避免与功能改动混在一起、污染 `git blame`。
