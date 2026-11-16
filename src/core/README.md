# 核心数据与算法工厂

`core` 定义哨兵导航、自瞄的进程内数据契约与算法接口。实现留在各业务模块，
应用层负责构造、初始化、调度、时间同步、坐标变换、发布订阅及驱动输出。
所有新接口位于 `pure::core` 命名空间，使用 C++17 和 Eigen3；不依赖 ROS、
PCL、Sophus、OpenCV 或硬件 SDK。

本次完善的是接口和工厂创建能力。地形识别、三维规划、自瞄模型尚未在此实现，
现有 `pure_nav::Navigation` 二维闭环也未自动接入这些新接口。

## 文件与职责

| 文件 | 内容 |
|---|---|
| `abstract.hpp` | `IProduct` 生命周期、`IFactory`、类型化工厂与通用具体工厂 |
| `type_alias.hpp` | Eigen 类型、单调时间、坐标系、观测身份、三维位姿 |
| `nav_ds.hpp` | 同步传感器输入、定位、地形、车体能力、任务、轨迹、底盘命令 |
| `navigation_factory.hpp` | `ISlam / ITerrain / IPlan / IControl` 与导航工厂 |
| `autoaim_ds.hpp` | 图像、检测、相机标定、同步观察者状态、三维目标、云台跟随结果 |
| `autoaim_factory.hpp` | `IDetector / ITracker / IAimSolver` 与自瞄工厂 |

旧文件没有被其他项目源码引用，因此直接整理为命名空间内的接口，未保留全局
`using namespace cv` 或不完整的旧方法。`global_plan/local_plan` 改为
`globalPlan/localPlan`，`getFreq` 改为 const 查询，工厂不再返回空产品。

## 数据约定

长度 m、角度 rad、速度 m/s、时间戳 ns；统一使用本机单调时钟。
设备时间到主机时钟的映射发生在输入适配层。`session + sequence` 标识一次来源
观测，重启时更换 session；0 时间合法，有效性不能只用时间是否为零判断。

`Pose3D` 的四元数将局部系旋转到消息头参考系，必须有限且归一化。
消息头标识参考系，不会执行任何坐标变换。车体、雷达、相机、云台的外参由
算法构造配置或输入适配层提供。姿态必须保留横滚和俯仰，不能在坡面上只传 yaw。

这些结构包含 vector、Eigen 对象或 shared_ptr，是**进程内模型**，不能直接
memcpy 到 iceoryx；跨进程仍需固定容量消息和显式转换。
快照按值组织以明确所有权；`TerrainMap::volume` 是可选只读三维快照。
数据量较大时，调度层可在保证生命周期和不再修改底层数据的前提下共享快照，
不能把临时回调内的裸指针交给算法线程。

## 导航数据流

```text
原始点云 + 按序惯性历史 → ISlam → NavigationState
去畸变且配准到同一参考系的点云 + 雷达射线原点 + 状态 + RobotCapability
                         → ITerrain → TerrainMap
NavigationGoal + 状态 + 地形 + 车体能力 → IPlan → Trajectory
当前时间 + 控制间隔 + 状态 + 轨迹 + 地形 + 车体能力 → IControl → ChassisCommand
```

`ISlam` 支持建图/定位模式以及安装先验地图。点云输出和地形输入间的去畸变、
扫描身份匹配、外参变换仍须具体定位实现或适配层提供。
`IPlan` 将全局路线和局部轨迹分开；任务包括保持、到点、巡逻、补给和穿行洞口。
巡逻点切换、比赛状态和补给优先级由决策层处理，工厂不承担决策。

`Trajectory` 关联任务 ID、地形会话和序号，并保留三维姿态、期望速度、相对时间
和有效期限。`IControl` 独立周期运行，不能等待规划线程产出新路径才更新控制。

### 坡面、洞口与负障碍

| 场景 | 数据表达 | 算法接入要求 |
|---|---|---|
| 坡面 | `kSlope`、地面法向、高程、最大坡角、粗糙度 | 用路线方向和法向分解纵坡/横坡，分别对照上下坡及横坡能力 |
| 台阶 | `kStep`、`step_up_m` | 检查轮系能力与完整车体包络，不能只按格心判定 |
| 悬崖、落差 | `kDropOff`、`drop_down_m` | 下坡和下台阶不同，按最大落差约束单独检查 |
| 地面坑洞 | `kHole`、`kMissing` 支撑 | 必须有支撑缺失证据；无回波不能直接认定为坑洞 |
| 可穿行洞口 | `Passage` 入口/出口姿态、最小净宽净高、长度 | 检查含安全余量的车体包络、转弯扫掠体和通道地面支撑 |
| 顶棚、悬空结构 | `kOverhang`、`clearance_m`、三维占据网格 | 检查净空，空的 volume 代表没有三维证据 |
| 未观测区域 | `kUnknown`、NaN 几何、无限代价 | 默认不放行，不能解释为平地或空闲 |

`GridMap2D` 是单层带高程网格，同一 XY 的上下两层道路不能合并到一格。
多层地形需使用三维体素或后续增加分层地表表示，并建立路线连通关系。
`GridMap3D` 采用行主序，调用方必须先检查尺寸乘法溢出和存储长度。
当前 JPS 的二维占据图不能直接实现上述三维通行判断，需要适配与算法扩展。

`TerrainType` 表示识别语义，`Traversability` 表示对本车的通行结论，两者不能等同。
`RobotCapability` 默认 `configured=false`，实际尺寸、横坡/纵坡、台阶、落差和间隙
能力须按本车填写与验证，本接口没有内置通用车体能力数值。

调用算法前须检查：输入有限性、标定就绪、尺寸与像素/网格存储长度、时间顺序、
同一扫描的状态与点云身份、坐标系、任务身份和有效期限。
地形还须检查各格观测年龄；整图刷新不代表每格都被重新观测。
这些是具体实现的接口合同，数据结构本身不会执行识别、同步或完整合法性验证。
规划失败应返回明确状态；控制输入无效、过期、定位丢失或预测碰撞时返回无效零速。
驱动仍须独立检查命令期限和看门狗，不能把这些头文件当成已接通的安全控制实现。

## 自瞄数据流

`Image → IDetector → DetectionFrame`；`ITracker` 结合原图、模型关键点、
相机标定与曝光时刻的观察者姿态，输出三维目标及跟踪状态。
`IAimSolver` 根据目标和观察者状态输出云台跟随角度。模块不直接访问硬件。
空但有效的检测与推理失败分开；目标丢失、预测和跟踪均有明确状态及有效期限。
跟踪器需要的目标几何、模型参数和相机畸变模型通过具体算法配置注入。

## 使用工厂

保留原有的产品/工厂结构，每个工厂创建一种具体算法，应用层组合多个工厂。
下面的具体算法类由业务模块实现，并非本目录已经提供：

```cpp
#include "navigation_factory.hpp"

// MyTerrain 必须完整实现 ITerrain 的 init、getFreq、analyze。
pure::core::NavigationFactory<MyTerrain> factory([config] {
  return std::make_unique<MyTerrain>(config);
});
auto terrain = factory.create();  // unique_ptr<MyTerrain>，可直接调用 analyze
terrain->init();
auto map = terrain->analyze(input);

const pure::core::INavigationFactory& typed = factory;
auto navigation_product = typed.createTypedProduct();
const pure::core::IFactory& generic = factory;
auto product = generic.createProduct();
```

自瞄使用 `AutoAimFactory<MyDetector>`，默认构造产品可省略回调。
工厂不会隐式调用 init，也不会启动线程。构造回调每次返回新实例，优先按值捕获
配置；引用捕获需要调用方保证生命周期。空回调、空创建结果通过异常报告；
创建异常原样传播。产品类别在编译期检查，不允许将自瞄算法注册到导航工厂。
工厂与产品均不承诺线程安全；初始化和调用由应用层串行调度。

## 构建与验证

链接 `pure_nav_core` 获得头文件、C++17 和 Eigen3 依赖。
核心接口也支持独立构建，不需要先修复主工程历史目录路径：

```bash
cmake -S src/core -B build/core
cmake --build build/core
ctest --test-dir build/core --output-on-failure
```

`test_core_contracts` 验证工厂依赖注入、独立实例、显式初始化、类型擦除、
空创建拒绝、观测身份保留、默认未知地形以及网格越界/尺寸溢出。
这些验证是接口合同测试，不是实车地形识别或导航性能验收。
