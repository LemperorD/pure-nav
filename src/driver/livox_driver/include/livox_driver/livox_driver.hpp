// ============================================================================
// livox_driver.hpp —— Livox（览沃）雷达驱动类
//
// 职责边界：
//   * 管 SDK 生命周期（初始化 / 启动 / 反初始化，多实例引用计数）
//   * 在 SDK 全局回调里做**最轻量**的搬运：把 UDP 包拷进有界队列，立即返回
//   * 用**独立工作线程**做点云组帧/解码，用**另一个独立工作线程**做 IMU 分发
//   * 向上层提供回调与"取最新帧"两种消费方式，并暴露丢包统计
//
// ---------------------------------------------------------------------------
// 关于"点云和 IMU 要不要分成两个取流线程"——结论：要，理由如下
// ---------------------------------------------------------------------------
// 事实（读 Livox-SDK2 源码得到，不是猜测）：
//   sdk_core/data_handler/data_handler.cpp:69 的 DataHandler::Handle() 同时
//   分发点云和 IMU；它的调用点是 device_manager.cpp:495/508，位于
//   DeviceManager::OnData()（IOLoopDelegate::OnData）。而点云口和 IMU 口的
//   两个 socket 都注册在**同一个** data_io_thread_（device_manager.cpp:411 /
//   735 / 749）。也就是说 SDK 内部两者**本来就是串行**在一个线程上回调的。
//
// 由此得到两条设计约束：
//   1) 既然 SDK 只有一条收包线程，我们的回调就绝不能在里面做重活——否则点云
//      解码会直接把 IMU 的回调也一起卡住（以及造成内核收包缓冲溢出丢包）。
//      所以回调只做 memcpy + 入队。
//   2) IMU 与点云解码必须放在**不同的消费线程**：
//        - 节奏不同：点云 10Hz/整帧、单帧上万点；IMU 200Hz 且每包极小。
//        - 时延敏感度不同：IMU 直接喂 LIO/里程计，毫秒级抖动就会恶化姿态估计，
//          而点云解码是"攒够一帧再算"的批处理，天然有几十毫秒缓冲。
//        - 若共用一个消费线程，一次整帧组装（数 ms）会让后面的 IMU 排队等待，
//          形成周期性头阻塞(head-of-line blocking)。
//        - 两条流互相独立，点云消费端变慢（下游感知卡顿）不应该牵扯 IMU。
//      代价只是多一个线程的栈，收益是 IMU 时延有界，因此选择分开。
//
// 反过来，如果 IMU 只是拿来打日志、不参与实时估计，那么单线程也够用——
// LivoxConfig::enable_imu 关掉即可完全不启动 IMU 线程。
// ============================================================================

#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "livox_driver/blocking_queue.hpp"
#include "livox_driver/livox_types.hpp"

namespace pure {
namespace driver {

class LivoxSdkRuntime; // 定义在 livox_driver.cpp：SDK 生命周期 + 全局回调路由

using PointCloudCallback = std::function<void(const PointCloudFrame&)>;
using ImuCallback = std::function<void(const ImuSample&)>;

class LivoxDriver {
public:
    LivoxDriver();
    ~LivoxDriver();

    LivoxDriver(const LivoxDriver&) = delete;
    LivoxDriver& operator=(const LivoxDriver&) = delete;

    // ------------------------------------------------------------------------
    // 生命周期
    // ------------------------------------------------------------------------

    // 初始化 SDK（必要时）+ 启动工作线程。返回 true 表示"驱动已开始工作"，
    // **不**代表雷达已连接：雷达可能开机较晚，连上后会自动绑定并配置。
    // 若需要阻塞等待雷达出现，随后调用 waitForLidar()。
    bool start(const LivoxConfig& config);

    // 幂等。顺序：注销回调路由 → 关闭队列唤醒线程 → join → 释放 SDK 引用。
    void stop();

    bool running() const { return running_.load(); }

    // 阻塞等待目标雷达被发现并完成绑定/配置
    bool waitForLidar(int timeout_ms);

    bool connected() const { return connected_.load(); }

    // ------------------------------------------------------------------------
    // 状态查询
    // ------------------------------------------------------------------------

    uint32_t handle() const { return handle_.load(); }
    std::string serialNumber() const;
    std::string lidarIp() const;
    LidarModel model() const { return model_.load(); }

    // 注意：返回的是 start() 时保存的配置，启动后不应再改
    const LivoxConfig& config() const { return config_; }

    // ------------------------------------------------------------------------
    // 数据消费
    //
    // 回调在工作线程里被调用（点云线程 / IMU 线程），**不要**在回调里调用
    // stop() 或做长时间阻塞操作。需要"拉"模式的消费者用 latestFrame/latestImu。
    // ------------------------------------------------------------------------
    void setPointCloudCallback(PointCloudCallback callback);
    void setImuCallback(ImuCallback callback);

    // 取最近一帧的深拷贝；未收到数据时返回 false
    bool latestFrame(PointCloudFrame& frame) const;
    bool latestImu(ImuSample& sample) const;

    // ------------------------------------------------------------------------
    // 统计
    // ------------------------------------------------------------------------
    LivoxStats stats() const;
    void resetStats();

private:
    friend class LivoxSdkRuntime;

    // ---- 以下方法在 SDK 回调线程里执行，必须保持非阻塞 ----
    bool accepts(uint32_t handle, const char* sn) const;
    void bindLidar(uint32_t handle, uint8_t dev_type, const char* sn, const char* ip);
    void onRawPointPacket(const RawPacket& packet);
    void onRawImuPacket(const RawPacket& packet);
    void onSdkPushMessage(uint8_t dev_type, const std::string& info);

    // ---- 工作线程 ----
    void pointThreadMain();
    void imuThreadMain();
    void publishFrame(PointCloudFrame& frame);
    void publishImu(const ImuSample& sample);

    void configureLidar(uint32_t handle);

    // ---- 配置（start() 里写一次，之后只读）----
    LivoxConfig config_;

    std::mutex lifecycle_mutex_; // 串行化 start()/stop()

    std::atomic<bool> running_;
    std::atomic<bool> connected_;
    std::atomic<uint32_t> handle_;
    std::atomic<LidarModel> model_;

    bool point_enabled_;
    bool imu_enabled_;
    bool imu_acc_is_g_;
    bool imu_gyro_is_rad_s_;
    int frame_timeout_ms_;

    mutable std::mutex lidar_mutex_;
    std::condition_variable lidar_cv_;
    std::string sn_;
    std::string ip_;

    BlockingQueue<RawPacket> point_queue_;
    BlockingQueue<ImuSample> imu_queue_;
    FrameAssembler assembler_;

    std::thread point_thread_;
    std::thread imu_thread_;

    mutable std::mutex callback_mutex_;
    PointCloudCallback point_callback_;
    ImuCallback imu_callback_;

    mutable std::mutex latest_mutex_;
    PointCloudFrame latest_frame_;
    ImuSample latest_imu_;
    bool has_frame_;
    bool has_imu_;

    std::atomic<uint64_t> st_packets_received_;
    std::atomic<uint64_t> st_packets_dropped_;
    std::atomic<uint64_t> st_frames_published_;
    std::atomic<uint64_t> st_frames_incomplete_;
    std::atomic<uint64_t> st_points_published_;
    std::atomic<uint64_t> st_imu_packets_received_;
    std::atomic<uint64_t> st_imu_samples_published_;
    std::atomic<uint64_t> st_imu_dropped_;
};

} // namespace driver
} // namespace pure
