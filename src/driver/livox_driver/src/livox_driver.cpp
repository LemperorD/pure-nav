// ============================================================================
// livox_driver.cpp —— Livox 雷达驱动实现
//
// 线程模型（为什么这样切分，详见 livox_driver.hpp 顶部注释）：
//
//   [SDK data_io_thread_]  ← Livox-SDK2 内部唯一的数据收包线程
//        |  DataHandler::Handle() 同时分发点云与 IMU（同一个线程！）
//        v
//   pointCloudTrampoline / imuTrampoline   ← 本文件
//        |  只做：查表找驱动 → memcpy 进栈上 RawPacket → 入队 → 返回
//        v
//   point_queue_ (RawPacket, 2KB)         imu_queue_ (ImuSample, 56B)
//        |                                      |
//        v                                      v
//   pointThreadMain()                      imuThreadMain()
//        |  组帧 + 坐标解码（ms 级批处理）        |  单位换算 + 分发（us 级）
//        v                                      v
//   用户点云回调 / latestFrame_             用户 IMU 回调 / latest_imu_
//
// 关键点：两个回调都不做重活，两个消费者线程互相独立，因此点云组帧的
// 周期性耗时不会变成 IMU 的头阻塞延迟。
// ============================================================================

#include "livox_driver/livox_driver.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <exception>
#include <map>
#include <sstream>
#include <vector>

#ifdef PURE_NAV_HAS_LIVOX_SDK2
#include "livox_lidar_api.h"
#include "livox_lidar_def.h"
#endif

namespace pure {
namespace driver {

namespace {

// 单包 IMU 采样数上限（Mid360 为 1；留足余量给其它机型）
constexpr std::size_t kMaxImuSamplesPerPacket = 32;

std::atomic<bool> g_verbose(false);

void logLine(const char* level, const std::string& text) {
    std::fprintf(stderr, "[livox_driver][%s] %s\n", level, text.c_str());
}

void logInfo(const std::string& text) {
    if (g_verbose.load()) {
        logLine("info", text);
    }
}

void logWarn(const std::string& text) { logLine("warn", text); }

void logError(const std::string& text) { logLine("error", text); }

// 从定长 char 数组（可能没有 '\0'）构造 std::string
std::string boundedString(const char* text, std::size_t max_length) {
    if (text == nullptr) {
        return std::string();
    }
    std::size_t length = 0;
    while (length < max_length && text[length] != '\0') {
        ++length;
    }
    return std::string(text, length);
}

#ifdef PURE_NAV_HAS_LIVOX_SDK2

std::string toHexHandle(uint32_t handle) {
    std::ostringstream stream;
    stream << "0x" << std::hex << handle;
    return stream.str();
}

uint8_t toSdkPointDataType(PointDataType type) {
    switch (type) {
        case PointDataType::kCartesianHigh:
            return kLivoxLidarCartesianCoordinateHighData;
        case PointDataType::kCartesianLow:
            return kLivoxLidarCartesianCoordinateLowData;
        case PointDataType::kSpherical:
            return kLivoxLidarSphericalCoordinateData;
        case PointDataType::kDoubleEcho:
            return kLivoxLidarDoubleEchoData;
    }
    return kLivoxLidarCartesianCoordinateHighData;
}

// 异步控制命令的应答回调：运行在 SDK 命令线程，只做日志，绝不阻塞
void controlResponseCallback(livox_status status,
                             uint32_t handle,
                             LivoxLidarAsyncControlResponse* response,
                             void* client_data) {
    (void)client_data;
    if (response == nullptr) {
        return;
    }
    if (status != kLivoxLidarStatusSuccess || response->ret_code != 0) {
        std::ostringstream stream;
        stream << "控制命令失败 handle=" << toHexHandle(handle) << " status=" << status
               << " ret_code=" << static_cast<int>(response->ret_code)
               << " error_key=0x" << std::hex << response->error_key;
        logWarn(stream.str());
    }
}

#endif // PURE_NAV_HAS_LIVOX_SDK2

} // namespace

// ============================================================================
// LivoxSdkRuntime
//
// Livox-SDK2 的把流/命令回调都是**进程级全局**的（SetLivoxLidarPointCloudCallBack
// 只能设置一个），因此需要一层单例做"全局回调 → 具体驱动实例"的路由：
//   * 引用计数管理 LivoxLidarSdkInit/Start/Uninit，支持多个 LivoxDriver 共存；
//   * 用 handle(雷达 IP) → LivoxDriver* 的绑定表分发数据与状态；
//   * 回调里持有 registry_mutex_ 直到调用完驱动方法，保证 unregisterDriver()
//     返回后不会再有回调触碰即将析构的驱动对象（避免 use-after-free）。
// ============================================================================
class LivoxSdkRuntime {
public:
    static LivoxSdkRuntime& instance() {
        static LivoxSdkRuntime runtime;
        return runtime;
    }

    bool acquire(const LivoxConfig& config);
    void release();

    void registerDriver(LivoxDriver* driver) {
        std::lock_guard<std::mutex> lock(registry_mutex_);
        drivers_.push_back(driver);
    }

    void unregisterDriver(LivoxDriver* driver) {
        std::lock_guard<std::mutex> lock(registry_mutex_);
        drivers_.erase(std::remove(drivers_.begin(), drivers_.end(), driver), drivers_.end());
        for (std::map<uint32_t, LivoxDriver*>::iterator it = bound_.begin(); it != bound_.end();) {
            if (it->second == driver) {
                it = bound_.erase(it);
            } else {
                ++it;
            }
        }
    }

private:
    LivoxSdkRuntime() : ref_count_(0) {}

    LivoxSdkRuntime(const LivoxSdkRuntime&) = delete;
    LivoxSdkRuntime& operator=(const LivoxSdkRuntime&) = delete;

#ifdef PURE_NAV_HAS_LIVOX_SDK2
    static void pointCloudTrampoline(const uint32_t handle,
                                     const uint8_t dev_type,
                                     LivoxLidarEthernetPacket* data,
                                     void* client_data);
    static void imuTrampoline(const uint32_t handle,
                              const uint8_t dev_type,
                              LivoxLidarEthernetPacket* data,
                              void* client_data);
    static void infoChangeTrampoline(const uint32_t handle, const LivoxLidarInfo* info, void* client_data);
    static void pushMessageTrampoline(const uint32_t handle,
                                      const uint8_t dev_type,
                                      const char* info,
                                      void* client_data);

    static bool buildRawPacket(uint32_t handle,
                               uint8_t dev_type,
                               LivoxLidarEthernetPacket* data,
                               RawPacket& out);
    void handleInfoChange(uint32_t handle, const LivoxLidarInfo* info);
#endif

    std::mutex lifecycle_mutex_; // 串行化 acquire/release
    std::mutex registry_mutex_;  // 保护 drivers_ / bound_
    int ref_count_;
    std::string config_path_;
    std::vector<LivoxDriver*> drivers_;
    std::map<uint32_t, LivoxDriver*> bound_;
};

bool LivoxSdkRuntime::acquire(const LivoxConfig& config) {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);

    if (ref_count_ > 0) {
        if (!config_path_.empty() && config_path_ != config.config_path) {
            logWarn("SDK 已用配置 " + config_path_ + " 初始化，忽略新路径 " + config.config_path);
        }
        ++ref_count_;
        return true;
    }

#ifdef PURE_NAV_HAS_LIVOX_SDK2
    if (!LivoxLidarSdkInit(config.config_path.c_str())) {
        logError("LivoxLidarSdkInit 失败，请检查配置文件与网卡: " + config.config_path);
        return false;
    }

    // 全局回调只需要设置一次；即使后续 release 也不会清空，所以 trampoline
    // 必须能在"没有注册任何驱动"的情况下安全返回。
    SetLivoxLidarPointCloudCallBack(&LivoxSdkRuntime::pointCloudTrampoline, nullptr);
    SetLivoxLidarImuDataCallback(&LivoxSdkRuntime::imuTrampoline, nullptr);
    SetLivoxLidarInfoChangeCallback(&LivoxSdkRuntime::infoChangeTrampoline, nullptr);
    SetLivoxLidarInfoCallback(&LivoxSdkRuntime::pushMessageTrampoline, nullptr);

    if (!LivoxLidarSdkStart()) {
        logError("LivoxLidarSdkStart 失败");
        LivoxLidarSdkUninit();
        return false;
    }

    config_path_ = config.config_path;
    ref_count_ = 1;
    logInfo("Livox SDK2 初始化完成: " + config.config_path);
    return true;
#else
    (void)config;
    logError("编译时未链接 Livox-SDK2（缺少 PURE_NAV_HAS_LIVOX_SDK2）。"
             "请先执行 scripts/autoBuild.sh thirdparty 再重新 cmake。");
    return false;
#endif
}

void LivoxSdkRuntime::release() {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (ref_count_ == 0) {
        return;
    }
    --ref_count_;
    if (ref_count_ > 0) {
        return;
    }
#ifdef PURE_NAV_HAS_LIVOX_SDK2
    LivoxLidarSdkUninit();
    logInfo("Livox SDK2 已反初始化");
#endif
    config_path_.clear();
}

#ifdef PURE_NAV_HAS_LIVOX_SDK2

bool LivoxSdkRuntime::buildRawPacket(uint32_t handle,
                                     uint8_t dev_type,
                                     LivoxLidarEthernetPacket* data,
                                     RawPacket& out) {
    if (data == nullptr) {
        return false;
    }
    const std::size_t point_size = livoxPointSize(data->data_type);
    if (point_size == 0) {
        return false;
    }
    const std::size_t bytes = static_cast<std::size_t>(data->dot_num) * point_size;
    if (bytes > kLivoxMaxUdpPayload) {
        return false;
    }

    out.handle = handle;
    out.timestamp_ns = livoxReadTimestamp(data->timestamp);
    out.time_interval = data->time_interval;
    out.dot_num = data->dot_num;
    out.udp_cnt = data->udp_cnt;
    out.data_len = static_cast<uint16_t>(bytes);
    out.dev_type = dev_type;
    out.data_type = data->data_type;
    out.frame_cnt = data->frame_cnt;
    out.time_type = data->time_type;
    if (bytes > 0) {
        std::memcpy(out.payload, data->data, bytes);
    }
    return true;
}

void LivoxSdkRuntime::pointCloudTrampoline(const uint32_t handle,
                                           const uint8_t dev_type,
                                           LivoxLidarEthernetPacket* data,
                                           void* client_data) {
    (void)client_data;
    if (data == nullptr || data->data_type == kLivoxLidarImuData) {
        return;
    }

    LivoxSdkRuntime& runtime = instance();
    std::lock_guard<std::mutex> lock(runtime.registry_mutex_);
    std::map<uint32_t, LivoxDriver*>::iterator it = runtime.bound_.find(handle);
    if (it == runtime.bound_.end()) {
        return; // 尚未绑定（刚上电 / 未配置），直接丢弃
    }

    RawPacket packet; // 栈上 2KB，不分配堆内存
    if (!buildRawPacket(handle, dev_type, data, packet)) {
        return;
    }
    it->second->onRawPointPacket(packet);
}

void LivoxSdkRuntime::imuTrampoline(const uint32_t handle,
                                    const uint8_t dev_type,
                                    LivoxLidarEthernetPacket* data,
                                    void* client_data) {
    (void)client_data;
    if (data == nullptr) {
        return;
    }

    LivoxSdkRuntime& runtime = instance();
    std::lock_guard<std::mutex> lock(runtime.registry_mutex_);
    std::map<uint32_t, LivoxDriver*>::iterator it = runtime.bound_.find(handle);
    if (it == runtime.bound_.end()) {
        return;
    }

    RawPacket packet;
    if (!buildRawPacket(handle, dev_type, data, packet)) {
        return;
    }
    it->second->onRawImuPacket(packet);
}

void LivoxSdkRuntime::infoChangeTrampoline(const uint32_t handle,
                                           const LivoxLidarInfo* info,
                                           void* client_data) {
    (void)client_data;
    if (info == nullptr) {
        return;
    }
    instance().handleInfoChange(handle, info);
}

void LivoxSdkRuntime::pushMessageTrampoline(const uint32_t handle,
                                            const uint8_t dev_type,
                                            const char* info,
                                            void* client_data) {
    (void)client_data;
    if (info == nullptr) {
        return;
    }
    LivoxSdkRuntime& runtime = instance();
    std::lock_guard<std::mutex> lock(runtime.registry_mutex_);
    std::map<uint32_t, LivoxDriver*>::iterator it = runtime.bound_.find(handle);
    if (it != runtime.bound_.end()) {
        it->second->onSdkPushMessage(dev_type, std::string(info));
    }
}

void LivoxSdkRuntime::handleInfoChange(uint32_t handle, const LivoxLidarInfo* info) {
    const std::string sn = boundedString(info->sn, sizeof(info->sn));
    const std::string ip = boundedString(info->lidar_ip, sizeof(info->lidar_ip));

    std::lock_guard<std::mutex> lock(registry_mutex_);
    if (bound_.find(handle) != bound_.end()) {
        return; // 该 handle 已绑定给某个驱动
    }

    for (std::size_t i = 0; i < drivers_.size(); ++i) {
        LivoxDriver* driver = drivers_[i];
        if (driver->connected() || !driver->accepts(handle, sn.c_str())) {
            continue;
        }
        bound_[handle] = driver;
        logInfo("发现并绑定雷达: SN=" + sn + " IP=" + ip + " handle=" + toHexHandle(handle));
        driver->bindLidar(handle, info->dev_type, sn.c_str(), ip.c_str());
        return;
    }
}

#endif // PURE_NAV_HAS_LIVOX_SDK2

// ============================================================================
// LivoxDriver
// ============================================================================

LivoxDriver::LivoxDriver()
    : running_(false),
      connected_(false),
      handle_(0),
      model_(LidarModel::kUnknown),
      point_enabled_(false),
      imu_enabled_(false),
      imu_acc_is_g_(true),
      imu_gyro_is_rad_s_(true),
      frame_timeout_ms_(200),
      has_frame_(false),
      has_imu_(false),
      st_packets_received_(0),
      st_packets_dropped_(0),
      st_frames_published_(0),
      st_frames_incomplete_(0),
      st_points_published_(0),
      st_imu_packets_received_(0),
      st_imu_samples_published_(0),
      st_imu_dropped_(0) {}

LivoxDriver::~LivoxDriver() { stop(); }

bool LivoxDriver::start(const LivoxConfig& config) {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (running_.load()) {
        logWarn("start() 重复调用，已忽略");
        return false;
    }

    std::string error;
    if (!config.valid(&error)) {
        logError("配置非法: " + error);
        return false;
    }

    // ---- 保存配置（此后只读；回调线程可见性由 registerDriver 的互斥量保证）----
    config_ = config;
    g_verbose.store(config_.verbose);
    point_enabled_ = config_.enable_point_cloud;
    imu_enabled_ = config_.enable_imu;
    imu_acc_is_g_ = config_.imu_acc_is_g;
    imu_gyro_is_rad_s_ = config_.imu_gyro_is_rad_s;
    frame_timeout_ms_ = config_.frame_timeout_ms;

    connected_.store(false);
    handle_.store(0);
    model_.store(LidarModel::kUnknown);
    {
        std::lock_guard<std::mutex> lidar_lock(lidar_mutex_);
        sn_.clear();
        ip_.clear();
    }
    {
        std::lock_guard<std::mutex> latest_lock(latest_mutex_);
        latest_frame_.clear();
        latest_imu_ = ImuSample();
        has_frame_ = false;
        has_imu_ = false;
    }

    // ---- 预分配队列（运行期不再分配）----
    try {
        if (point_enabled_) {
            point_queue_.reset(config_.point_queue_capacity, OverflowPolicy::kDropOldest);
        }
        if (imu_enabled_) {
            imu_queue_.reset(config_.imu_queue_capacity, OverflowPolicy::kDropOldest);
        }
    } catch (const std::exception& exception) {
        logError(std::string("队列容量非法: ") + exception.what());
        return false;
    }
    assembler_.reset();
    assembler_.setUseSecondEcho(config_.use_second_echo);

    LivoxSdkRuntime& runtime = LivoxSdkRuntime::instance();
    if (!runtime.acquire(config_)) {
        return false;
    }

    running_.store(true);
    try {
        if (point_enabled_) {
            point_thread_ = std::thread(&LivoxDriver::pointThreadMain, this);
        }
        if (imu_enabled_) {
            imu_thread_ = std::thread(&LivoxDriver::imuThreadMain, this);
        }
    } catch (const std::exception& exception) {
        logError(std::string("创建工作线程失败: ") + exception.what());
        running_.store(false);
        runtime.release();
        return false;
    }

    // 线程就绪后再对外可见：保证"回调进入本对象时，消费线程一定在运行"
    runtime.registerDriver(this);

    logInfo("LivoxDriver 已启动（点云线程=" + std::string(point_enabled_ ? "on" : "off") +
            "，IMU 线程=" + std::string(imu_enabled_ ? "on" : "off") + "）");
    return true;
}

void LivoxDriver::stop() {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (!running_.load()) {
        return;
    }
    running_.store(false);

    // 1) 先摘除路由：返回后不会再有 SDK 回调触碰本对象
    LivoxSdkRuntime::instance().unregisterDriver(this);

    // 2) 关闭队列唤醒工作线程，并等待它们退出（退出前会冲刷最后一帧）
    if (point_enabled_) {
        point_queue_.close();
    }
    if (imu_enabled_) {
        imu_queue_.close();
    }
    if (point_thread_.joinable()) {
        point_thread_.join();
    }
    if (imu_thread_.joinable()) {
        imu_thread_.join();
    }

    // 3) 释放 SDK 引用（引用计数归零时才真正 Uninit）
    LivoxSdkRuntime::instance().release();

    connected_.store(false);
    point_enabled_ = false;
    imu_enabled_ = false;
    logInfo("LivoxDriver 已停止");
}

bool LivoxDriver::waitForLidar(int timeout_ms) {
    std::unique_lock<std::mutex> lock(lidar_mutex_);
    if (connected_.load()) {
        return true;
    }
    if (timeout_ms < 0) {
        lidar_cv_.wait(lock, [this] { return connected_.load(); });
        return true;
    }
    return lidar_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this] { return connected_.load(); });
}

std::string LivoxDriver::serialNumber() const {
    std::lock_guard<std::mutex> lock(lidar_mutex_);
    return sn_;
}

std::string LivoxDriver::lidarIp() const {
    std::lock_guard<std::mutex> lock(lidar_mutex_);
    return ip_;
}

void LivoxDriver::setPointCloudCallback(PointCloudCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    point_callback_ = callback;
}

void LivoxDriver::setImuCallback(ImuCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    imu_callback_ = callback;
}

bool LivoxDriver::latestFrame(PointCloudFrame& frame) const {
    std::lock_guard<std::mutex> lock(latest_mutex_);
    if (!has_frame_) {
        return false;
    }
    frame = latest_frame_;
    return true;
}

bool LivoxDriver::latestImu(ImuSample& sample) const {
    std::lock_guard<std::mutex> lock(latest_mutex_);
    if (!has_imu_) {
        return false;
    }
    sample = latest_imu_;
    return true;
}

LivoxStats LivoxDriver::stats() const {
    LivoxStats snapshot;
    snapshot.packets_received = st_packets_received_.load();
    snapshot.packets_dropped = st_packets_dropped_.load();
    snapshot.frames_published = st_frames_published_.load();
    snapshot.frames_incomplete = st_frames_incomplete_.load();
    snapshot.points_published = st_points_published_.load();
    snapshot.imu_packets_received = st_imu_packets_received_.load();
    snapshot.imu_samples_published = st_imu_samples_published_.load();
    snapshot.imu_dropped = st_imu_dropped_.load();
    return snapshot;
}

void LivoxDriver::resetStats() {
    st_packets_received_.store(0);
    st_packets_dropped_.store(0);
    st_frames_published_.store(0);
    st_frames_incomplete_.store(0);
    st_points_published_.store(0);
    st_imu_packets_received_.store(0);
    st_imu_samples_published_.store(0);
    st_imu_dropped_.store(0);
}

// ----------------------------------------------------------------------------
// SDK 回调线程入口（必须极轻量）
// ----------------------------------------------------------------------------

void LivoxDriver::onRawPointPacket(const RawPacket& packet) {
    st_packets_received_.fetch_add(1);
    if (!point_enabled_) {
        return;
    }
    const uint64_t dropped_before = point_queue_.dropped();
    point_queue_.push(packet);
    const uint64_t dropped_after = point_queue_.dropped();
    if (dropped_after > dropped_before) {
        st_packets_dropped_.fetch_add(dropped_after - dropped_before);
    }
}

void LivoxDriver::onRawImuPacket(const RawPacket& packet) {
    st_imu_packets_received_.fetch_add(1);
    if (!imu_enabled_) {
        return;
    }

    // IMU 包很小：直接在当前线程解码（无堆分配），只把 56B 的结果入队，
    // 避免把 2KB 的 RawPacket 搬进队列
    ImuSample samples[kMaxImuSamplesPerPacket];
    const std::size_t count =
        decodeImuPacket(packet, imu_acc_is_g_, imu_gyro_is_rad_s_, samples, kMaxImuSamplesPerPacket);
    for (std::size_t i = 0; i < count; ++i) {
        const uint64_t dropped_before = imu_queue_.dropped();
        imu_queue_.push(samples[i]);
        const uint64_t dropped_after = imu_queue_.dropped();
        if (dropped_after > dropped_before) {
            st_imu_dropped_.fetch_add(dropped_after - dropped_before);
        }
    }
}

void LivoxDriver::onSdkPushMessage(uint8_t dev_type, const std::string& info) {
    (void)dev_type;
    logInfo("雷达状态消息: " + info);
}

bool LivoxDriver::accepts(uint32_t handle, const char* sn) const {
    if (config_.handle != 0 && config_.handle != handle) {
        return false;
    }
    if (!config_.sn.empty() && config_.sn != boundedString(sn, 16)) {
        return false;
    }
    return true;
}

void LivoxDriver::bindLidar(uint32_t handle, uint8_t dev_type, const char* sn, const char* ip) {
    handle_.store(handle);
    model_.store(lidarModelFromSdkType(dev_type));
    {
        std::lock_guard<std::mutex> lock(lidar_mutex_);
        sn_ = boundedString(sn, 16);
        ip_ = boundedString(ip, 16);
        // 必须在持锁时修改谓词状态，否则 waitForLidar 可能丢唤醒而白等到超时
        connected_.store(true);
    }
    lidar_cv_.notify_all();

    configureLidar(handle);

    logInfo("雷达已就绪: SN=" + serialNumber() + " IP=" + lidarIp() +
            " 型号=" + lidarModelName(model_.load()));
}

void LivoxDriver::configureLidar(uint32_t handle) {
    if (!config_.configure_on_connect) {
        return;
    }
#ifdef PURE_NAV_HAS_LIVOX_SDK2
    // 只下发与取流直接相关的三项；机型专有参数（ESC/FOV/点频/时间滤波等）
    // 交给专门的配置流程，避免在不支持的机型上刷一堆 not supported 警告。
    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, &controlResponseCallback, nullptr);

    const uint8_t data_type = toSdkPointDataType(config_.point_data_type);
    SetLivoxLidarPclDataType(handle, static_cast<LivoxLidarPointDataType>(data_type),
                             &controlResponseCallback, nullptr);

    if (config_.enable_imu) {
        EnableLivoxLidarImuData(handle, &controlResponseCallback, nullptr);
    } else {
        DisableLivoxLidarImuData(handle, &controlResponseCallback, nullptr);
    }
#else
    (void)handle;
#endif
}

// ----------------------------------------------------------------------------
// 工作线程
// ----------------------------------------------------------------------------

void LivoxDriver::pointThreadMain() {
    PointCloudFrame frame;
    RawPacket packet;

    while (running_.load()) {
        if (!point_queue_.pop(packet, frame_timeout_ms_)) {
            if (point_queue_.closed() || !running_.load()) {
                break;
            }
            // 超时说明流中断（雷达停转/拔线），冲刷尚未闭合的半帧
            if (assembler_.flush(frame)) {
                publishFrame(frame);
            }
            continue;
        }
        if (assembler_.push(packet, frame)) {
            publishFrame(frame);
        }
    }

    // 退出前把最后一帧交出去，避免丢掉关机瞬间的数据
    if (assembler_.flush(frame)) {
        publishFrame(frame);
    }
}

void LivoxDriver::imuThreadMain() {
    ImuSample sample;
    while (running_.load()) {
        if (!imu_queue_.pop(sample, 100)) {
            if (imu_queue_.closed() || !running_.load()) {
                break;
            }
            continue;
        }
        publishImu(sample);
    }

    // 排空队列中剩余的采样
    while (imu_queue_.tryPop(sample)) {
        publishImu(sample);
    }
}

// ----------------------------------------------------------------------------
// 数据发布
// ----------------------------------------------------------------------------

namespace {

void copyFrameMetadata(const PointCloudFrame& source, PointCloudFrame& target) {
    target.handle = source.handle;
    target.frame_index = source.frame_index;
    target.timestamp_ns = source.timestamp_ns;
    target.received_packets = source.received_packets;
    target.expected_packets = source.expected_packets;
    target.complete = source.complete;
    target.dev_type = source.dev_type;
    target.data_type = source.data_type;
}

} // namespace

void LivoxDriver::publishFrame(PointCloudFrame& frame) {
    st_frames_published_.fetch_add(1);
    if (!frame.complete) {
        st_frames_incomplete_.fetch_add(1);
    }
    st_points_published_.fetch_add(frame.points.size());

    PointCloudCallback callback;
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        callback = point_callback_;
    }
    if (callback) {
        callback(frame);
    }

    // 回调之后用 swap 把点云零拷贝挪进 latest_frame_（frame 由本线程复用）
    {
        std::lock_guard<std::mutex> lock(latest_mutex_);
        latest_frame_.points.swap(frame.points);
        copyFrameMetadata(frame, latest_frame_);
        has_frame_ = true;
    }
}

void LivoxDriver::publishImu(const ImuSample& sample) {
    st_imu_samples_published_.fetch_add(1);
    {
        std::lock_guard<std::mutex> lock(latest_mutex_);
        latest_imu_ = sample;
        has_imu_ = true;
    }

    ImuCallback callback;
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        callback = imu_callback_;
    }
    if (callback) {
        callback(sample);
    }
}

} // namespace driver
} // namespace pure
