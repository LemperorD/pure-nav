#include "shm.hpp"

#include "shm_internal.hpp"

#include <fstream>
#include <stdexcept>
#include <string>

#include <iceoryx_posh/runtime/posh_runtime.hpp>

namespace pure {
namespace common {

namespace {

constexpr std::size_t kMaxTopicSegmentLength = static_cast<std::size_t>(iox::capro::IdString_t::capacity());
constexpr std::size_t kMaxRuntimeNameLength = static_cast<std::size_t>(iox::RuntimeName_t::capacity());

std::string trimmed(std::string value) {
    while (!value.empty() && (value.back() == '\n' || value.back() == '\r' || value.back() == ' ')) {
        value.pop_back();
    }
    return value;
}

iox::capro::IdString_t toIdString(const std::string& value, const char* what) {
    iox::capro::IdString_t result;
    if (!result.unsafe_assign(value.c_str())) {
        throw std::invalid_argument(std::string("共享内存主题") + what + "过长（上限 " + std::to_string(kMaxTopicSegmentLength)
                                    + " 字符）：" + value);
    }
    return result;
}

// 进程级唯一：是否已经初始化过 iceoryx 运行时
bool& runtimeFlag() noexcept {
    static bool initialized = false;
    return initialized;
}

} // namespace

// ============================================================================
// ShmTopic
// ============================================================================
ShmTopic::ShmTopic(std::string service, std::string instance, std::string event)
    : service_(std::move(service))
    , instance_(std::move(instance))
    , event_(std::move(event)) {
    if (!valid()) {
        throw std::invalid_argument("ShmTopic 非法：service/instance/event 均不能为空，且每段不超过 "
                                    + std::to_string(kMaxTopicSegmentLength) + " 字符");
    }
}

ShmTopic ShmTopic::fromString(const std::string& name) {
    const std::size_t first = name.find('/');
    if (first == std::string::npos) {
        throw std::invalid_argument("ShmTopic::fromString 需要 \"service/instance/event\" 三段，实际：" + name);
    }
    const std::size_t second = name.find('/', first + 1);
    if (second == std::string::npos) {
        throw std::invalid_argument("ShmTopic::fromString 需要 \"service/instance/event\" 三段，实际：" + name);
    }
    if (name.find('/', second + 1) != std::string::npos) {
        throw std::invalid_argument("ShmTopic::fromString 段数过多，实际：" + name);
    }
    return ShmTopic(name.substr(0, first), name.substr(first + 1, second - first - 1), name.substr(second + 1));
}

std::string ShmTopic::name() const {
    return service_ + "/" + instance_ + "/" + event_;
}

bool ShmTopic::valid() const noexcept {
    const auto segment_ok = [](const std::string& segment) {
        return !segment.empty() && segment.size() <= kMaxTopicSegmentLength;
    };
    return segment_ok(service_) && segment_ok(instance_) && segment_ok(event_);
}

bool ShmTopic::operator==(const ShmTopic& rhs) const noexcept {
    return service_ == rhs.service_ && instance_ == rhs.instance_ && event_ == rhs.event_;
}

bool ShmTopic::operator<(const ShmTopic& rhs) const noexcept {
    if (service_ != rhs.service_) {
        return service_ < rhs.service_;
    }
    if (instance_ != rhs.instance_) {
        return instance_ < rhs.instance_;
    }
    return event_ < rhs.event_;
}

// ============================================================================
// ShmRuntime
// ============================================================================
void ShmRuntime::init(const std::string& app_name) {
    if (detail::runtimeInitialized()) {
        return;
    }
    const std::string effective_name = app_name.empty() ? defaultAppName() : app_name;
    iox::runtime::PoshRuntime::initRuntime(detail::makeRuntimeName(effective_name));
    detail::markRuntimeInitialized();
}

bool ShmRuntime::initialized() noexcept {
    return detail::runtimeInitialized();
}

std::string ShmRuntime::defaultAppName() {
    std::string name;
    std::ifstream comm("/proc/self/comm");
    if (comm.is_open()) {
        std::getline(comm, name);
        name = trimmed(std::move(name));
    }
    if (name.empty()) {
        name = "pure_nav_shm";
    }
    if (name.size() > kMaxRuntimeNameLength) {
        name.resize(kMaxRuntimeNameLength);
    }
    return name;
}

// ============================================================================
// 结果码转字符串
// ============================================================================
const char* toString(ShmPublishResult result) noexcept {
    switch (result) {
    case ShmPublishResult::OK:
        return "OK";
    case ShmPublishResult::NO_MEMPOOL:
        return "NO_MEMPOOL（没有装得下该消息的内存池，请调大 RouDi chunk_size）";
    case ShmPublishResult::OUT_OF_CHUNKS:
        return "OUT_OF_CHUNKS（内存池耗尽，发布过快或 chunk_count 太小）";
    case ShmPublishResult::TOO_MANY_LOANS:
        return "TOO_MANY_LOANS（同一 publisher 同时借出的 chunk 过多）";
    case ShmPublishResult::INVALID_PARAMETER:
        return "INVALID_PARAMETER（消息过大或参数非法）";
    case ShmPublishResult::ERROR:
    default:
        return "ERROR（未分类错误）";
    }
}

const char* toString(ShmTakeResult result) noexcept {
    switch (result) {
    case ShmTakeResult::OK:
        return "OK";
    case ShmTakeResult::EMPTY:
        return "EMPTY（当前没有新数据）";
    case ShmTakeResult::TOO_MANY_HELD:
        return "TOO_MANY_HELD（同时持有的样本数超过上限）";
    case ShmTakeResult::ERROR:
    default:
        return "ERROR（未分类错误）";
    }
}

// ============================================================================
// 选项 / 服务描述转换
// ============================================================================
namespace detail {

iox::capro::ServiceDescription makeServiceDescription(const ShmTopic& topic) {
    return iox::capro::ServiceDescription(toIdString(topic.service(), " service"), toIdString(topic.instance(), " instance"),
                                          toIdString(topic.event(), " event"));
}

iox::popo::PublisherOptions makePublisherOptions(const ShmPublisherConfig& config) noexcept {
    iox::popo::PublisherOptions options;
    options.historyCapacity = config.history_capacity;
    options.offerOnCreate = config.offer_on_create;
    return options;
}

iox::popo::SubscriberOptions makeSubscriberOptions(const ShmSubscriberConfig& config) noexcept {
    iox::popo::SubscriberOptions options;
    if (config.queue_capacity > 0) {
        options.queueCapacity = config.queue_capacity;
    }
    options.historyRequest = config.history_request;
    options.subscribeOnCreate = config.subscribe_on_create;
    return options;
}

bool runtimeInitialized() noexcept {
    return runtimeFlag();
}

void markRuntimeInitialized() noexcept {
    runtimeFlag() = true;
}

iox::RuntimeName_t makeRuntimeName(const std::string& name) {
    iox::RuntimeName_t result;
    if (!result.unsafe_assign(name.c_str())) {
        throw std::invalid_argument("共享内存进程名过长（上限 " + std::to_string(kMaxRuntimeNameLength) + " 字符）：" + name);
    }
    return result;
}

} // namespace detail

} // namespace common
} // namespace pure
