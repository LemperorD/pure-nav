// ============================================================================
// test_shm_cross_process.cpp —— 真正的跨进程共享内存验证
//
// 与 test_shm.cpp 不同，本用例不启动进程内 RouDi，而是：
//   1. fork + exec 一个真正的 iox-roudi 守护进程（路径由 CMake 注入）；
//   2. fork 出一个子进程作为订阅者（独立 iceoryx 运行时）；
//   3. 父进程作为发布者，把若干帧写进共享内存；
//   4. 子进程读出来逐帧校验，通过退出码把结果带回父进程。
//
// 只有这一步跑通，才能证明"雷达驱动写、感知模块读"这种多进程架构成立。
// 若环境中已有 RouDi 在运行导致本地 RouDi 起不来，用例会明确 SKIP 而不是失败。
// ============================================================================

#include "shm.hpp"
#include "test_check.hpp"

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <thread>

#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

namespace {

using pure::common::ShmPublishResult;
using pure::common::ShmPublisher;
using pure::common::ShmRuntime;
using pure::common::ShmSubscriber;
using pure::common::ShmTopic;

constexpr uint32_t kSampleCount = 8;
constexpr uint32_t kFloatCount = 256;
constexpr const char* kTopicName = "test/xproc/sample";

struct Sample {
    uint32_t sequence = 0;
    uint32_t count = 0;
    float data[kFloatCount] = {};
};

float expectedValue(uint32_t sequence, uint32_t index) {
    return static_cast<float>(sequence) * 1000.0F + static_cast<float>(index);
}

// --- RouDi 子进程生命周期 ---------------------------------------------------
struct RouDiProcess {
    pid_t pid = -1;

    // 用 SIGKILL 收尾：SIGTERM 会让 RouDi 反过来给所有注册进程发 SIGTERM，
    // 把正在收尾的测试进程一起杀掉。
    void stop() {
        if (pid <= 0) {
            return;
        }
        ::kill(pid, SIGKILL);
        int status = 0;
        ::waitpid(pid, &status, 0);
        pid = -1;
    }

    ~RouDiProcess() {
        stop();
    }
};

bool startRouDi(RouDiProcess& roudi) {
#ifdef PURE_NAV_IOX_ROUDI
    const pid_t pid = ::fork();
    if (pid < 0) {
        return false;
    }
    if (pid == 0) {
        ::execl(PURE_NAV_IOX_ROUDI, "iox-roudi", "-l", "off", static_cast<char*>(nullptr));
        ::_exit(127);
    }
    roudi.pid = pid;

    // 给 RouDi 一点启动时间；若它很快退出，说明端口/资源被占用（已有实例在跑）
    ::usleep(300 * 1000);
    int status = 0;
    if (::waitpid(pid, &status, WNOHANG) == pid) {
        roudi.pid = -1;
        return false;
    }
    return true;
#else
    (void)roudi;
    return false;
#endif
}

// --- 子进程：订阅者 ---------------------------------------------------------
int runSubscriber() {
    const ShmTopic topic = ShmTopic::fromString(kTopicName);
    ShmRuntime::init("pure_nav_xproc_sub");
    ShmSubscriber<Sample> subscriber(topic);

    uint32_t received = 0;
    bool content_ok = true;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
    while (received < kSampleCount && std::chrono::steady_clock::now() < deadline) {
        subscriber.drain([&](const Sample& sample) {
            if (sample.sequence != received || sample.count != kFloatCount) {
                content_ok = false;
            } else {
                for (uint32_t i = 0; i < kFloatCount; ++i) {
                    if (sample.data[i] != expectedValue(sample.sequence, i)) {
                        content_ok = false;
                        break;
                    }
                }
            }
            ++received;
        });
        if (received < kSampleCount) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    if (!content_ok) {
        return 4;  // 收到的数据内容不对
    }
    if (received != kSampleCount) {
        return 5;  // 收帧数不足（超时）
    }
    return 0;
}

// --- 父进程：发布者 ---------------------------------------------------------
void test_cross_process_pubsub() {
    const pid_t child = ::fork();
    CHECK(child >= 0);
    if (child < 0) {
        return;
    }
    if (child == 0) {
        const int code = runSubscriber();
        ::_exit(code);  // _exit：不要触发继承来的 atexit / 静态析构
    }

    ShmRuntime::init("pure_nav_xproc_pub");
    ShmPublisher<Sample> publisher(ShmTopic::fromString(kTopicName));

    // 等 RouDi 把订阅者接上（发布者侧能直接看到订阅者）
    bool connected = false;
    const auto connect_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < connect_deadline) {
        if (publisher.hasSubscribers()) {
            connected = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    CHECK(connected);

    if (connected) {
        for (uint32_t sequence = 0; sequence < kSampleCount; ++sequence) {
            const ShmPublishResult result = publisher.publish([&](Sample& sample) {
                sample.sequence = sequence;
                sample.count = kFloatCount;
                for (uint32_t i = 0; i < kFloatCount; ++i) {
                    sample.data[i] = expectedValue(sequence, i);
                }
            });
            CHECK(result == ShmPublishResult::OK);
        }
    }

    int status = 0;
    CHECK(::waitpid(child, &status, 0) == child);
    CHECK(WIFEXITED(status));
    if (WIFEXITED(status)) {
        CHECK(WEXITSTATUS(status) == 0);
    }
}

} // namespace

int main() {
#ifndef PURE_NAV_IOX_ROUDI
    std::cout << "[SKIP] 未配置 iox-roudi 路径，跨进程共享内存测试跳过" << std::endl;
    return 0;
#else
    RouDiProcess roudi;
    if (!startRouDi(roudi)) {
        std::cout << "[SKIP] iox-roudi 无法启动（可能已有实例在运行），跨进程共享内存测试跳过" << std::endl;
        return 0;
    }

    pure_nav_test::run("cross_process_pubsub", test_cross_process_pubsub);

    const int exit_code = pure_nav_test::summary();
    std::cout.flush();
    std::cerr.flush();
    roudi.stop();
    // 用 _exit 跳过进程静态析构：RouDi 已经退出，iceoryx 运行时的析构函数
    // 只会往已消失的 RouDi 发 TERMINATION 并打一条无害的 Error 日志。
    ::_exit(exit_code);
#endif
}
