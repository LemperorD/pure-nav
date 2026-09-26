// ============================================================================
// blocking_queue.hpp —— 单生产者/单消费者有界阻塞队列
//
// 为什么不用 std::queue + mutex 现成写法：
//   1. 底层是**预分配环形缓冲**（std::vector<T> 在 reset() 时一次性 resize），
//      运行期 push/pop 不发生堆分配。这对"在 SDK 回调线程里入队"很关键：
//      回调路径必须是常数时间，不能因为 malloc 抖动而拖慢 SDK 的收包线程。
//   2. push() **永不等待空位**，队列满时按策略丢包并计数，绝不阻塞 SDK 线程；
//      pop() 支持超时，便于工作线程在流中断时冲刷半帧或退出。
//   3. close() 会唤醒所有等待者，让 join 一定能返回。
//
// 命名与 filters.hpp 保持一致：容量非法时抛 std::invalid_argument。
// ============================================================================

#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <vector>

namespace pure {
namespace driver {

// 队列满时的丢弃策略
enum class OverflowPolicy {
    kDropOldest, // 丢最旧的样本（传感器流常用：保证拿到最新数据）
    kDropNewest, // 丢刚到的样本（保证已有数据完整）
};

template <typename T>
class BlockingQueue {
public:
    // 默认容量 0：此时 push 恒返回 false 并计入丢弃，pop 只能超时/被 close 唤醒
    BlockingQueue()
        : head_(0), tail_(0), size_(0), capacity_(0), dropped_(0), closed_(false),
          policy_(OverflowPolicy::kDropOldest) {}

    explicit BlockingQueue(std::size_t capacity, OverflowPolicy policy = OverflowPolicy::kDropOldest)
        : head_(0), tail_(0), size_(0), capacity_(0), dropped_(0), closed_(false), policy_(policy) {
        reset(capacity, policy);
    }

    ~BlockingQueue() { close(); }

    BlockingQueue(const BlockingQueue&) = delete;
    BlockingQueue& operator=(const BlockingQueue&) = delete;

    // 重新分配并清空。**只能在确认没有其它线程访问时调用**（例如 start() 之前）。
    void reset(std::size_t capacity, OverflowPolicy policy = OverflowPolicy::kDropOldest) {
        if (capacity == 0) {
            throw std::invalid_argument("BlockingQueue: capacity 必须大于 0");
        }
        std::lock_guard<std::mutex> lock(mutex_);
        buffer_.clear();
        buffer_.resize(capacity);
        head_ = 0;
        tail_ = 0;
        size_ = 0;
        capacity_ = capacity;
        dropped_ = 0;
        closed_ = false;
        policy_ = policy;
    }

    // 入队。永不等待空位；返回 true 表示 item 已入队（可能顺带挤掉了最旧样本）。
    bool push(const T& item) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (closed_ || capacity_ == 0) {
            ++dropped_;
            return false;
        }

        if (size_ == capacity_) {
            ++dropped_;
            if (policy_ == OverflowPolicy::kDropNewest) {
                return false;
            }
            // 丢最旧：覆盖写 tail_ 位置，然后整体前移
            buffer_[tail_] = item;
            tail_ = (tail_ + 1) % capacity_;
            head_ = (head_ + 1) % capacity_;
            lock.unlock();
            not_empty_.notify_one();
            return true;
        }

        buffer_[tail_] = item;
        tail_ = (tail_ + 1) % capacity_;
        ++size_;
        lock.unlock();
        not_empty_.notify_one();
        return true;
    }

    // 出队。timeout_ms < 0 表示无限等待；返回 false 表示超时或队列已关闭且为空。
    bool pop(T& out, int timeout_ms) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (timeout_ms < 0) {
            not_empty_.wait(lock, [this] { return size_ > 0 || closed_; });
        } else if (!not_empty_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                                        [this] { return size_ > 0 || closed_; })) {
            return false;
        }

        if (size_ == 0) {
            return false; // 已关闭且排空
        }
        out = buffer_[head_];
        head_ = (head_ + 1) % capacity_;
        --size_;
        return true;
    }

    // 非阻塞出队
    bool tryPop(T& out) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (size_ == 0) {
            return false;
        }
        out = buffer_[head_];
        head_ = (head_ + 1) % capacity_;
        --size_;
        return true;
    }

    void close() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            closed_ = true;
        }
        not_empty_.notify_all();
    }

    bool closed() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return closed_;
    }

    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return size_ == 0;
    }

    std::size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return size_;
    }

    std::size_t capacity() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return capacity_;
    }

    uint64_t dropped() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return dropped_;
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable not_empty_;
    std::vector<T> buffer_; // 预分配环形缓冲，size == capacity_
    std::size_t head_;      // 最旧元素下标
    std::size_t tail_;      // 下一个写入位置
    std::size_t size_;
    std::size_t capacity_;
    uint64_t dropped_;
    bool closed_;
    OverflowPolicy policy_;
};

} // namespace driver
} // namespace pure
