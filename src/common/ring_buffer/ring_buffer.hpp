#pragma once

#include <cstddef>
#include <type_traits>
#include <vector>

namespace pure::common {
namespace detail {

// 按元素计数的字节存储；不在共享内存中放置此对象。
class RingBufferStorage {
public:
    RingBufferStorage(std::size_t capacity, std::size_t element_size);
    RingBufferStorage(const RingBufferStorage&) = default;
    RingBufferStorage& operator=(const RingBufferStorage&) = default;
    RingBufferStorage(RingBufferStorage&& other) noexcept;
    RingBufferStorage& operator=(RingBufferStorage&& other) noexcept;
    ~RingBufferStorage() = default;

    std::size_t write(const void* data, std::size_t count);
    std::size_t read(void* destination, std::size_t count);
    std::size_t capacity() const noexcept;
    std::size_t occupied() const noexcept;
    std::size_t free() const noexcept;
    void clear() noexcept;

private:
    std::vector<unsigned char> buffer_;
    std::size_t capacity_;
    std::size_t element_size_;
    std::size_t read_pos_ = 0;
    std::size_t write_pos_ = 0;
    std::size_t occupied_ = 0;
};

} // namespace detail

// 固定容量 FIFO，构造后读写不分配内存。容量、返回值均以 T 的元素个数计。
// 满时只写入剩余空间，不覆盖旧数据；空时返回 0。
// 非线程安全：跨线程访问（包括查询）需要调用方统一加锁。
// 非 IPC 对象：自身存储位于进程堆上，不能直接放入 mmap 使用。
template <typename T>
class RingBuffer {
    static_assert(std::is_trivially_copyable<T>::value,
                  "RingBuffer<T> requires trivially copyable elements");
    static_assert(!std::is_const<T>::value && !std::is_volatile<T>::value,
                  "RingBuffer<T> requires an unqualified element type");

public:
    // capacity 必须大于 0；无效容量或空指针参数会抛出标准异常。
    explicit RingBuffer(std::size_t capacity) : storage_(capacity, sizeof(T)) {}

    // 值语义：拷贝后独立持有数据；移动后源对象容量为 0，可查询、清空、赋值。
    std::size_t write(const T* data, std::size_t count) {
        return storage_.write(data, count);
    }
    std::size_t read(T* destination, std::size_t count) {
        return storage_.read(destination, count);
    }
    std::size_t capacity() const noexcept { return storage_.capacity(); }
    std::size_t getOccupied() const noexcept { return storage_.occupied(); }
    std::size_t getFree() const noexcept { return storage_.free(); }
    void clear() noexcept { storage_.clear(); }

private:
    detail::RingBufferStorage storage_;
};

} // namespace pure::common

// 兼容原有 ringbuffer<T> 名称。
template <typename T>
using ringbuffer = pure::common::RingBuffer<T>;
