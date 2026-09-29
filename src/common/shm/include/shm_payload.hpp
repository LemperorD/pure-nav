#pragma once

// ============================================================================
// shm_payload.hpp —— 放进共享内存的变长数据容器
//
// iceoryx 的消息类型必须是定长、可平凡复制的 POD，不能直接用 std::vector /
// std::string。这里提供一个"定容变长"的 FixedVector<T, Capacity>：容量在编译期
// 固定，运行时只记录有效长度，从而既满足 iceoryx 的零拷贝要求，又能表达
// 点云点数可变、图像尺寸可变这类需求。
//
// 用法（点云示例）：
//     struct PointXYZI { float x, y, z, intensity; };
//     using PointCloudMsg = pure::common::FixedVector<PointXYZI, 20000>;
//
// 注意：Capacity 直接决定消息大小，也就决定了 RouDi 需要预留多大的内存池
// （chunk），两者必须匹配，否则发布会返回 ShmPublishResult::NO_MEMPOOL。
// ============================================================================

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>

namespace pure {
namespace common {

template <typename T, std::size_t Capacity>
struct FixedVector {
    static_assert(Capacity > 0, "FixedVector 的容量必须大于 0");
    static_assert(std::is_trivially_copyable<T>::value, "FixedVector 的元素类型必须可平凡复制");

    using value_type = T;
    using iterator = T*;
    using const_iterator = const T*;

    std::array<T, Capacity> items{};
    uint32_t size = 0;

    constexpr std::size_t capacity() const noexcept {
        return Capacity;
    }

    std::size_t length() const noexcept {
        return size;
    }

    bool empty() const noexcept {
        return size == 0;
    }

    bool full() const noexcept {
        return size >= Capacity;
    }

    void clear() noexcept {
        size = 0;
    }

    T* data() noexcept {
        return items.data();
    }

    const T* data() const noexcept {
        return items.data();
    }

    iterator begin() noexcept {
        return items.data();
    }

    const_iterator begin() const noexcept {
        return items.data();
    }

    iterator end() noexcept {
        return items.data() + size;
    }

    const_iterator end() const noexcept {
        return items.data() + size;
    }

    T& operator[](std::size_t index) noexcept {
        return items[index];
    }

    const T& operator[](std::size_t index) const noexcept {
        return items[index];
    }

    T& front() noexcept {
        return items[0];
    }

    const T& front() const noexcept {
        return items[0];
    }

    T& back() noexcept {
        return items[size - 1];
    }

    const T& back() const noexcept {
        return items[size - 1];
    }

    // 追加一个元素；已满时返回 false（这是"定容"的边界，由调用方决定如何处理）。
    bool push_back(const T& value) noexcept {
        if (full()) {
            return false;
        }
        items[size++] = value;
        return true;
    }

    template <typename... Args>
    bool emplace_back(Args&&... args) noexcept {
        if (full()) {
            return false;
        }
        items[size++] = T(std::forward<Args>(args)...);
        return true;
    }

    // 只调整有效长度（不构造 / 不析构元素）。n 超过容量时会被截断到容量。
    void resize(std::size_t count) noexcept {
        size = static_cast<uint32_t>(count > Capacity ? Capacity : count);
    }
};

} // namespace common
} // namespace pure
