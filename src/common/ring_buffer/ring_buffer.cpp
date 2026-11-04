#include "ring_buffer.hpp"

#include <algorithm>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>

namespace pure::common::detail {
namespace {

std::size_t storage_size(std::size_t capacity, std::size_t element_size) {
    if (capacity == 0 || element_size == 0) {
        throw std::invalid_argument("RingBuffer capacity and element size must be positive");
    }
    if (capacity > std::numeric_limits<std::size_t>::max() / element_size) {
        throw std::length_error("RingBuffer storage size overflow");
    }
    return capacity * element_size;
}

// 避免 position + count 在极大容量时溢出；count 不超过 capacity。
std::size_t advance(std::size_t position, std::size_t count, std::size_t capacity) {
    const auto until_end = capacity - position;
    return count < until_end ? position + count : count - until_end;
}

} // namespace

RingBufferStorage::RingBufferStorage(std::size_t capacity, std::size_t element_size)
    : buffer_(storage_size(capacity, element_size)),
      capacity_(capacity), element_size_(element_size) {}

RingBufferStorage::RingBufferStorage(RingBufferStorage&& other) noexcept
    : buffer_(std::move(other.buffer_)),
      capacity_(std::exchange(other.capacity_, 0)),
      element_size_(other.element_size_),
      read_pos_(std::exchange(other.read_pos_, 0)),
      write_pos_(std::exchange(other.write_pos_, 0)),
      occupied_(std::exchange(other.occupied_, 0)) {}

RingBufferStorage& RingBufferStorage::operator=(RingBufferStorage&& other) noexcept {
    if (this != &other) {
        buffer_ = std::move(other.buffer_);
        capacity_ = std::exchange(other.capacity_, 0);
        element_size_ = other.element_size_;
        read_pos_ = std::exchange(other.read_pos_, 0);
        write_pos_ = std::exchange(other.write_pos_, 0);
        occupied_ = std::exchange(other.occupied_, 0);
    }
    return *this;
}

std::size_t RingBufferStorage::write(const void* data, std::size_t count) {
    if (count != 0 && data == nullptr) {
        throw std::invalid_argument("RingBuffer write source is null");
    }
    count = std::min(count, free());
    if (count == 0) {
        return 0;
    }
    const auto first = std::min(count, capacity_ - write_pos_);
    const auto* source = static_cast<const unsigned char*>(data);
    std::memcpy(buffer_.data() + write_pos_ * element_size_, source, first * element_size_);
    if (first < count) {
        std::memcpy(buffer_.data(), source + first * element_size_,
                    (count - first) * element_size_);
    }
    write_pos_ = advance(write_pos_, count, capacity_);
    occupied_ += count;
    return count;
}

std::size_t RingBufferStorage::read(void* destination, std::size_t count) {
    if (count != 0 && destination == nullptr) {
        throw std::invalid_argument("RingBuffer read destination is null");
    }
    count = std::min(count, occupied_);
    if (count == 0) {
        return 0;
    }
    const auto first = std::min(count, capacity_ - read_pos_);
    auto* target = static_cast<unsigned char*>(destination);
    std::memcpy(target, buffer_.data() + read_pos_ * element_size_, first * element_size_);
    if (first < count) {
        std::memcpy(target + first * element_size_, buffer_.data(),
                    (count - first) * element_size_);
    }
    read_pos_ = advance(read_pos_, count, capacity_);
    occupied_ -= count;
    return count;
}

std::size_t RingBufferStorage::capacity() const noexcept { return capacity_; }
std::size_t RingBufferStorage::occupied() const noexcept { return occupied_; }
std::size_t RingBufferStorage::free() const noexcept { return capacity_ - occupied_; }

void RingBufferStorage::clear() noexcept {
    read_pos_ = 0;
    write_pos_ = 0;
    occupied_ = 0;
}

} // namespace pure::common::detail
