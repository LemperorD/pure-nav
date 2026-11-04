#include "ring_buffer.hpp"
#include "test_check.hpp"

#include <algorithm>
#include <array>
#include <deque>
#include <limits>
#include <random>
#include <utility>

namespace {

template <typename Exception, typename Function>
void check_throws(Function function) {
    bool caught = false;
    try {
        function();
    } catch (const Exception&) {
        caught = true;
    }
    CHECK(caught);
}

void boundaries() {
    check_throws<std::invalid_argument>([] { ringbuffer<int> invalid(0); });
    check_throws<std::length_error>([] {
        ringbuffer<int> invalid(std::numeric_limits<std::size_t>::max());
    });
    const int input[] = {10, 20, 30, 40};
    int output[4] = {};
    ringbuffer<int> ring(1);
    CHECK(ring.capacity() == 1);
    CHECK(ring.read(output, 4) == 0);
    CHECK(ring.write(nullptr, 0) == 0);
    CHECK(ring.read(nullptr, 0) == 0);
    check_throws<std::invalid_argument>([&] { ring.write(nullptr, 1); });
    check_throws<std::invalid_argument>([&] { ring.read(nullptr, 1); });
    CHECK(ring.write(input, 4) == 1);
    CHECK(ring.write(input + 1, 1) == 0);
    CHECK(ring.getOccupied() == 1);
    CHECK(ring.getFree() == 0);
    CHECK(ring.read(output, 4) == 1);
    CHECK(output[0] == 10);
    CHECK(ring.write(input + 1, 1) == 1);
    CHECK(ring.read(output, 1) == 1);
    CHECK(output[0] == 20);
    ring.clear();
    CHECK(ring.getFree() == 1);
}

void wrap_and_value_semantics() {
    ringbuffer<int> ring(4);
    const int input[] = {1, 2, 3, 4, 5, 6};
    int output[4] = {};
    CHECK(ring.write(input, 4) == 4);
    auto full_copy = ring;
    CHECK(full_copy.getOccupied() == 4);
    CHECK(full_copy.read(output, 4) == 4);
    CHECK(std::equal(output, output + 4, input));
    CHECK(ring.getOccupied() == 4);
    CHECK(ring.read(output, 2) == 2);
    CHECK(ring.write(input + 4, 2) == 2);

    auto copy = ring;
    ringbuffer<int> assigned(1);
    assigned = ring;
    ring = ring; // 自赋值保持内容
    CHECK(ring.read(output, 4) == 4);
    CHECK(std::equal(output, output + 4, input + 2));
    CHECK(copy.read(output, 4) == 4);
    CHECK(std::equal(output, output + 4, input + 2));
    CHECK(assigned.capacity() == 4);

    auto moved = std::move(assigned);
    CHECK(assigned.capacity() == 0);
    CHECK(assigned.getFree() == 0);
    CHECK(assigned.read(output, 4) == 0);
    CHECK(assigned.write(input, 4) == 0);
    assigned.clear();
    CHECK(moved.read(output, 4) == 4);
    CHECK(std::equal(output, output + 4, input + 2));
    CHECK(ring.write(input, 3) == 3);
    assigned = std::move(ring);
    CHECK(ring.capacity() == 0);
    CHECK(assigned.read(output, 4) == 3);
    CHECK(std::equal(output, output + 3, input));
    ring = full_copy;
    CHECK(ring.capacity() == 4);
    CHECK(ring.write(input, 4) == 4);
    ring.clear();
    CHECK(ring.getOccupied() == 0);
    CHECK(ring.getFree() == 4);

    struct Sample { double time; int value; };
    pure::common::RingBuffer<Sample> samples(2);
    const Sample in[] = {{1.25, 7}, {2.5, 8}};
    Sample out[2] = {};
    CHECK(samples.write(in, 2) == 2);
    CHECK(samples.read(out, 2) == 2);
    CHECK(out[0].time == 1.25 && out[0].value == 7);
    CHECK(out[1].time == 2.5 && out[1].value == 8);
}

void compare_with_deque() {
    std::mt19937 random(42);
    for (const std::size_t capacity : {1u, 2u, 3u, 7u, 16u}) {
        ringbuffer<int> ring(capacity);
        std::deque<int> expected;
        for (int iteration = 0; iteration < 3000; ++iteration) {
            const std::size_t requested = random() % 24;
            std::array<int, 24> data{};
            if (random() % 2 == 0) {
                for (std::size_t i = 0; i < requested; ++i) {
                    data[i] = static_cast<int>(random() % 10000);
                }
                const auto count = std::min(requested, capacity - expected.size());
                CHECK(ring.write(data.data(), requested) == count);
                expected.insert(expected.end(), data.begin(), data.begin() + count);
            } else {
                const auto count = std::min(requested, expected.size());
                CHECK(ring.read(data.data(), requested) == count);
                for (std::size_t i = 0; i < count; ++i) {
                    CHECK(data[i] == expected.front());
                    expected.pop_front();
                }
            }
            const auto& view = ring;
            CHECK(view.getOccupied() == expected.size());
            CHECK(view.getFree() == capacity - expected.size());
        }
    }
}

} // namespace

int main() {
    pure_nav_test::run("capacity / null / full / empty", boundaries);
    pure_nav_test::run("wrap / copy / move / clear / POD", wrap_and_value_semantics);
    pure_nav_test::run("random FIFO against std::deque", compare_with_deque);
    return pure_nav_test::summary();
}
