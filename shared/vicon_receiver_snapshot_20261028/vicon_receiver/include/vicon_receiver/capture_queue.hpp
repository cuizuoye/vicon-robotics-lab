#pragma once
#include <array>
#include <atomic>
#include <cstdint>
#include <cstddef>
#include <type_traits>

namespace vicon_capture {
// Fixed-size snapshots; no strings, allocations, disk I/O or wait in producer.
struct Frame {
    uint64_t sequence{}, sdk_frame{}, sdk_epoch{}, missing_frames{}, queue_dropped{};
    uint64_t begin_ns{}, end_ns{};
    int64_t received_ros_ns{}, source_ros_ns{};
    double hz{}, sdk_pose[7]{}, world_pose[7]{}; // xyz + xyzw
    bool sdk_ok{}, occluded{}, pose_published{};
};

template <std::size_t N> class Queue {
    static_assert(N > 1);
    static_assert(std::is_trivially_copyable<Frame>::value);
    static_assert(std::atomic<std::size_t>::is_always_lock_free);
    static_assert(std::atomic<uint64_t>::is_always_lock_free);
    std::array<Frame, N> rows_{};
    std::atomic<std::size_t> head_{0}, tail_{0};
    std::atomic<uint64_t> dropped_{0};
public:
    bool push(Frame row) {
        const auto head = head_.load(std::memory_order_relaxed);
        const auto next = (head + 1) % N;
        if (next == tail_.load(std::memory_order_acquire)) {
            dropped_.fetch_add(1, std::memory_order_relaxed); return false;
        }
        row.queue_dropped = dropped_.load(std::memory_order_relaxed);
        rows_[head] = row;
        head_.store(next, std::memory_order_release);
        return true;
    }
    bool pop(Frame &row) {
        const auto tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) return false;
        row = rows_[tail];
        // Also reveal terminal overflow, even if no later SDK frame is pushed.
        row.queue_dropped = dropped_.load(std::memory_order_relaxed);
        tail_.store((tail + 1) % N, std::memory_order_release);
        return true;
    }
};
} // namespace vicon_capture
