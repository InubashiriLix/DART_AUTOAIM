#pragma once

#include <array>
#include <atomic>
#include <optional>

template <class T, std::size_t N>
class SPSCQueue {
    static_assert((N & (N - 1)) == 0, "N must be power of two for masking");

   public:
    [[nodiscard]] bool push(T v) {
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        const std::size_t head = head_.load(std::memory_order_acquire);
        if (tail - head == N) return false;
        buf_[tail & mask_] = std::move(v);
        tail_.store(tail + 1, std::memory_order_release);
        return true;
    }

    [[nodiscard]] bool pop(T& out) {
        const std::size_t head = head_.load(std::memory_order_relaxed);
        const std::size_t tail = tail_.load(std::memory_order_acquire);
        if (tail == head) return false;
        out = std::move(buf_[head & mask_]);
        head_.store(head + 1, std::memory_order_release);
        return true;
    }

    [[nodiscard]] bool try_pop(T& out) { return pop(out); }

    bool empty() const { return size() == 0; }
    bool full() const { return size() == N; }
    std::size_t size() const {
        const auto t = tail_.load(std::memory_order_relaxed);
        const auto h = head_.load(std::memory_order_acquire);
        return t - h;
    }
    static constexpr std::size_t capacity() { return N; }

    void clear() {
        head_.store(0, std::memory_order_relaxed);
        tail_.store(0, std::memory_order_relaxed);
    }

   private:
    static constexpr std::size_t mask_ = N - 1;

    alignas(64) std::atomic<std::size_t> head_{0};
    alignas(64) std::atomic<std::size_t> tail_{0};
    alignas(64) std::array<T, N> buf_{};
};
