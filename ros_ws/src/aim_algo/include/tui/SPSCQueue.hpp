#pragma once

#include <array>
#include <atomic>
#include <optional>

template <class T, std::size_t N>
class SPSCQueue {
    static_assert((N & (N - 1)) == 0, "N must be power of two for masking");

   public:
    bool push(T v) {}

    bool pop(T& out) {}

   private:
    static constexpr std::size_t mask_ = N - 1;
    alignas(64) std::atomic<std::size_t> head_{0};
    alignas(64) std::atomic<std::size_t> tail_{0};
    alignas(64) std::array<T, N> buf_;
};
