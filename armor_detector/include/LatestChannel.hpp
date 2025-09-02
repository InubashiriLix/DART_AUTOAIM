#pragma once
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <utility>

template <typename T>

class LatestChannel {
   public:
    struct Ticket {
        uint64_t seq = 0;
    };

    LatestChannel() noexcept = default;
    void publish(const std::shared_ptr<const T>& v) noexcept { publish_impl(v); }

    void publish(std::shared_ptr<const T>&& v) noexcept { publish_impl(std::move(v)); }

    std::shared_ptr<const T> get() const noexcept {
        return std::atomic_load_explicit(&ptr_, std::memory_order_acquire);
    }

    Ticket ticket() const noexcept { return Ticket{seq_.load(std::memory_order_acquire)}; }

    std::pair<std::shared_ptr<const T>, Ticket> wait_next(Ticket t) {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait(lk, [&] { return seq_.load(std::memory_order_acquire) != t.seq; });
        Ticket nt{seq_.load(std::memory_order_relaxed)};
        lk.unlock();
        return {std::atomic_load_explicit(&ptr_, std::memory_order_acquire), nt};
    }

    template <class Rep, class Period>
    std::optional<std::pair<std::shared_ptr<const T>, Ticket>> wait_next_for(
        Ticket t, const std::chrono::duration<Rep, Period>& d) {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait(lk, [&] { return seq_.load(std::memory_order_acquire) != t.seq; });
        Ticket nt{seq_.load(std::memory_order_relaxed)};
        lk.unlock();
        return {std::atomic_load_explicit(&ptr_, std::memory_order_acquire), nt};
    }

   private:
    template <class Ptr>
    void publish_impl(Ptr&& v) noexcept {
        std::atomic_store_explicit(&ptr_, std::forward<Ptr>(v), std::memory_order_release);

        {
            std::lock_guard<std::mutex> lk(m_);
            ++seq_;
        }
        cv_.notify_all();
    }

    mutable std::mutex m_;
    std::condition_variable cv_;
    std::atomic<uint64_t> seq_;
    std::shared_ptr<const T> ptr_{};
};
