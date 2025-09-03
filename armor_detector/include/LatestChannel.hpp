// LatestChannel.hpp
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

    LatestChannel() noexcept : seq_(0) {}

    // 发布（支持左值/右值 shared_ptr）
    void publish(const std::shared_ptr<const T>& v) noexcept { publish_impl(v); }
    void publish(std::shared_ptr<const T>&& v) noexcept { publish_impl(std::move(v)); }

    // 获取当前最新（shared_ptr）
    std::shared_ptr<const T> get() const noexcept {
        return std::atomic_load_explicit(&ptr_, std::memory_order_acquire);
    }
    std::shared_ptr<const T> get_ptr() const noexcept { return get(); }

    // 拷贝出一份 T（方便结构化绑定：auto [a,b] = *chan.get_copy();）
    std::optional<T> get_copy() const noexcept {
        auto p = get();
        if (!p) return std::nullopt;
        return *p;  // 注意：对 cv::Mat 来说是“头拷贝”，像素仍共享；如需独立像素请 clone()
    }

    // 发票（用于等待下一条）
    Ticket ticket() const noexcept { return Ticket{seq_.load(std::memory_order_acquire)}; }

    // 阻塞等待下一条
    std::pair<std::shared_ptr<const T>, Ticket> wait_next(Ticket t) {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait(lk, [&] { return seq_.load(std::memory_order_acquire) != t.seq; });
        Ticket nt{seq_.load(std::memory_order_relaxed)};
        lk.unlock();
        return {std::atomic_load_explicit(&ptr_, std::memory_order_acquire), nt};
    }

    // 带超时等待
    template <class Rep, class Period>
    std::optional<std::pair<std::shared_ptr<const T>, Ticket>> wait_next_for(
        Ticket t, const std::chrono::duration<Rep, Period>& d) {
        std::unique_lock<std::mutex> lk(m_);
        bool ok =
            cv_.wait_for(lk, d, [&] { return seq_.load(std::memory_order_acquire) != t.seq; });
        if (!ok) return std::nullopt;  // 超时
        Ticket nt{seq_.load(std::memory_order_relaxed)};
        lk.unlock();
        return std::make_optional(
            std::make_pair(std::atomic_load_explicit(&ptr_, std::memory_order_acquire), nt));
    }

   private:
    template <class Ptr>
    void publish_impl(Ptr&& v) noexcept {
        std::atomic_store_explicit(&ptr_, std::forward<Ptr>(v), std::memory_order_release);
        {
            std::lock_guard<std::mutex> lk(m_);
            seq_.fetch_add(1, std::memory_order_release);
        }
        cv_.notify_all();
    }

    mutable std::mutex m_;
    std::condition_variable cv_;
    std::atomic<uint64_t> seq_;       // 关键：构造函数中初始化为 0
    std::shared_ptr<const T> ptr_{};  // 最新值（共享给读者）
};
