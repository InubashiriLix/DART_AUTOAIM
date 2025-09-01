#pragma once

#include <atomic>
#include <memory>

template <typename T>
class LatestChannel {
   public:
    void publish(const std::shared_ptr<const T>& v) noexcept {
        std::atomic_store_explicit(&ptr_, v, std::memory_order_release);
    }
    void publish(std::shared_ptr<const T>&& v) noexcept {
        std::atomic_store_explicit(&ptr_, std::move(v), std::memory_order_release);
    }

    std::shared_ptr<const T> get() const noexcept {
        return std::atomic_load_explicit(&ptr_, std::memory_order_acquire);
    }

   private:
    std::shared_ptr<const T> ptr_;
};
