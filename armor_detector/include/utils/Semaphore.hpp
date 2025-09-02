#pragma once
#include <condition_variable>
#include <mutex>

class Semaphore {
   public:
    explicit Semaphore(int count = 0) : count_(count) {}

    void release() {
        std::unique_lock<std::mutex> lock(mtx_);
        ++count_;
        cv_.notify_one();
    }

    void acquire() {
        std::unique_lock<std::mutex> lock(mtx_);
        cv_.wait(lock, [this]() { return count_ > 0; });
        --count_;
    }

   private:
    std::mutex mtx_;
    std::condition_variable cv_;
    int count_;
};
