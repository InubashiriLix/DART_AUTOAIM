#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>

#include "LatestChannel.hpp"

namespace detector {

using Mat = cv::Mat;
using Msg = std::pair<Mat, int>;  // the mat and timestamp
using DtOutput = std::pair<std::pair<int, int>, int>;
using MatChannel = LatestChannel<Msg>;

class ArmorDetection {
   public:
    ArmorDetection(std::shared_ptr<MatChannel> input_channel) : input_channel_(input_channel) {
        std::cout << "ArmorDetection Constructor Start" << std::endl;
    }
    ~ArmorDetection() { stop(); }

    bool start() {
        worker_ = std::thread(&ArmorDetection::run, this);

        running_.store(true, std::memory_order_acquire);

        return true;
    }

    bool stop() {
        running_.store(false, std::memory_order_acquire);
        return true;
    }

    void run() {
        auto tk = input_channel_->ticket();
        while (running_.load(std::memory_order_relaxed)) {
            auto [msg, ntk] = input_channel_->wait_next(tk);
            tk = ntk;

            // show the image
            cv::imshow("ArmorDetection", msg->first);
        }
    }

   private:
    std::atomic<bool> running_{false};
    std::thread worker_;

    std::shared_ptr<MatChannel> input_channel_;
};

}  // namespace detector
