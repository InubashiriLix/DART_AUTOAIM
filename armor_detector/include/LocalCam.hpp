// LLocalCam.hpp
#pragma once
#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "LatestChannel.hpp"
#include "camera/CamWrapperDH.h"  // 直接用大恒相机封装；若要多机型可自行扩展

namespace localcam {

using Mat = cv::Mat;
// 注意：为兼容你现有代码，这里时间戳仍用 int（ms）。更稳妥可改成 int64_t。
using Msg = std::pair<Mat, int>;
using MatChannel = LatestChannel<Msg>;

struct LLocalCamConfig {
    // 硬件
    std::string SN = "FGV22100004";  // 空串=第一个
    bool FOR_PC = true;

    // 采集
    int FPS = 120;
    int frame_refresh_rate = 120;  // 轮询频率（Hz）
    bool IS_ROTATE = false;
    int avg_frame_delay_num = 60;  // 每 N 帧统计一次耗时
    bool PRINT_STATS = true;

    // ROI / 传感器
    int nBinning = 1;
    int sensor_width = 1280;
    int sensor_height = 1024;
    int ROI_width = 1280;
    int ROI_height = 1024;

    // 内参（示例默认，与 ROS 版保持一致，外部可覆盖）
    double K_fx = 1557.2, K_fy = 1557.5, K_cx = 638.7311, K_cy = 515.1176, K_s = 0.2065;
    cv::Mat D = (cv::Mat_<double>(1, 5) << -0.1295, 0.0804, 4.85e-04, 6.37e-04, 0.2375);

    // 发布时是否深拷贝（强烈建议 true，避免 SDK 复用缓冲导致 UAF）
    bool DEEP_COPY_ON_PUBLISH = true;

    // 额外相机参数（按需扩展）
    int exposure_us = 1500;
    int gain = 16;
    bool hw_trigger = false;
};

class LLocalCam {
   public:
    LLocalCam(std::shared_ptr<MatChannel> output_channel, const LLocalCamConfig& cfg)
        : cfg_(cfg), output_channel_(std::move(output_channel)) {
        compute_cam_info_();
    }

    ~LLocalCam() { stop(); }

    // 启动采集线程
    bool start() {
        if (running_.exchange(true, std::memory_order_acq_rel)) return true;

        if (!cfg_.FOR_PC) {
            running_.store(false, std::memory_order_release);
            return false;
        }
        if (!open_camera_()) {
            running_.store(false, std::memory_order_release);
            return false;
        }
        th_worker_ = std::thread([this] { worker_loop_(); });
        return true;
    }

    // 停止采集线程
    void stop() {
        if (!running_.exchange(false, std::memory_order_acq_rel)) return;
        if (th_worker_.joinable()) th_worker_.join();
        close_camera_();
    }

    bool isRunning() const { return running_.load(std::memory_order_relaxed); }

    // 可选：读取 SDK 传输/缓存延迟（若 SDK 支持的话）
    double get_cam_trans_delay_ms() const {
        if (!camera_) return 0.0;
        return 0.0;
    }

    // 内参访问
    cv::Mat getK() const { return K_.clone(); }
    cv::Mat getD() const { return D_.clone(); }
    cv::Mat getP3x3() const { return P3x3_.clone(); }

   private:
    using clock_t = std::chrono::steady_clock;

    static inline int mono_ms_() {
        using namespace std::chrono;
        return static_cast<int>(
            duration_cast<milliseconds>(clock_t::now().time_since_epoch()).count());
    }

    void compute_cam_info_() {
        int nB = std::max(1, cfg_.nBinning);
        const int sensor_w = cfg_.sensor_width;
        const int sensor_h = cfg_.sensor_height;
        const int ROI_w = cfg_.ROI_width;
        const int ROI_h = cfg_.ROI_height;

        double fx = cfg_.K_fx / nB;
        double fy = cfg_.K_fy / nB;
        double s = cfg_.K_s;

        double cx_scaled = (cfg_.K_cx / nB) / ((double(sensor_w) / nB) / ROI_w);
        double cy_scaled = (cfg_.K_cy / nB) / ((double(sensor_h) / nB) / ROI_h);

        K_ = (cv::Mat_<double>(3, 3) << fx, s, cx_scaled, 0.0, fy, cy_scaled, 0.0, 0.0, 1.0);
        D_ = cfg_.D.clone();
        P3x3_ = cv::getOptimalNewCameraMatrix(K_, D_, cv::Size(ROI_w, ROI_h), 0);
    }

    bool open_camera_() {
        std::cout << "[LLocalCam] opening DHCamera SN=" << (cfg_.SN.empty() ? "<first>" : cfg_.SN)
                  << std::endl;
        camera_ = std::make_unique<DHCamera>(cfg_.SN);

        int offx = (cfg_.sensor_width / std::max(1, cfg_.nBinning) - cfg_.ROI_width) / 2;
        int offy = (cfg_.sensor_height / std::max(1, cfg_.nBinning) - cfg_.ROI_height) / 2;

        bool ok = camera_->init(offx, offy, cfg_.ROI_width, cfg_.ROI_height, cfg_.exposure_us,
                                cfg_.gain, cfg_.hw_trigger, cfg_.FPS, cfg_.nBinning);
        if (!ok) {
            camera_.reset();
            return false;
        }
        if (!camera_->start()) {
            camera_.reset();
            return false;
        }
        return true;
    }

    void close_camera_() {
        if (camera_) {
            camera_->stop();
            camera_.reset();
        }
    }

    void worker_loop_() {
        const int hz = std::max(1, cfg_.frame_refresh_rate);
        const auto period = std::chrono::milliseconds(1000 / hz);
        auto next_tick = clock_t::now();

        cv::Mat frame;
        std::vector<double> cost_ms;
        cost_ms.reserve(std::max(1, cfg_.avg_frame_delay_num));

        while (running_.load(std::memory_order_relaxed)) {
            auto t0 = clock_t::now();

            bool ok = (camera_ && camera_->read(frame));
            if (!ok || frame.empty()) {
                next_tick = t0 + period;
                std::this_thread::sleep_until(next_tick);
                continue;
            }

            if (cfg_.IS_ROTATE) cv::rotate(frame, frame, cv::ROTATE_180);

            // 强烈建议深拷贝后再发布，避免 SDK 复用缓冲导致悬挂
            cv::Mat msg_frame = cfg_.DEEP_COPY_ON_PUBLISH ? frame.clone() : frame;

            int ts = mono_ms_();
            if (output_channel_) {
                // 注意：不要 std::move(frame)，否则下次 read(frame) 可能踩坑
                auto msg = std::make_shared<const Msg>(std::move(msg_frame), ts);
                output_channel_->publish(msg);
            }

            auto t1 = clock_t::now();
            double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
            if (cfg_.PRINT_STATS) {
                cost_ms.push_back(ms);
                if ((int)cost_ms.size() >= cfg_.avg_frame_delay_num) {
                    double avg =
                        std::accumulate(cost_ms.begin(), cost_ms.end(), 0.0) / cost_ms.size();
                    std::cout << "[LLocalCam] avg " << cfg_.avg_frame_delay_num
                              << " capture cost: " << avg << " ms\n";
                    cost_ms.clear();
                }
            }

            next_tick += period;
            auto now = clock_t::now();
            if (now < next_tick)
                std::this_thread::sleep_until(next_tick);
            else if (now - next_tick > period)
                next_tick = now;
        }
    }

   private:
    LLocalCamConfig cfg_;
    std::shared_ptr<MatChannel> output_channel_{};

    std::unique_ptr<DHCamera> camera_;  // 安全的生命周期管理

    std::atomic<bool> running_{false};
    std::thread th_worker_;

    cv::Mat K_, D_, P3x3_;
};

}  // namespace localcam
