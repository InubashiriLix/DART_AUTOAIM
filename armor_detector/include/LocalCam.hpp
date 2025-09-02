// localcam/DHLocalCam.hpp
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
#include "camera/CamWrapper.h"
#include "camera/CamWrapperDH.h"

namespace localcam {

// ===== 通道与消息格式（与原 LocalCam 保持一致）=====
using Mat = cv::Mat;
// 注意：第二个字段是时间戳（ms, 单调时钟）。如需更安全可改成 int64_t 并同步改 LatestChannel 模板。
using Msg = std::pair<Mat, int>;
using MatChannel = LatestChannel<Msg>;

// ===== 运行配置（纯 C++，不依赖 ROS）=====
struct DHLocalCamConfig {
    // 硬件
    std::string SN = "FGV22100004";  // 大恒相机序列号（空串=第一个）
    bool FOR_PC = true;              // 与 CamNode 同名，false 时不启设备（便于在无机环境编译）

    // 采集
    int FPS = 120;                 // 期望相机帧率（传给 SDK）
    int frame_refresh_rate = 120;  // 轮询读取频率（Hz）
    bool IS_ROTATE = false;        // 采集后是否旋转 180°
    int avg_frame_delay_num = 60;  // 每 N 帧打印一次平均耗时（stdout）
    bool PRINT_STATS = true;       // 是否打印采集耗时

    // ROI / binning / 传感器尺寸（与 CamNode 同步）
    int nBinning = 1;
    int sensor_width = 1280;
    int sensor_height = 1024;
    int ROI_width = 1280;
    int ROI_height = 1024;

    // 相机内参默认值（与 CamNode::prepare_cam_info 的常量一致，可外部覆写）
    double K_fx = 1557.2, K_fy = 1557.5, K_cx = 638.7311, K_cy = 515.1176, K_s = 0.2065;
    cv::Mat D = (cv::Mat_<double>(1, 5) << -0.1295, 0.0804, 4.85e-04, 6.37e-04, 0.2375);

    // 发布时是否深拷贝（默认 false：零拷贝 header；如担心底层缓冲被复用，可设为 true）
    bool DEEP_COPY_ON_PUBLISH = false;
};

class DHLocalCam {
   public:
    DHLocalCam(std::shared_ptr<MatChannel> output_channel, const DHLocalCamConfig& cfg)
        : cfg_(cfg), output_channel_(std::move(output_channel)) {
        compute_cam_info_();
    }

    ~DHLocalCam() { stop(); }

    // 启停（start 成功后后台线程开始采集并不断 publish 到 channel）
    bool start() {
        if (running_.exchange(true, std::memory_order_acq_rel)) return true;

        if (!cfg_.FOR_PC) {
            // 与 CamNode 行为一致：不启设备
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

    void stop() {
        if (!running_.exchange(false, std::memory_order_acq_rel)) return;
        if (th_worker_.joinable()) th_worker_.join();
        close_camera_();
    }

    bool isRunning() const { return running_.load(std::memory_order_relaxed); }

    // 读取相机 SDK 给的传输/缓存延迟（若 SDK 支持）
    double get_cam_trans_delay_ms() const {
        if (!camera_) return 0.0;
        return 0;
    }

    // 内参访问（深拷贝返回）
    cv::Mat getK() const { return K_.clone(); }
    cv::Mat getD() const { return D_.clone(); }
    cv::Mat getP3x3() const { return P3x3_.clone(); }

   private:
    using clock_t = std::chrono::steady_clock;

    // ===== 内部工具 =====
    static inline int mono_ms_() {
        using namespace std::chrono;
        return static_cast<int>(
            duration_cast<milliseconds>(clock_t::now().time_since_epoch()).count());
    }

    void compute_cam_info_() {
        // 与 CamNode::prepare_cam_info 的推导保持一致
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
        std::cout << "opening cam with SN" << cfg_.SN << std::endl;
        camera_ = new DHCamera("FGV22100004");
        // ROI 居中
        int offx = (cfg_.sensor_width / std::max(1, cfg_.nBinning) - cfg_.ROI_width) / 2;
        int offy = (cfg_.sensor_height / std::max(1, cfg_.nBinning) - cfg_.ROI_height) / 2;

        // init(offsetX, offsetY, ROI_W, ROI_H, exposure(us), gain, isHWTrigger, fps, binning)
        bool ok = camera_->init(offx, offy, cfg_.ROI_width, cfg_.ROI_height,
                                /*exposure_us=*/1500, /*gain=*/16,
                                /*hw_trigger=*/false, cfg_.FPS, cfg_.nBinning);
        if (!ok) {
            // delete camera_;
            camera_ = nullptr;
            return false;
        }
        if (!camera_->start()) {
            // delete camera_;
            camera_ = nullptr;
            return false;
        }
        return true;
    }

    void close_camera_() {
        if (camera_) {
            camera_->stop();
            // delete camera_;
            camera_ = nullptr;
        }
    }

    void worker_loop_() {
        // 与 CamNode 同样的固定周期轮询
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

            int ts = mono_ms_();

            if (cfg_.DEEP_COPY_ON_PUBLISH) {
                // 安全但更慢：深拷贝一份
                auto msg = std::make_shared<const Msg>(frame.clone(), ts);
                output_channel_->publish(msg);
            } else {
                // 快速：零拷贝 header（注意底层缓冲复用风险，按需切换到深拷贝）
                auto msg = std::make_shared<const Msg>(std::move(frame), ts);
                output_channel_->publish(msg);
            }

            // 统计耗时
            auto t1 = clock_t::now();
            double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
            if (cfg_.PRINT_STATS) {
                cost_ms.push_back(ms);
                if ((int)cost_ms.size() >= cfg_.avg_frame_delay_num) {
                    double avg =
                        std::accumulate(cost_ms.begin(), cost_ms.end(), 0.0) / cost_ms.size();
                    std::cout << "[DHLocalCam] avg " << cfg_.avg_frame_delay_num
                              << " capture cost: " << avg << " ms\n";
                    cost_ms.clear();
                }
            }

            // 周期对齐
            next_tick += period;
            auto now = clock_t::now();
            if (now < next_tick) {
                std::this_thread::sleep_until(next_tick);
            } else if (now - next_tick > period) {
                next_tick = now;
            }
        }
    }

   private:
    DHLocalCamConfig cfg_;
    std::shared_ptr<MatChannel> output_channel_{};

    Camera* camera_ = nullptr;

    std::atomic<bool> running_{false};
    std::thread th_worker_;

    cv::Mat K_, D_, P3x3_;
};

}  // namespace localcam
