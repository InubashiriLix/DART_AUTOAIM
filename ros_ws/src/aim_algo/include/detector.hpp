#pragma once

#include <cv_bridge/cv_bridge.h>

#include <atomic>
#include <boost/lockfree/spsc_queue.hpp>
#include <condition_variable>
#include <geometry_msgs/msg/twist.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <thread>

#include "camera/CamNode.hpp"
#include "config_parser.hpp"

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

struct WhiteLampParams {
    int Y_min = 210;            // Y 亮度下限
    int Cr_tol = 15;            // |Cr-128| < tol
    int Cb_tol = 15;            // |Cb-128| < tol
    int min_area = 80;          // 最小连通域面积
    float min_solidity = 0.6f;  // 面积 / 外接矩形面积
    cv::Size k_open = {3, 3};   // 开操作核
    cv::Size k_close = {5, 5};  // 闭操作核
};

struct KalmanMsg {
    time_t img_target_time_stamp;     // 时间戳
    time_t lower_machine_time_stamp;  // 下位机时间戳

    // the target in the image
    float x = 0;
    float y = 0;

    // the joint state and q[4], pitch, yaw angle
    float joint_state_x = 0;
    float joint_state_y = 0;
    float pitch_angle = 0;
    float yaw_angle = 0;
    float q[4] = {0, 0, 0, 0};
};

class Detector : public rclcpp::Node {
   public:
    Detector(std::shared_ptr<CameraPublisher> cam_node);

    bool start();
    void stop();

   private:
    std::shared_ptr<CameraPublisher> _cam_node;

    void welcom();

    void detector_worker();  // 视觉工作线程
    void kf_worker();        // KF/控制线程（预留）
    void commu_worker();
    void ui_worker();  // 可选的 UI 线程（用于调试）
    cv::Mat _ui_frame;

    // aim cv functions and configs
    std::vector<double> time_stamps;
    void init_white_lamp_detector(const WhiteLampParams& p);
    bool detect_white_lamp(const cv::Mat& bgr, cv::Point2f& center_px, cv::Rect& bbox,
                           cv::Mat* debug /*可选，传入则画可视化*/);
    // 作为 Detector 成员，循环外预分配（避免反复分配）
    cv::Mat ycrcb_, Y_, Cr_, Cb_, tmpCr_, tmpCb_, mY_, mCr_, mCb_, mask_;
    cv::Mat kOpen_, kClose_;
    WhiteLampParams wl_;

    std::thread _th_worker;
    std::thread _th_kf;
    std::thread _th_commu;
    std::thread _th_ui;
    std::atomic<bool> _running{false};

    std::queue<KalmanMsg> _kalman_msg_queue;
    std::mutex _kalman_mgs_mutex;
    Semaphore _sem_kalman_msg;  // Initial count 0

    // 你的配置
    detector_config _config;
    double _center_x{}, _center_y{};
    int _cam_qos_keep_last{};
};
