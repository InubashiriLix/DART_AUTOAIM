#pragma once

#include <cv_bridge/cv_bridge.h>

#include <atomic>
#include <boost/lockfree/spsc_queue.hpp>
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

#include "camera/CamGeo.hpp"
#include "camera/CamNode.hpp"
#include "config_parser.hpp"
#include "contact/Contact.h"
#include "contact/Protocol.h"
#include "kalman/kalman.hpp"
#include "utils/Semaphore.hpp"
#include "utils/logging.hpp"

struct WhiteLampParams {
    int Y_min = 210;            // Y 亮度下限
    int Cr_tol = 15;            // |Cr-128| < tol
    int Cb_tol = 15;            // |Cb-128| < tol
    int min_area = 80;          // 最小连通域面积
    float min_solidity = 0.6f;  // 面积 / 外接矩形面积
    cv::Size k_open = {3, 3};   // 开操作核
    cv::Size k_close = {5, 5};  // 闭操作核
};

class Detector : public rclcpp::Node {
   public:
    Detector(std::shared_ptr<CameraPublisher> cam_node);

    bool start();
    void stop();

   private:
    // ========================= common settings and tools =============================
    // the running flag
    std::atomic<bool> _running{false};
    detector_config _config;
    double _center_x{}, _center_y{};
    std::shared_ptr<CameraPublisher> _cam_node;
    void welcom();
    // camera info logger
    std::shared_ptr<spdlog::logger> _cam_log;

    // ======================== the detector worker and related ========================
    // aim cv functions and configs
    std::vector<double> time_stamps;
    void init_white_lamp_detector(const WhiteLampParams& p);
    bool detect_white_lamp(const cv::Mat& bgr, cv::Point2f& center_px, cv::Rect& bbox,
                           cv::Mat* debug /*可选，传入则画可视化*/);
    // 作为 Detector 成员，循环外预分配（避免反复分配）
    cv::Mat ycrcb_, Y_, Cr_, Cb_, tmpCr_, tmpCb_, mY_, mCr_, mCb_, mask_, kOpen_, kClose_;
    WhiteLampParams wl_;
    // the point2f to angle class
    geom::CameraGeometry _cam_geo;
    void prepare_cam_geometry();
    // threads
    std::thread _th_worker;
    void detector_worker();
    // logger
    std::shared_ptr<spdlog::logger> _detector_log;

    // =================== kalman filter with delay aware ==============================
    // logger
    std::shared_ptr<spdlog::logger> _kalman_log;
    // the kalman class
    Kalman _kf;
    // the msg queue for kalman
    std::mutex _kf_msg_mtx;
    KalmanMsg _kf_msg;                   // 最新融合消息（由两个线程共同填充）
    std::atomic<uint64_t> _meas_seq{0};  // 自瞄量测序号（ detector 写，kf 读 ）
    uint64_t _meas_seq_seen{0};          // kf 线程已消费到的序号
    std::atomic<bool> _target_lost{true};

    std::atomic<float> _lat_est_ms{25.f};  // dynamic vision latency estimate (ms)

    std::atomic<int64_t> _preview_ms{40};  // 动态预瞄（外部可随时改）

    // the thread for kalman filter
    std::thread _th_kf;
    void kf_worker();

    // ========================= commu lower machine thread ============================
    // logger
    std::shared_ptr<spdlog::logger> _contact_log;
    // the contact object
    contact_config contact_conf;
    Contact _contact_;
    // tools
    bool prepare_contact();
    // threads
    std::thread _th_contact;
    void contact_worker();

    // ================================ ui display thread ==============================
    cv::Mat _ui_frame;
    std::thread _th_ui;
    void ui_worker();
};
