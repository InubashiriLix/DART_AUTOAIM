#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <thread>

#include "config_parser.hpp"
#include "utils/logging.hpp"
namespace cv {
class Mat;
}

class CameraPublisher : public rclcpp::Node {
   public:
    CameraPublisher(int /*argc*/, char** /*argv*/);

    bool start();
    void stop();
    bool is_running() const;

    /**
     * @brief get the actual delay in
     *
     * @return [TODO:return]
     */
    double get_cam_trans_delay();

    sensor_msgs::msg::Image::ConstSharedPtr get_latest_iamge_msg();
    sensor_msgs::msg::CameraInfo get_camera_info_buf();

    ~CameraPublisher() override;

    std::mutex _latest_mtx;

    const cv::Mat get_cam_info_k();
    const cv::Mat get_cam_info_d();
    const cv::Mat get_cam_info_p3x3();

    camera_config config;

   private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_info_pub_;
    rclcpp::TimerBase::SharedPtr _info_timer_;

    // the camera info should be shared
    void prepare_cam_info();
    cv::Mat K;
    cv::Mat D;
    cv::Mat P3x3;

    std::shared_ptr<spdlog::logger> _cam_log;
    std::thread _th_worker;
    std::thread _th_ui;
    std::atomic<bool> _running{false};

    // UI 最新帧
    std::shared_ptr<cv::Mat> _latest_frame_for_ui;
    std::shared_ptr<sensor_msgs::msg::Image> latest_img_;

    sensor_msgs::msg::Image _img_msg_buf;
    sensor_msgs::msg::CameraInfo _cam_info_buf;

    void welcom();
    void worker_loop();
    void ui_loop();
    void publish_camera_info();
    void _prepare_image_msg();
};
