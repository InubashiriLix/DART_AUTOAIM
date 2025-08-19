#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <thread>

namespace cv {
class Mat;
}

class CameraPublisher : public rclcpp::Node {
   public:
    CameraPublisher(int /*argc*/, char ** /*argv*/);

    bool start();
    void stop();
    bool is_running() const;

    sensor_msgs::msg::Image get_image_buf();

    ~CameraPublisher() override;

    std::mutex _latest_mtx;

   private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_info_pub_;
    rclcpp::TimerBase::SharedPtr _info_timer_;

    std::thread _th_worker;
    std::thread _th_ui;
    std::atomic<bool> _running{false};

    // UI 最新帧
    std::shared_ptr<cv::Mat> _latest_frame_for_ui;

    sensor_msgs::msg::Image _img_msg_buf;

    void welcom();
    void worker_loop();
    void ui_loop();
    void publish_camera_info();
    void _prepare_image_msg();
    void fill_image_msg(sensor_msgs::msg::Image &msg, const rclcpp::Time &stamp,
                        const cv::Mat &bgr);
};
