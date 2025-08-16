#include <detect.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>

using namespace std::chrono_literals;

void LowCamNode::pub_timer_callback() {
    auto low_cam_target_msg = geometry_msgs::msg::Point();
    low_cam_target_msg.x = _green_spot_detector_->get_center().dx;
    low_cam_target_msg.y = _green_spot_detector_->get_center().dy;
    _low_cam_target_pub_->publish(low_cam_target_msg);
    RCLCPP_INFO(this->get_logger(), "Published low cam target: %f, %f, %f", low_cam_target_msg.x,
                low_cam_target_msg.y, low_cam_target_msg.z);
}

LowCamNode::LowCamNode(const std::string& name) : Node(name) {
    // create the detect instance and open the camera
    _green_spot_detector_ = std::make_shared<lower_cam_detector::GreenSpotDetector>(0);
    int status_vision_start = _green_spot_detector_->start();
    // if open cam failed, abort
    if (status_vision_start != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start vision thread");
        _green_spot_detector_->stop();
        throw std::runtime_error("Camera initialization failed");
    }

    // NOTE: if the cam is not open, then the thread will not be started and publisher will not run
    _low_cam_target_pub_ = this->create_publisher<geometry_msgs::msg::Point>(this->TOPIC_NAME, 10);
    __pub_timer__ = this->create_wall_timer(10ms, std::bind(&LowCamNode::pub_timer_callback, this));

    // start the detect loop
    vision_thread =
        std::thread(&lower_cam_detector::GreenSpotDetector::run, _green_spot_detector_.get());
}

LowCamNode::~LowCamNode() {
    RCLCPP_INFO(this->get_logger(), "Shutting down node, stopping worker thread");
    _green_spot_detector_->stop();

    if (vision_thread.joinable()) {
        vision_thread.join();
    }
    RCLCPP_INFO(this->get_logger(), "Worker thread stopped");
}
