// #define NODE_HPP
// #ifdef NODE_HPP
#pragma once

#include <detect.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#define NODE_NAME "low_cam_node"

class LowCamNode : public rclcpp::Node {
   private:
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr _low_cam_target_pub_;
    rclcpp::TimerBase::SharedPtr __pub_timer__;
    const std::string TOPIC_NAME = "/low_cam_target";

    void pub_timer_callback();

    std::shared_ptr<lower_cam_detector::GreenSpotDetector> _green_spot_detector_;
    std::thread vision_thread;
    // std::atomic<bool> running_vision_thread{true};

   public:
    explicit LowCamNode(const std::string& name);
    ~LowCamNode() override;
};

// #endif  // NODE_HPP
