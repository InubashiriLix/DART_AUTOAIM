#ifndef CAPTURE_PUB_H
#define CAPTURE_PUB_H

#include <mutex>
#include <condition_variable>
#include <vector>
#include <queue>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
class CapturePub {
public:
    CapturePub(rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub1);

private:
    int img_width;
    int img_height;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub1_;
};

#endif // INFER_REQUEST_POOL_H
