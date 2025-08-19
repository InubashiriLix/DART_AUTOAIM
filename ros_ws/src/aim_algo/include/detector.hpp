#include <cv_bridge/cv_bridge.h>

#include <atomic>
#include <boost/lockfree/spsc_queue.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <thread>

#include "config_parser.hpp"

class Detector : public rclcpp::Node {
   public:
    explicit Detector(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());

   private:
    void welcom();

    void img_worker();  // 视觉工作线程
    void kf_worker();   // KF/控制线程（预留）

    std::thread _th_img, _th_kf;
    std::atomic<bool> _running{true};

    // 相机参数缓存（如果要PnP）
    sensor_msgs::msg::CameraInfo _cam_info;
    std::atomic<bool> _has_cam_info{false};

    // 你的配置
    detector_config _config;
    double _center_x{}, _center_y{};
    int _cam_qos_keep_last{};
};
