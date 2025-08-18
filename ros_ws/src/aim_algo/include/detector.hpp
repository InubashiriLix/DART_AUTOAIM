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

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void image_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr info);

    void img_worker();  // 视觉工作线程
    void kf_worker();   // KF/控制线程（预留）

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _cam_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _cam_info_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_pub_;

    // SPSC 队列：跨线程传图（小容量，满了就丢旧帧）
    boost::lockfree::spsc_queue<cv_bridge::CvImageConstPtr> _img_q{4};

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
