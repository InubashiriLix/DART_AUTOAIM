#include "detector.hpp"

Detector::Detector(const rclcpp::NodeOptions& opts)
    : Node("algo_detector_node", rclcpp::NodeOptions(opts).use_intra_process_comms(true)) {
    this->_center_x = _config.center_x;
    this->_center_y = _config.center_y;
    this->_cam_qos_keep_last = _config.camera_qos_keep_last;

    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(4).best_effort().durability_volatile();

    rclcpp::SubscriptionOptions img_opt, info_opt;
    auto cg_img = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto cg_info = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    img_opt.callback_group = cg_img;
    info_opt.callback_group = cg_info;

    _cam_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", sensor_qos, std::bind(&Detector::image_callback, this, std::placeholders::_1),
        img_opt);

    _cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", sensor_qos,
        std::bind(&Detector::image_info_callback, this, std::placeholders::_1), info_opt);

    _cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/gimbal/cmd", 10);

    // 工作线程
    // _th_img = std::thread([this] { img_worker(); });
    // _th_kf = std::thread([this] { kf_worker(); });

    welcom();
}

// detector.cc 末尾加上 —— 与头文件原型保持一致（无 const、无参数）
void Detector::img_worker() {
    cv_bridge::CvImageConstPtr frame;
    while (_running.load(std::memory_order_relaxed)) {
        if (_img_q.pop(frame)) {
            const cv::Mat img = frame->image;  // 你后续的视觉处理...
            if (_config.SHOW_CV_MONITOR_WINDOWS) {
                cv::imshow("Detector Image", img);
                cv::waitKey(1);
            }
            // TODO: 推测量给 KF 队列
        } else {
            std::this_thread::yield();
        }
    }
}

void Detector::kf_worker() {
    while (_running.load(std::memory_order_relaxed)) {
        // TODO: 从测量队列取数据，KF predict/update，最后发布控制
        std::this_thread::yield();
    }
}

void Detector::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Process the image message
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Point center(img.cols / 2, img.rows / 2);
    cv::circle(img, center, 5, cv::Scalar(0, 0, 255), -1);
    if (_config.SHOW_CV_MONITOR_WINDOWS) {
        cv::imshow("Detector Image", img);
        cv::waitKey(1);
    }

    double delay_ms =
        ((msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9) - now().seconds()) * 1000.0;
    RCLCPP_INFO(this->get_logger(), "delay: %.2f ms", delay_ms);
}

void Detector::image_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr info) {
    // RCLCPP_INFO(this->get_logger(), "binning_x: %d, binning_y %d", info->binning_x,
    //             info->binning_y);
    RCLCPP_DEBUG(this->get_logger(), "width: %d, height %d", info->width, info->height);
    RCLCPP_DEBUG(this->get_logger(), "x_center: %d, y_center: %d", info->width / 2,
                 info->height / 2);
}

void Detector::welcom() {
    std::cout << "detector start" << std::endl;

    std::cout << "░▀█▀░█▀█░█░█░░░█▀▄░█▀▀░▀█▀░█▀▀░█▀▀░▀█▀░█▀█░█▀▄\n"
                 "░░█░░█░█░█░█░░░█░█░█▀▀░░█░░█▀▀░█░░░░█░░█░█░█▀▄\n"
                 "░▀▀▀░▀░▀░▀▀▀░░░▀▀░░▀▀▀░░▀░░▀▀▀░▀▀▀░░▀░░▀▀▀░▀░▀\n"
              << std::endl;
    std::cout << "============= configs ==============" << std ::endl;
    std::cout << "SHOW CV MONITOR: " << _config.SHOW_CV_MONITOR_WINDOWS << std::endl;
    std::cout << "CENTER_X: " << _config.center_x << std::endl;
    std::cout << "CENTER_Y: " << _config.center_y << std::endl;
    std::cout << "=========== configs end ============" << std::endl;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Detector>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
}
