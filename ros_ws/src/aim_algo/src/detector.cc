#include "detector.hpp"

#include "camera/CamNode.hpp"

Detector::Detector(const rclcpp::NodeOptions& opts)
    : Node("algo_detector_node", rclcpp::NodeOptions(opts).use_intra_process_comms(true)) {
    this->_center_x = _config.center_x;
    this->_center_y = _config.center_y;
    this->_cam_qos_keep_last = _config.camera_qos_keep_last;

    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(4).best_effort().durability_volatile();

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

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<Detector>();
//     rclcpp::executors::MultiThreadedExecutor exec;
//     exec.add_node(node);
//     exec.spin();
//     rclcpp::shutdown();
// }
