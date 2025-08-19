#include "detector.hpp"

Detector::Detector(CameraPublisher&& cam_node)
    : Node("detector_bus_thread", rclcpp::NodeOptions().use_intra_process_comms(true)) {
    this->_center_x = _config.center_x;
    this->_center_y = _config.center_y;
    this->_cam_qos_keep_last = _config.camera_qos_keep_last;

    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(4).best_effort().durability_volatile();

    _th_worker = std::thread([this] { img_worker(); });
    _th_kf = std::thread([this] { kf_worker(); });

    welcom();
}

void Detector::img_worker() { while (_running); }

void Detector::kf_worker() {
    // while (_running.load(std::memory_order_relaxed)) {
    //     // TODO: 从测量队列取数据，KF predict/update，最后发布控制
    //     std::this_thread::yield();
    // }
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
