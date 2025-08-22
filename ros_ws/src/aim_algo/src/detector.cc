#include "detector.hpp"

Detector::Detector(CameraPublisher&& cam_node)
    : Node("detector_bus_thread", rclcpp::NodeOptions().use_intra_process_comms(true)) {
    this->_center_x = _config.center_x;
    this->_center_y = _config.center_y;
    this->_cam_qos_keep_last = _config.camera_qos_keep_last;

    _th_worker = std::thread([this] { detector_worker(); });
    _th_kf = std::thread([this] { kf_worker(); });

    welcom();
}

void Detector::detector_worker() {
    while (_running.load(std::memory_order_relaxed)) {
        //
    }
}

void Detector::kf_worker() {
    while (_running.load(std::memory_order_relaxed)) {
        //
    }
}

void Detector::commu_worker() {
    while (_running.load(std::memory_order_relaxed)) {
        // 这里可以添加通信逻辑
    }
}

bool Detector::start() {
    if (_running.load()) return true;

    _th_worker = std::thread([this] { this->detector_worker(); });
    _th_kf = std::thread([this] { this->kf_worker(); });
    _th_commu = std::thread([this] { this->commu_worker(); });

    _running.store(true);
    return true;
}

void Detector::stop() {
    if (!_running.exchange(false)) return;

    if (_th_worker.joinable()) _th_worker.join();
    if (_th_kf.joinable()) _th_kf.join();
    RCLCPP_INFO(this->get_logger(), "detector node, kalman node thread closed");
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
