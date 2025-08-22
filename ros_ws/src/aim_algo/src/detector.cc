#include "detector.hpp"

Detector::Detector(std::shared_ptr<CameraPublisher> cam_node)
    : Node("detector_bus_thread", rclcpp::NodeOptions().use_intra_process_comms(true)),
      _cam_node(cam_node) {
    this->_center_x = _config.center_x;
    this->_center_y = _config.center_y;
    this->_cam_qos_keep_last = _config.camera_qos_keep_last;
    this->wl_ = WhiteLampParams();

    init_white_lamp_detector(this->wl_);

    welcom();
}

void Detector::detector_worker() {
    while (_running.load(std::memory_order_relaxed)) {
        // lock the mutex for the img msg first
        auto img = _cam_node->get_latest_iamge_msg();
        if (!img) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        auto cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        const cv::Mat& frame = cv_ptr->image;

        cv::Point2f center_px = cv::Point2f(0, 0);
        cv::Rect bbox;
        bool find = detect_white_lamp(frame, center_px, bbox, &_ui_frame);
        if (find) RCLCPP_INFO(this->get_logger(), "find");
    }
}

void Detector::ui_worker() {
    while (_running.load(std::memory_order_relaxed)) {
        if (_ui_frame.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        cv::Mat ui_frame = _ui_frame.clone();
        if (_config.SHOW_CV_MONITOR_WINDOWS) {
            cv::imshow("Detector UI", ui_frame);
            cv::waitKey(1);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

void Detector::kf_worker() {
    while (_running.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void Detector::commu_worker() {
    while (_running.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

bool Detector::start() {
    if (_running.load()) return true;

    _running.store(true);

    _th_worker = std::thread([this] { this->detector_worker(); });
    _th_kf = std::thread([this] { this->kf_worker(); });
    _th_commu = std::thread([this] { this->commu_worker(); });

    if (_config.SHOW_CV_MONITOR_WINDOWS) _th_ui = std::thread([this] { this->ui_worker(); });

    RCLCPP_INFO(this->get_logger(), "detector node thread start");
    return true;
}

void Detector::stop() {
    if (!_running.exchange(false)) return;

    if (_th_worker.joinable()) _th_worker.join();
    if (_th_kf.joinable()) _th_kf.join();
    RCLCPP_INFO(this->get_logger(), "detector node, kalman node thread closed");
}

void Detector::welcom() {
    std::cout << "welcom to detector" << std::endl;

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
