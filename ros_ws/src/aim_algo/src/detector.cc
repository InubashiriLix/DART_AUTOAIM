#include "detector.hpp"

#include "contact/Protocol.h"

// #include <boost/iterator/iterator_categories.hpp>

Detector::Detector(std::shared_ptr<CameraPublisher> cam_node)
    : Node("detector_bus_thread", rclcpp::NodeOptions().use_intra_process_comms(true)),
      _cam_node(cam_node),
      contact_conf(contact_config()),
      _contact_(contact_conf) {
    // the loggers
    this->_detector_log = perflog::get("detector");
    this->_cam_log = perflog::get("cam");
    this->_contact_log = perflog::get("contact");
    this->_kalman_log = perflog::get("kalman");
    // NOTE: maybe I should remove them
    this->_center_x = _config.center_x;
    this->_center_y = _config.center_y;
    this->_cam_qos_keep_last = _config.camera_qos_keep_last;

    this->wl_ = WhiteLampParams();

    init_white_lamp_detector(this->wl_);
    prepare_cam_geometry();

    welcom();
}

void Detector::prepare_cam_geometry() {
    _cam_geo.rectified = false;
    _cam_geo.K = _cam_node->get_cam_info_k().clone();
    _cam_geo.D = _cam_node->get_cam_info_d().clone();
    _cam_geo.image_width = _cam_node->config.ROI_width;
    _cam_geo.image_height = _cam_node->config.ROI_height;

    auto deg2rad = [](double d) { return d * CV_PI / 180.0; };
    // TODO: you may want to set the offset in the toml config
    double alpha_deg = 0.0;  // pitch 偏置
    double beta_deg = 0.0;   // yaw 偏置
    cv::Mat Rx, Rz;
    {
        cv::Mat rvecX = (cv::Mat_<double>(3, 1) << deg2rad(alpha_deg), 0, 0);
        cv::Mat rvecZ = (cv::Mat_<double>(3, 1) << 0, 0, deg2rad(beta_deg));
        cv::Rodrigues(rvecX, Rx);
        cv::Rodrigues(rvecZ, Rz);
    }
    _cam_geo.R_cam2gimbal = Rz * Rx;

    _cam_geo.print(this->_cam_log);
}

void Detector::detector_worker() {
    while (_running.load(std::memory_order_relaxed)) {
        auto start_tp = this->now();
        // lock the mutex for the img msg first
        auto img = _cam_node->get_latest_iamge_msg();
        if (!img) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        auto cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        const cv::Mat& frame = cv_ptr->image;

        cv::Point2f center_px = cv::Point2f(0, 0);
        double yaw_deg = 0, pitch_deg = 0;
        cv::Rect bbox;
        bool find = detect_white_lamp(frame, center_px, bbox, &_ui_frame);
        if (find) {
            if (_cam_geo.pixelToYawPitchDeg(center_px, yaw_deg, pitch_deg)) {
                if (_config.SHOW_TARGET_ANGLE)
                    _detector_log->info("Pixel ({:.1f},{:.1f}) => yaw={:.1f} deg, pitch={:.1f} deg",
                                        center_px.x, center_px.y, yaw_deg, pitch_deg);
            } else {
                if (_config.SHOW_TARGET_ANGLE) _detector_log->warn("pixel to yaw pitch failed");
            }
        }

        // delay
        if (_config.SHOW_CV_CAL_DELAY) {
            auto end = this->now();
            auto ms_used = (end - start_tp).nanoseconds() / 1e6;
            time_stamps.push_back(ms_used);
            if (time_stamps.size() >= _config.avg_frame_delay_num) {
                double sum = std::accumulate(time_stamps.begin(), time_stamps.end(), 0.0);
                double delay_avg = sum / time_stamps.size();
                _detector_log->info("[frame detector] avg {} frame delay: {:.3f} ms",
                                    _config.avg_frame_delay_num, delay_avg);
                time_stamps.clear();
            }
        }
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

void Detector::contact_worker() {
    while (_running.load(std::memory_order_relaxed)) {
        ProjectileRx rx;
        if (_contact_.latest_rx(rx)) {
            _contact_log->info(
                "latest rx: pitch {:.2f} deg, yaw {:.2f} deg, quaternion [{:.2f}, {:.2f}, {:.2f}, "
                "{:.2f}]",
                rx.pitch, rx.yaw, rx.q[0], rx.q[1], rx.q[2], rx.q[3]);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

bool Detector::prepare_contact() {
    _contact_log->info("prepare contact...");
    _contact_log->info("=============== configs ================");
    _contact_log->info("serial_dev: {}", contact_conf.serial_dev);
    _contact_log->info("baud: {}", contact_conf.baud);
    _contact_log->info("rate_hz: {}", contact_conf.rate_hz);
    _contact_log->info("pitch_min_deg: {}", contact_conf.pitch_min_deg);
    _contact_log->info("pitch_max_deg: {}", contact_conf.pitch_max_deg);
    _contact_log->info("wrap_yaw_deg: {}", contact_conf.wrap_yaw_deg);
    _contact_log->info("header: 0x{:X}", contact_conf.header);
    _contact_log->info("require_rx_before_send: {}", contact_conf.require_rx_before_send);
    _contact_log->info("========================================");
    // start the contact inter thread here
    if (!_contact_.start(
            true)) {  // spawn_thread set as true here, so that we don't need to call run() to block
        RCLCPP_ERROR(this->get_logger(), "contact start failed");
        _detector_log->error("contact start failed");
        _contact_log->error("contact start failed");
        _running.store(false);
        return false;
    }

    _contact_log->info(
        "\n"
        "░█░░░█▀█░█▀▀░░░█▀▀░█▀█░█▀█░▀█▀░█▀█░█▀▀░▀█▀\n"
        "░█░░░█░█░▀▀█░░░█░░░█░█░█░█░░█░░█▀█░█░░░░█░\n"
        "░▀▀▀░▀▀▀░▀▀▀░░░▀▀▀░▀▀▀░▀░▀░░▀░░▀░▀░▀▀▀░░▀░\n");
    return true;
}

bool Detector::start() {
    if (_running.load()) return true;

    _running.store(true);

    if (!prepare_contact()) {
        _running.store(false);
        return false;
    }

    _th_worker = std::thread([this] { this->detector_worker(); });
    _kf = KalmanDelayAware("/home/orangepi/08_DART_AUTOAIM/ros_ws/config.toml", "kalman");
    _th_kf = std::thread([this] { this->kf_worker(); });
    _th_contact = std::thread([this] { this->contact_worker(); });

    if (_config.SHOW_CV_MONITOR_WINDOWS) _th_ui = std::thread([this] { this->ui_worker(); });

    RCLCPP_INFO(this->get_logger(), "detector node thread start");
    _detector_log->info("detector node thread start");

    return true;
}

void Detector::stop() {
    if (!_running.exchange(false)) return;

    if (_th_worker.joinable()) _th_worker.join();
    if (_th_kf.joinable()) _th_kf.join();
    if (_th_contact.joinable()) _th_contact.join();

    _contact_.stop();
    // RCLCPP_INFO(this->get_logger(), "detector node, kalman node thread closed");
    _detector_log->info("detector node, kalman node thread closed");
}

void Detector::welcom() {
    _detector_log->info("welcom to detector");
    _detector_log->info(
        "\n"
        "░▀█▀░█▀█░█░█░░░█▀▄░█▀▀░▀█▀░█▀▀░█▀▀░▀█▀░█▀█░█▀▄\n"
        "░░█░░█░█░█░█░░░█░█░█▀▀░░█░░█▀▀░█░░░░█░░█░█░█▀▄\n"
        "░▀▀▀░▀░▀░▀▀▀░░░▀▀░░▀▀▀░░▀░░▀▀▀░▀▀▀░░▀░░▀▀▀░▀░▀\n");
    _detector_log->info("============= configs ==============");
    _detector_log->info("SHOW_CV_MONITOR: {}", _config.SHOW_CV_MONITOR_WINDOWS);
    _detector_log->info("CENTER_X: {}", _config.center_x);
    _detector_log->info("CENTER_Y: {}", _config.center_y);
    _detector_log->info("camera_qos_keep_last: {}", _config.camera_qos_keep_last);
    _detector_log->info("SHOW_CV_CAL_DELAY: {}", _config.SHOW_CV_CAL_DELAY);
    _detector_log->info("avg_frame_delay_num: {}", _config.avg_frame_delay_num);
    _detector_log->info("=========== configs end ============");

    // std::cout << "welcom to detector" << std::endl;
    // std::cout << "░▀█▀░█▀█░█░█░░░█▀▄░█▀▀░▀█▀░█▀▀░█▀▀░▀█▀░█▀█░█▀▄\n"
    //              "░░█░░█░█░█░█░░░█░█░█▀▀░░█░░█▀▀░█░░░░█░░█░█░█▀▄\n"
    //              "░▀▀▀░▀░▀░▀▀▀░░░▀▀░░▀▀▀░░▀░░▀▀▀░▀▀▀░░▀░░▀▀▀░▀░▀\n"
    //           << std::endl;
    // std::cout << "============= configs ==============" << std ::endl;
    // std::cout << "SHOW CV MONITOR: " << _config.SHOW_CV_MONITOR_WINDOWS << std::endl;
    // std::cout << "CENTER_X: " << _config.center_x << std::endl;
    // std::cout << "CENTER_Y: " << _config.center_y << std::endl;
    // std::cout << "SHOW_CV_CAL_DELAY: " << _config.SHOW_CV_CAL_DELAY << std::endl;
    // std::cout << "avg_frame_delay_num: " << _config.avg_frame_delay_num << std::endl;
    // std::cout << "=========== configs end ============" << std::endl;
}
