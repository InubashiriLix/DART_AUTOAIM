#include "detector.hpp"

#include "contact/Protocol.h"

/*
 * Well, the whole process is
 * CamNode (frame + ~=5ms usb3.0 delay) -> Detector (update Kalman vision target msg)
 * + Contact (update kalman lowermachine msg) => Kalman filter
 * => Contact (send command to the lower machine + 10ms)
 * as the result, to get the software delay, we need this formula:
 * [[ ts_contact_send - ts_cam_img + 10ms (contact) + 5ms (usb3.0) ]] = software delay
 * it is recommanded that use dynamic software delay + bullet delay compensation to
 * get the real delay, that is, how many ms should the kalman filter predict forward
 * Luckily, our kalman filter is specially designed for this, so, enjoy it.
 * oh shit! cause' the camNode is routating to get the nesest frame, so you'd better notice there
 * may have some small noise in the system delay (maybe +-5ms? go compute it by yourself using the
 * fresh_rate in the toml settings)
 */

static inline int64_t to_ms(const rclcpp::Time& t) {
    return t.nanoseconds() / 1000000;  // ns -> ms
}

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

    this->_target_lost = true;

    this->_contact_.update_target(Target{0, 0, 0.0, 0});

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
                find = false;  // fall back to "no valid measurement"
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

        {
            // write the kf msg
            std::lock_guard<std::mutex> lk(_kf_msg_mtx);
            const auto now_ms = to_ms(this->now());

            if (find) {
                // valid measurement: absolute yaw/pitch at image time
                _kf_msg.img_target_time_stamp = to_ms(img->header.stamp);
                _kf_msg.x = static_cast<float>(yaw_deg);    // abs yaw (deg)
                _kf_msg.y = static_cast<float>(pitch_deg);  // abs pitch (deg)

                // bump measurement sequence so KF thread will fuse it this tick
                ++_meas_seq;

                // online vision-latency estimate (optional but useful when 10~100ms drift)
                float obs_latency_ms = float(now_ms - _kf_msg.img_target_time_stamp);
                obs_latency_ms = std::clamp(obs_latency_ms, 0.f, 200.f);
                float lp = 0.2f;  // EMA factor; tune if needed
                float lat_est = (1.f - lp) * _lat_est_ms.load() + lp * obs_latency_ms;
                _lat_est_ms.store(lat_est);
                _kf.setVisionLatencyMs(lat_est);  // dynamic latency! (runtime adjustable)

                _target_lost = false;
                // TODO: customize the lost time
            } else if (now_ms - _kf_msg.img_target_time_stamp > _config.target_lost_threshold_ms) {
                // if the target has been lost in this frame
                _target_lost = true;
            } else {
                // do nothing, don't update the invalid kalman msg cause' target degree is 0,
                // it will corrupt the kalman filter
            }
            // update done
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void Detector::kf_worker() {
    while (_running.load(std::memory_order_relaxed)) {
        bool lost = false;
        {
            std::lock_guard<std::mutex> lk(_kf_msg_mtx);
            lost = _target_lost;
        }

        if (lost) {
            _kf.reset();
            _contact_.update_target(Target{0, 0, 0.0, 0});
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        KalmanMsg km;
        uint64_t seq_now = _meas_seq.load(std::memory_order_relaxed);
        {
            // take a snapshot (short critical section)
            std::lock_guard<std::mutex> lk(_kf_msg_mtx);
            km = _kf_msg;
        }

        // NOTE: always step the filter with "now + latest gimbal" (even if no new measurement)
        // km.lower_machine_time_stamp = to_ms(this->now());  // basetime = NOW
        const bool has_new_meas = (seq_now != _meas_seq_seen);

        // step once: drive by gimbal accel; optionally fuse vision meas at NOW with robust R
        _kf.step(km, has_new_meas);
        _meas_seq_seen = seq_now;  // mark measurement consumed (if any)

        // TODO: measure / calculate the real time needed to be predict
        // TODO: get the actual delay of camera usb3
        _preview_ms.store(to_ms(this->now()) - km.img_target_time_stamp +
                          int(_cam_node->get_cam_trans_delay()));
        const int64_t horizon_ms = _preview_ms.load(std::memory_order_relaxed);  // dynamic preview
        auto [dp_deg, dy_deg] = _kf.predictDeltaAt(km, horizon_ms);

        _contact_.update_target(Target{dp_deg, dy_deg, 1.0, 0});
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void Detector::contact_worker() {
    ProjectileRx rx_last{};
    while (_running.load(std::memory_order_relaxed)) {
        // ======================== RX part =========================
        ProjectileRx rx;
        if (_contact_.latest_rx(rx)) {
            _contact_log->info(
                "latest rx: pitch {} deg, yaw {} deg, quaternion [{:.2f}, {:.2f}, {:.2f}, {:.2f}]",
                float(rx.pitch), float(rx.yaw), float(rx.q[0]), float(rx.q[1]), float(rx.q[2]),
                float(rx.q[3]));

            if (!(rx == rx_last)) {  // when there is definitely a new rx msg
                // update the kalman message after get the new rx msg from lower machine
                // protect the _kf_msg and _kf_msg_updated_flag
                std::lock_guard<std::mutex> lk(_kf_msg_mtx);
                // only fuck with data (the timestamp will be valued in the kalman thread)
                _kf_msg.lower_machine_time_stamp = to_ms(this->now());
                _kf_msg.pitch_angle = static_cast<float>(rx.pitch);
                _kf_msg.yaw_angle = static_cast<float>(rx.yaw);
                _kf_msg.roll_angle = static_cast<float>(rx.roll);
                _kf_msg.q[0] = rx.q[0];
                _kf_msg.q[1] = rx.q[1];
                _kf_msg.q[2] = rx.q[2];
                _kf_msg.q[3] = rx.q[3];
                ++_kf_msg.rx_seq;
                rx_last = rx;
            }
            // else do nothing
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // ====================== TX part ========================
        // the sending logic has been compltete in the contact class's thread
        // the command value has been update in the kalman thread (see above)
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
    if (!_contact_.start(true)) {  // spawn_thread set as true here, so that we don't need to
                                   // call run() to block
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
    _th_contact = std::thread([this] { this->contact_worker(); });
    _kf = Kalman();
    _kf.loadFromToml("/home/inubashiri/17_DART_AUTOAIM/ros_ws/config.toml", "kalman");
    _th_kf = std::thread([this] { this->kf_worker(); });

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
    _detector_log->info("SHOW_CV_CAL_DELAY: {}", _config.SHOW_CV_CAL_DELAY);
    _detector_log->info("avg_frame_delay_num: {}", _config.avg_frame_delay_num);
    _detector_log->info("SHOW_TARGET_ANGLE: {}", _config.SHOW_TARGET_ANGLE);
    _detector_log->info("=========== configs end ============");
}
