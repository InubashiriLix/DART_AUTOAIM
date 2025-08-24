#include "camera/CamNode.hpp"

#include <chrono>
#include <iostream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <vector>

#include "camera/CamWrapper.h"
#include "camera/CamWrapperDH.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.hpp"
#include "std_msgs/msg/header.hpp"
#include "utils/logging.hpp"

using namespace std;
using namespace cv;
using namespace std::chrono_literals;

static Camera *camera = nullptr;

CameraPublisher::CameraPublisher(int /*argc*/, char ** /*argv*/)
    : Node("camera_publisher", rclcpp::NodeOptions().use_intra_process_comms(true)) {
    this->_cam_log = perflog::get("cam");
    this->config = camera_config();
    prepare_cam_info();
    _image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        declare_parameter<std::string>("camera_topic", "image_raw"),
        rclcpp::SensorDataQoS().keep_last(1).best_effort());

    auto qos_info = rclcpp::QoS(rclcpp::KeepLast(1))
                        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                        .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    rclcpp::PublisherOptions info_opts;
    info_opts.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;

    _camera_info_pub_ =
        this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", qos_info, info_opts);

    if (config.publish_camera_info)
        _info_timer_ =
            this->create_wall_timer(100ms, std::bind(&CameraPublisher::publish_camera_info, this));
    else {
        this->declare_parameter("publish_camera_info", false);
    }
}

void CameraPublisher::prepare_cam_info() {
    int nBinning = config.nBinning;
    int sensor_w = config.sensor_width;
    int sensor_h = config.sensor_height;
    int ROI_w = config.ROI_width;
    int ROI_h = config.ROI_height;
    K = (cv::Mat_<double>(3, 3) << /*0*/ 1557.2 / nBinning, /*1*/ 0.2065,
         /*2*/ (638.7311 / nBinning) / ((double(sensor_w) / nBinning) / ROI_w), /*3*/ 0,
         /*4*/ 1557.5 / nBinning,
         /*5*/ (515.1176 / nBinning) / ((double(sensor_h) / nBinning) / ROI_h), /*6*/ 0, /*7*/ 0,
         /*8*/ 1);
    this->D = (cv::Mat_<double>(1, 5) << -0.1295, 0.0804, 4.85E-04, 6.37E-04, 0.2375);
    this->P3x3 = getOptimalNewCameraMatrix(K, D, Size(config.ROI_width, config.ROI_height), 0);
}

bool CameraPublisher::start() {
    if (_running.load()) return true;

    if (config.FOR_PC) {
        camera = new DHCamera(config.SN);
        bool ok = camera->init((config.sensor_width / config.nBinning - config.ROI_width) / 2,
                               (config.sensor_height / config.nBinning - config.ROI_height) / 2,
                               config.ROI_width, config.ROI_height, 1500, 16, false, config.FPS,
                               config.nBinning);
        if (!ok || !camera->start()) {
            _cam_log->error("camera init or start failed!");
            RCLCPP_WARN(this->get_logger(), "No camera");
            return false;
        }
    } else {
        _cam_log->warn("FOR_PC=false, not opening hardware camera.");
        RCLCPP_WARN(this->get_logger(), "FOR_PC=false, not opening hardware camera.");
        return false;
    }

    _prepare_image_msg();
    _running.store(true);

    welcom();

    _th_worker = std::thread([this] { this->worker_loop(); });
    if (config.SHOW_CV_MONITOR_WINDOWS) {
        _th_ui = std::thread([this] { this->ui_loop(); });
    }
    return true;
}

void CameraPublisher::stop() {
    if (!_running.exchange(false)) return;

    if (_th_worker.joinable()) _th_worker.join();
    if (_th_ui.joinable()) _th_ui.join();

    if (camera) {
        _cam_log->info("closing camera instance on quitting node");
        RCLCPP_INFO(this->get_logger(), "closing camera instance on quitting node");
        camera->stop();
        delete camera;
        camera = nullptr;
    }
}

bool CameraPublisher::is_running() const { return _running.load(); }

sensor_msgs::msg::Image::ConstSharedPtr CameraPublisher::get_latest_iamge_msg() {
    std::lock_guard<std::mutex> lk(_latest_mtx);
    return latest_img_;
}

sensor_msgs::msg::CameraInfo CameraPublisher::get_camera_info_buf() { return _cam_info_buf; }

CameraPublisher::~CameraPublisher() { stop(); }

void CameraPublisher::welcom() {
    _cam_log->info("CameraPublisher Node started.");
    _cam_log->info(
        "\n"
        "░█▀▀░▀█▀░█▀█░█░░░█░░░█▀█░█░░░█▀▀░█▀█░█▄█░▀█░\n"
        "░█░░░░█░░█▀█░█░░░█░░░█░█░▀░░░█░░░█▀█░█░█░░█░\n"
        "░▀▀▀░▀▀▀░▀░▀░▀▀▀░▀▀▀░▀▀▀░▀░░░▀▀▀░▀░▀░▀░▀░▀▀▀\n");
    _cam_log->info("============= configs ================");
    _cam_log->info("SN: {}", config.SN);
    _cam_log->info("IS_ROTATE: {}", config.IS_ROTATE ? "true" : "false");
    _cam_log->info("FOR_PC: {}", config.FOR_PC ? "true" : "false");
    _cam_log->info("SHOW_IMG: {}", config.SHOW_CV_MONITOR_WINDOWS ? "true" : "false");
    _cam_log->info("nBinning: {}", config.nBinning);
    _cam_log->info("MONITOR img GAIN: {}", fmt::join(config.MONITOR_IMG_GAIN, ", "));
    _cam_log->info("ROI_width: {}", config.ROI_width);
    _cam_log->info("ROI_height: {}", config.ROI_height);
    _cam_log->info("sensor_width: {}", config.sensor_width);
    _cam_log->info("sensor_height: {}", config.sensor_height);
    _cam_log->info("FPS: {}", config.FPS);
    _cam_log->info("publish_image_msg: {}", config.publish_image_msg ? "true" : "false");
    _cam_log->info("publish_camera_info: {}", config.publish_camera_info ? "true" : "false");
    _cam_log->info("======== end for configs =========");

    // std::cout << "\n"
    //              "░█▀▀░▀█▀░█▀█░█░░░█░░░█▀█░█░░░█▀▀░█▀█░█▄█░▀█░\n"
    //              "░█░░░░█░░█▀█░█░░░█░░░█░█░▀░░░█░░░█▀█░█░█░░█░\n"
    //              "░▀▀▀░▀▀▀░▀░▀░▀▀▀░▀▀▀░▀▀▀░▀░░░▀▀▀░▀░▀░▀░▀░▀▀▀\n";
    // std::cout << "===================================\nconfigs:\n";
    // std::cout << "SN: " << config.SN << "\n"
    //           << "IS_ROTATE: " << (config.IS_ROTATE ? "true" : "false") << "\n"
    //           << "FOR_PC: " << (config.FOR_PC ? "true" : "false") << "\n"
    //           << "SHOW_IMG: " << (config.SHOW_CV_MONITOR_WINDOWS ? "true" : "false") << "\n"
    //           << "MONITOR img GAIN: ";
    // for (size_t i = 0; i < config.MONITOR_IMG_GAIN.size(); ++i)
    //     std::cout << config.MONITOR_IMG_GAIN[i]
    //               << (i + 1 < config.MONITOR_IMG_GAIN.size() ? ", " : "");
    // std::cout << "\nROI_width: " << config.ROI_width << "\nROI_height: " << config.ROI_height
    //           << "\nsensor_width: " << config.sensor_width
    //           << "\nsensor_height: " << config.sensor_height << "\nnBinning: " << config.nBinning
    //           << "\nFPS: " << config.FPS
    //           << "\npublish_image_msg: " << (config.publish_image_msg ? "true" : "false")
    //           << "\npublish_camera_info:" << (config.publish_camera_info ? " true " : " false ")
    //           << "\n======== end for configs =========\n";
}

void CameraPublisher::worker_loop() {
    using clock_t = std::chrono::steady_clock;
    const auto period = std::chrono::milliseconds(8);  // = 100 Hz 上限

    cv::Mat frame;
    std::vector<double> time_stamps;
    time_stamps.reserve(512);

    auto next_tick = clock_t::now();  // 首次基准

    // 可选：减少积压（如果驱动支持，建议把缓冲设小）
    // if (camera) { camera->set(cv::CAP_PROP_BUFFERSIZE, 1); }

    while (_running.load(std::memory_order_relaxed)) {
        auto start_tp_ros = this->now();
        auto start_tp_steady = clock_t::now();

        if (!camera->read(frame) || frame.empty()) {
            next_tick = start_tp_steady + period;
            std::this_thread::sleep_until(next_tick);
            continue;
        }

        if (config.IS_ROTATE) cv::rotate(frame, frame, cv::ROTATE_180);

        auto msg = std::make_shared<sensor_msgs::msg::Image>();
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera_optical_frame";
        msg->height = frame.rows;
        msg->width = frame.cols;
        msg->encoding = sensor_msgs::image_encodings::BGR8;
        msg->is_bigendian = false;
        msg->step = frame.cols * 3;
        msg->data.resize(static_cast<size_t>(msg->step) * msg->height);
        std::memcpy(msg->data.data(), frame.data, msg->data.size());

        {
            std::lock_guard<std::mutex> lk(_latest_mtx);
            latest_img_ = msg;

            if (config.SHOW_CV_MONITOR_WINDOWS) {
                _latest_frame_for_ui = std::make_shared<cv::Mat>(frame);
            }
        }

        if (config.publish_image_msg) {
            _image_pub_->publish(*msg);
        }

        auto end_ros = this->now();
        double ms_used = (end_ros - start_tp_ros).nanoseconds() / 1e6;
        time_stamps.push_back(ms_used);
        if (time_stamps.size() >= config.avg_frame_delay_num) {
            double sum = std::accumulate(time_stamps.begin(), time_stamps.end(), 0.0);
            double delay_avg = sum / time_stamps.size();
            _cam_log->info("[frame camera] avg {} frame delay: {:.3f} ms",
                           config.avg_frame_delay_num, delay_avg);
            time_stamps.clear();
        }

        next_tick += period;
        auto now_steady = clock_t::now();
        if (now_steady < next_tick) {
            std::this_thread::sleep_until(next_tick);
        } else if (now_steady - next_tick > period) {
            next_tick = now_steady;
        }
    }
}

void CameraPublisher::ui_loop() {
    cv::setNumThreads(1);
    const int ui_interval_ms = 50;
    while (_running.load(std::memory_order_relaxed)) {
        std::shared_ptr<cv::Mat> f;
        {
            std::lock_guard<std::mutex> lk(_latest_mtx);
            f.swap(_latest_frame_for_ui);
        }
        if (f && !f->empty()) {
            cv::Mat view = *f;
            if (!config.MONITOR_IMG_GAIN.empty()) {
                cv::Scalar g(0, 0, 0);
                for (int i = 0; i < 3 && i < (int)config.MONITOR_IMG_GAIN.size(); ++i)
                    g[i] = config.MONITOR_IMG_GAIN[i];
                view = view + g;
            }
            cv::imshow("dst", view);
            cv::waitKey(1);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(ui_interval_ms));
    }
    cv::destroyWindow("dst");
}

void CameraPublisher::publish_camera_info() {
    _cam_info_buf.height = config.ROI_height;
    _cam_info_buf.width = config.ROI_width;
    _cam_info_buf.distortion_model = "plumb_bob";

    _cam_info_buf.k = {K.at<double>(0, 0), K.at<double>(0, 1), K.at<double>(0, 2),
                       K.at<double>(1, 0), K.at<double>(1, 1), K.at<double>(1, 2),
                       K.at<double>(2, 0), K.at<double>(2, 1), K.at<double>(2, 2)};

    _cam_info_buf.d = {D.at<double>(0, 0), D.at<double>(0, 1), D.at<double>(0, 2),
                       D.at<double>(0, 3), D.at<double>(0, 4)};

    _cam_info_buf.p = {P3x3.at<double>(0, 0), P3x3.at<double>(0, 1), P3x3.at<double>(0, 2), 0.0,
                       P3x3.at<double>(1, 0), P3x3.at<double>(1, 1), P3x3.at<double>(1, 2), 0.0,
                       P3x3.at<double>(2, 0), P3x3.at<double>(2, 1), P3x3.at<double>(2, 2), 1.0};

    _cam_info_buf.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    _cam_info_buf.binning_x = 0;
    _cam_info_buf.binning_y = 0;

    _cam_info_buf.header.stamp = this->get_clock()->now();
    _cam_info_buf.header.frame_id = "camera_optical_frame";
    _camera_info_pub_->publish(_cam_info_buf);
}

const cv::Mat CameraPublisher::get_cam_info_k() { return this->K.clone(); }
const cv::Mat CameraPublisher::get_cam_info_d() { return this->D.clone(); }
const cv::Mat CameraPublisher::get_cam_info_p3x3() { return this->P3x3.clone(); }

void CameraPublisher::_prepare_image_msg() {
    _img_msg_buf.header.frame_id = "camera_optical_frame";
    _img_msg_buf.height = config.ROI_height;
    _img_msg_buf.width = config.ROI_width;
    _img_msg_buf.encoding = sensor_msgs::image_encodings::BGR8;
    _img_msg_buf.is_bigendian = false;
    _img_msg_buf.step = static_cast<sensor_msgs::msg::Image::_step_type>(_img_msg_buf.width * 3);
    _img_msg_buf.data.resize(_img_msg_buf.step * _img_msg_buf.height);
}
