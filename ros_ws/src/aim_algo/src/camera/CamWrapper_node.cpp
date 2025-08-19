#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <numeric>
#include <string>
#include <thread>

#include "camera/CamWrapper.h"
#include "camera/CamWrapperDH.h"
#include "config_parser.hpp"
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std;
using namespace cv;
using namespace std::chrono_literals;

auto config = camera_config();
Camera *camera = nullptr;

std::string get_flag_option(const std::vector<std::string> &args, const std::string &option) {
    auto it = std::find(args.begin(), args.end(), option);
    if (it != args.end() && ++it != args.end()) return *it;
    return "";
}

class CameraPublisher : public rclcpp::Node {
   public:
    // 构造里只做“轻初始化”，真正开线程放到 start()
    CameraPublisher(int /*argc*/, char ** /*argv*/)
        : Node("camera_publisher", rclcpp::NodeOptions().use_intra_process_comms(true)) {
        _image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            declare_parameter<std::string>("camera_topic", "image_raw"),
            rclcpp::SensorDataQoS().keep_last(1).best_effort());

        auto qos_info = rclcpp::QoS(rclcpp::KeepLast(1))
                            .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                            .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

        rclcpp::PublisherOptions info_opts;
        info_opts.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;

        _camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "camera_info", qos_info, info_opts);

        _info_timer_ =
            this->create_wall_timer(100ms, std::bind(&CameraPublisher::publish_camera_info, this));
    }

    bool start() {
        if (_running.load()) return true;

        if (config.FOR_PC) {
            camera = new DHCamera(config.SN);
            bool ok = camera->init((config.sensor_width / config.nBinning - config.ROI_width) / 2,
                                   (config.sensor_height / config.nBinning - config.ROI_height) / 2,
                                   config.ROI_width, config.ROI_height, 1500, 16, false, config.FPS,
                                   config.nBinning);
            if (!ok || !camera->start()) {
                RCLCPP_WARN(this->get_logger(), "No camera");
                return false;
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "FOR_PC=false, not opening hardware camera.");
            return false;
        }

        _prepare_image_msg();

        _running.store(true);

        // 工作线程：采集→（可旋转）→ 发布
        _th_worker = std::thread([this] { this->worker_loop(); });

        // UI 线程（可选）：只取最新帧，限频显示，不影响发布
        if (config.SHOW_CV_MONITOR_WINDOWS) {
            _th_ui = std::thread([this] { this->ui_loop(); });
        }
        return true;
    }

    void stop() {
        if (!_running.exchange(false)) return;

        if (_th_worker.joinable()) _th_worker.join();
        if (_th_ui.joinable()) _th_ui.join();

        if (camera) {
            RCLCPP_INFO(this->get_logger(), "closing camera instance on quitting node");
            camera->stop();
            delete camera;
            camera = nullptr;
        }
    }

    ~CameraPublisher() override { stop(); }

   private:
    // ======== ROS 资源 ========
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_info_pub_;
    rclcpp::TimerBase::SharedPtr _info_timer_;

    // ======== 线程 & 同步 ========
    std::thread _th_worker;
    std::thread _th_ui;
    std::atomic<bool> _running{false};

    // 最新帧给 UI 线程（不走热路径），用轻量锁保护即可
    std::mutex _latest_mtx;
    std::shared_ptr<cv::Mat> _latest_frame_for_ui;

    // 复用的消息缓冲，避免每帧分配
    sensor_msgs::msg::Image _img_msg_buf;

    // 采集 + 发布：尽量把热路径做轻
    void worker_loop() {
        cv::Mat frame;
        vector<double> time_stamps = {};
        while (_running.load(std::memory_order_relaxed)) {
            auto start_tp = this->now();
            if (!camera->read(frame) || frame.empty()) {
                std::this_thread::sleep_for(1ms);
                continue;
            }

            if (config.IS_ROTATE) cv::rotate(frame, frame, cv::ROTATE_180);

            auto now = this->get_clock()->now();
            fill_image_msg(_img_msg_buf, now, frame);
            _image_pub_->publish(_img_msg_buf);

            if (config.SHOW_CV_MONITOR_WINDOWS) {
                auto cp = std::make_shared<cv::Mat>(
                    frame);  // ui thread may modify the original img, so just copy it
                std::lock_guard<std::mutex> lk(_latest_mtx);
                _latest_frame_for_ui = std::move(cp);
            }

            auto end = this->now();
            auto ms_used = (end - start_tp).nanoseconds() / 1e6;

            time_stamps.push_back(ms_used);
            auto delay_avg =
                std::accumulate(time_stamps.begin(), time_stamps.end(), 0.0) / time_stamps.size();
            if (time_stamps.size() > 3000) {
                RCLCPP_INFO(this->get_logger(), "avg img delay: used: %.3f ms", delay_avg);
                time_stamps.clear();  // 保持 300 帧的平均
            }
        }
    }

    // UI 线程：限频显示（10~30 FPS），避免 GUI/vsync 拖慢热路径
    void ui_loop() {
        cv::setNumThreads(1);
        const int ui_interval_ms = 50;  // 20 FPS 预览
        while (_running.load(std::memory_order_relaxed)) {
            std::shared_ptr<cv::Mat> f;
            {
                std::lock_guard<std::mutex> lk(_latest_mtx);
                f.swap(_latest_frame_for_ui);  // 取一次清掉
            }
            if (f && !f->empty()) {
                cv::Mat view = *f;
                // 仅用于监视：加亮度/增益，不影响发布帧
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
        // 收尾
        cv::destroyWindow("dst");
    }

    // 只负责 CameraInfo（不在热路径）
    void publish_camera_info() {
        sensor_msgs::msg::CameraInfo cam;
        // K、D、P 与你原始代码一致（修正 P 为 3x4=12 个元素）
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1557.2 / config.nBinning, 0.2065,
                     (638.7311 / config.nBinning) /
                         ((((double)config.sensor_width) / config.nBinning) / config.ROI_width),
                     0, 1557.5 / config.nBinning,
                     (515.1176 / config.nBinning) /
                         ((((double)config.sensor_height) / config.nBinning) / config.ROI_height),
                     0, 0, 1);
        cv::Mat D = (cv::Mat_<double>(1, 5) << -0.1295, 0.0804, 4.85E-04, 6.37E-04, 0.2375);
        cv::Mat P3x3 =
            getOptimalNewCameraMatrix(K, D, Size(config.ROI_width, config.ROI_height), 0);

        cam.height = config.ROI_height;
        cam.width = config.ROI_width;
        cam.distortion_model = "plumb_bob";

        cam.k = {K.at<double>(0, 0), K.at<double>(0, 1), K.at<double>(0, 2),
                 K.at<double>(1, 0), K.at<double>(1, 1), K.at<double>(1, 2),
                 K.at<double>(2, 0), K.at<double>(2, 1), K.at<double>(2, 2)};

        cam.d = {D.at<double>(0, 0), D.at<double>(0, 1), D.at<double>(0, 2), D.at<double>(0, 3),
                 D.at<double>(0, 4)};

        // P 需要 3x4，共 12 项；这里将 3x3 的 P3x3 塞在左上角，最后一列 [0,0,0,1]^T
        cam.p = {P3x3.at<double>(0, 0), P3x3.at<double>(0, 1), P3x3.at<double>(0, 2), 0.0,
                 P3x3.at<double>(1, 0), P3x3.at<double>(1, 1), P3x3.at<double>(1, 2), 0.0,
                 P3x3.at<double>(2, 0), P3x3.at<double>(2, 1), P3x3.at<double>(2, 2), 1.0};

        cam.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        cam.binning_x = 0;
        cam.binning_y = 0;

        cam.header.stamp = this->get_clock()->now();
        cam.header.frame_id = "camera_optical_frame";
        _camera_info_pub_->publish(cam);
    }

    void _prepare_image_msg() {
        _img_msg_buf.header.frame_id = "camera_optical_frame";
        _img_msg_buf.height = config.ROI_height;
        _img_msg_buf.width = config.ROI_width;
        _img_msg_buf.encoding = sensor_msgs::image_encodings::BGR8;
        _img_msg_buf.is_bigendian = false;
        _img_msg_buf.step =
            static_cast<sensor_msgs::msg::Image::_step_type>(_img_msg_buf.width * 3);
        _img_msg_buf.data.resize(_img_msg_buf.step * _img_msg_buf.height);
    }

    inline void fill_image_msg(sensor_msgs::msg::Image &msg, const rclcpp::Time &stamp,
                               const cv::Mat &bgr) {
        msg.header.stamp = stamp;

        if (bgr.cols != (int)msg.width || bgr.rows != (int)msg.height || bgr.type() != CV_8UC3) {
            msg.width = bgr.cols;
            msg.height = bgr.rows;
            msg.step = msg.width * 3;
            msg.data.resize(msg.step * msg.height);
        }

        std::memcpy(msg.data.data(), bgr.data, msg.step * msg.height);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    system("clear");
    cout << "\n"
            "░█▀▀░▀█▀░█▀█░█░░░█░░░█▀█░█░░░█▀▀░█▀█░█▄█░▀█░\n"
            "░█░░░░█░░█▀█░█░░░█░░░█░█░▀░░░█░░░█▀█░█░█░░█░\n"
            "░▀▀▀░▀▀▀░▀░▀░▀▀▀░▀▀▀░▀▀▀░▀░░░▀▀▀░▀░▀░▀░▀░▀▀▀\n";

    cout << "===================================\nconfigs:\n";
    cout << "SN: " << config.SN << "\n"
         << "IS_ROTATE: " << (config.IS_ROTATE ? "true" : "false") << "\n"
         << "FOR_PC: " << (config.FOR_PC ? "true" : "false") << "\n"
         << "SHOW_IMG: " << (config.SHOW_CV_MONITOR_WINDOWS ? "true" : "false") << "\n"
         << "MONITOR img GAIN: ";
    for (size_t i = 0; i < config.MONITOR_IMG_GAIN.size(); ++i)
        cout << config.MONITOR_IMG_GAIN[i] << (i + 1 < config.MONITOR_IMG_GAIN.size() ? ", " : "");
    cout << "\nROI_width: " << config.ROI_width << "\nROI_height: " << config.ROI_height
         << "\nsensor_width: " << config.sensor_width << "\nsensor_height: " << config.sensor_height
         << "\nnBinning: " << config.nBinning << "\nFPS: " << config.FPS
         << "\n======== end for configs =========\n";

    auto node = std::make_shared<CameraPublisher>(argc, argv);
    if (!node->start()) {
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    node->stop();
    rclcpp::shutdown();
    return 0;
}
