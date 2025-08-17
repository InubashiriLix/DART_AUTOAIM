#include <iostream>
#include <string>

#include "CamWrapper.h"
#include "CamWrapperDH.h"
#include "config.hpp"
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std;
using namespace cv;

// WARNING: you need to provide abs path for config toml;
auto config = toml_config();

Camera *camera = nullptr;

Mat img_src;
std::string get_flag_option(const std::vector<std::string> &args, const std::string &option) {
    auto it = std::find(args.begin(), args.end(), option);
    if (it != args.end()) return *(++it);
}

class CameraPublisher : public rclcpp::Node {
   public:
    /**
     * @brief constructor for camera publisher node
     *
     * @param argc
     * @param argv
     */
    CameraPublisher(int argc, char **argv) : Node("camera_publisher") {
        if (config.FOR_PC) {
            camera = new DHCamera(config.SN);
            camera->init((config.sensor_width / config.nBinning - config.ROI_width) / 2,
                         (config.sensor_height / config.nBinning - config.ROI_height) / 2,
                         config.ROI_width, config.ROI_height, 1500, 16, false, config.FPS,
                         config.nBinning);
            if (!camera->start()) {
                RCLCPP_WARN(rclcpp::get_logger("camera_publisher"), "No camera");
                exit(0);
            }
            this->declare_parameter("camera_topic", "image_raw");
            string camera_publish_topic_name;
            this->get_parameter("camera_topic", camera_publish_topic_name);

            // Image publisher: SensorDataQoS（BestEffort + 小深度）
            _image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
                camera_publish_topic_name, rclcpp::SensorDataQoS().keep_last(5));

            // CameraInfo publisher: 可靠 + “类 latch”
            _camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
                "camera_info", rclcpp::QoS(rclcpp::KeepLast(1))
                                   .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                   .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL));

            _image_timer_ =
                this->create_wall_timer(0.001s, std::bind(&CameraPublisher::publish_camera, this));
            _info_timer_ = this->create_wall_timer(
                0.01s, std::bind(&CameraPublisher::publish_camera_info, this));
        }
    }

    /**
     * @brief the destructor for camera publisher node
     */
    ~CameraPublisher() {
        if (camera != nullptr) {
            RCLCPP_INFO(this->get_logger(), "closing camera instance on quiting node");
            camera->stop();
        }
    }

   private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_info_pub_;
    rclcpp::TimerBase::SharedPtr _image_timer_;
    rclcpp::TimerBase::SharedPtr _info_timer_;

    cv::VideoCapture cap;

    /**
     * @brief 相机内参信息
     * height: 图像高度，int 类型
     * width: 图像宽度，int 类型
     * distortion_model: 畸变模型，string 类型
     * k: 相机内参矩阵（3x3），以一维数组形式存储，依次为 fx, 0, cx, 0, fy, cy, 0, 0, 1
     *      其中 fx, fy 为焦距，cx, cy 为主点坐标
     * d: 畸变系数数组
     * p: 投影矩阵（3x4），一维数组
     */
    void publish_camera_info() {
        sensor_msgs::msg::CameraInfo cam_p_0;
        cv::Mat cameraMatrix =
            (cv::Mat_<double>(3, 3) << 1557.2 / config.nBinning, 0.2065,
             (638.7311 / config.nBinning) /
                 ((((float)config.sensor_width) / config.nBinning) / config.ROI_width),
             0, 1557.5 / config.nBinning,
             (515.1176 / config.nBinning) /
                 ((((float)config.sensor_height) / config.nBinning) / config.ROI_height),
             0, 0, 1);
        cv::Mat distCoeffs =
            (cv::Mat_<double>(1, 5) << -0.1295, 0.0804, 4.85E-04, 6.37E-04, 0.2375);
        cv::Mat dst = cv::getOptimalNewCameraMatrix(
            cameraMatrix, distCoeffs, cv::Size(config.ROI_width, config.ROI_height), 0);
        cam_p_0.height = config.ROI_height;
        cam_p_0.width = config.ROI_width;
        cam_p_0.distortion_model = "plumb_bob";
        cam_p_0.k = {
            cameraMatrix.at<double>(0, 0), cameraMatrix.at<double>(0, 1),
            cameraMatrix.at<double>(0, 2), cameraMatrix.at<double>(1, 0),
            cameraMatrix.at<double>(1, 1), cameraMatrix.at<double>(1, 2),
            cameraMatrix.at<double>(2, 0), cameraMatrix.at<double>(2, 1),
            cameraMatrix.at<double>(2, 2),
        };
        cam_p_0.d = {distCoeffs.at<double>(0, 0), distCoeffs.at<double>(0, 1),
                     distCoeffs.at<double>(0, 2), distCoeffs.at<double>(0, 3),
                     distCoeffs.at<double>(0, 4)};
        cam_p_0.p = {
            dst.at<double>(0, 0), dst.at<double>(0, 1), dst.at<double>(0, 2),
            dst.at<double>(1, 0), dst.at<double>(1, 1), dst.at<double>(1, 2),
            dst.at<double>(2, 0), dst.at<double>(2, 1), dst.at<double>(2, 2),
        };
        cam_p_0.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        cam_p_0.binning_x = 0;
        cam_p_0.binning_y = 0;
        _camera_info_pub_->publish(cam_p_0);
    }

    /**
     * @brief the camera publish method
     */
    void publish_camera() {
        camera->read(img_src);
        if (img_src.empty()) {
            RCLCPP_WARN(rclcpp::get_logger("camera_publisher"), "No image");
            return;
        }
        sensor_msgs::msg::Image _img_msg;

        // is rotate
        if (config.IS_ROTATE) cv::rotate(img_src, img_src, cv::ROTATE_180);

        std_msgs::msg::Header _header;
        cv_bridge::CvImage _cv_bridge;
        _header.stamp = this->get_clock()->now();
        _header.frame_id = std::string("camera_optical_frame");
        _cv_bridge = cv_bridge::CvImage(_header, sensor_msgs::image_encodings::BGR8, img_src);
        _cv_bridge.toImageMsg(_img_msg);

        _image_pub_->publish(_img_msg);

        // show img in the system
        if (config.SHOW_CV_MONITOR_WINDOWS) {
            img_src += cv::Scalar(config.MONITOR_IMG_GAIN[0], config.MONITOR_IMG_GAIN[1],
                                  config.MONITOR_IMG_GAIN[2]);
            imshow("dst", img_src);
            waitKey(1);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    system("clear");
    cout << "\n"
            "░█▀▀░▀█▀░█▀█░█░░░█░░░█▀█░█░░░█▀▀░█▀█░█▄█░▀█░\n"
            "░█░░░░█░░█▀█░█░░░█░░░█░█░▀░░░█░░░█▀█░█░█░░█░\n"
            "░▀▀▀░▀▀▀░▀░▀░▀▀▀░▀▀▀░▀▀▀░▀░░░▀▀▀░▀░▀░▀░▀░▀▀▀\n";

    cout << "===================================" << endl;
    cout << "configs: " << endl;
    cout << "SN: " << config.SN << endl;
    cout << "IS_ROTATE: " << (config.IS_ROTATE ? "true" : "false") << endl;
    cout << "FOR_PC: " << (config.FOR_PC ? "true" : "false") << endl;
    cout << "SHOW_IMG: " << (config.SHOW_CV_MONITOR_WINDOWS ? "true" : "false") << endl;
    std::cout << "MONITOR img GAIN: ";
    for (size_t i = 0; i < config.MONITOR_IMG_GAIN.size(); ++i) {
        std::cout << config.MONITOR_IMG_GAIN[i];
        if (i + 1 < config.MONITOR_IMG_GAIN.size()) std::cout << ", ";
    }
    cout << endl;

    cout << "ROI_width: " << config.ROI_width << endl;
    cout << "ROI_height: " << config.ROI_height << endl;
    cout << "sensor_width: " << config.sensor_width << endl;
    cout << "sensor_height: " << config.sensor_height << endl;
    cout << "nBinning: " << config.nBinning << endl;
    cout << "FPS: " << config.FPS << endl;
    cout << "======== end for configs =========" << endl;

    rclcpp::spin(std::make_shared<CameraPublisher>(argc, argv));
    rclcpp::shutdown();
    return 0;
}
