#include <iostream>
#include <string>

#include "CamWrapper.h"
#include "CamWrapperDH.h"
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std;
using namespace cv;

// #define SHOW_CAM //此节点是否展示相机
#define is_rotate  // 是否旋转相机

#define ROI_width 1024
#define ROI_height 768
#define sensor_width 1280
#define sensor_height 1024
#define nBinning 1
#define FPS 100.0

bool FOR_PC = true;

Camera *camera = nullptr;

Mat img_src;
std::string get_flag_option(const std::vector<std::string> &args, const std::string &option) {
    auto it = std::find(args.begin(), args.end(), option);
    if (it != args.end()) return *(++it);
}

class CameraPublisher : public rclcpp::Node {
   public:
    CameraPublisher(int argc, char **argv) : Node("camera_publisher") {
        if (FOR_PC) {
            camera = new DHCamera("FGV22100004");
            camera->init((sensor_width / nBinning - ROI_width) / 2,
                         (sensor_height / nBinning - ROI_height) / 2, ROI_width, ROI_height, 1500,
                         16, false, FPS, nBinning);
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
    void publish_camera_info() {
        sensor_msgs::msg::CameraInfo cam_p_0;
        cv::Mat cameraMatrix =
            (cv::Mat_<double>(3, 3) << 1557.2 / nBinning, 0.2065,
             (638.7311 / nBinning) / ((((float)sensor_width) / nBinning) / ROI_width), 0,
             1557.5 / nBinning,
             (515.1176 / nBinning) / ((((float)sensor_height) / nBinning) / ROI_height), 0, 0, 1);
        cv::Mat distCoeffs =
            (cv::Mat_<double>(1, 5) << -0.1295, 0.0804, 4.85E-04, 6.37E-04, 0.2375);
        cv::Mat dst = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs,
                                                    cv::Size(ROI_width, ROI_height), 0);
        cam_p_0.height = ROI_height;
        cam_p_0.width = ROI_width;
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
    void publish_camera() {
        camera->read(img_src);
        if (img_src.empty()) {
            RCLCPP_WARN(rclcpp::get_logger("camera_publisher"), "No image");
            return;
        }
        sensor_msgs::msg::Image _img_msg;
#ifdef is_rotate
        cv::rotate(img_src, img_src, cv::ROTATE_180);
#endif
        std_msgs::msg::Header _header;
        cv_bridge::CvImage _cv_bridge;
        _header.stamp = this->get_clock()->now();
        _header.frame_id = std::string("camera_optical_frame");
        _cv_bridge = cv_bridge::CvImage(_header, sensor_msgs::image_encodings::BGR8, img_src);
        _cv_bridge.toImageMsg(_img_msg);

        _image_pub_->publish(_img_msg);
#ifdef SHOW_CAM
        imshow("dst", img_src);
        waitKey(1);
#endif
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    system("clear");
    cout << "                                                  \n"
            " cccccccc   ii               ll   ll              \n"
            "cc                aaaaaa     ll   ll      ooooooo \n"
            "cc          ii         aa    ll   ll     oo     oo\n"
            "cc          ii   aaaaaaaa    ll   ll     oo     oo\n"
            "cc          ii   aa    aa    ll   ll     oo     oo\n"
            " cccccccc   ii    aaaaaa a    ll   ll     ooooooo \n"
            "                                                  \n";
    rclcpp::spin(std::make_shared<CameraPublisher>(argc, argv));
    rclcpp::shutdown();
    return 0;
}
