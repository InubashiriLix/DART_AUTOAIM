#include <cv_bridge/cv_bridge.h>

#include <atomic>
#include <boost/lockfree/spsc_queue.hpp>
#include <condition_variable>
#include <geometry_msgs/msg/twist.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <thread>

#include "camera/CamNode.hpp"
#include "config_parser.hpp"

class Semaphore {
   public:
    explicit Semaphore(int count = 0) : count_(count) {}

    void release() {
        std::unique_lock<std::mutex> lock(mtx_);
        ++count_;
        cv_.notify_one();
    }

    void acquire() {
        std::unique_lock<std::mutex> lock(mtx_);
        cv_.wait(lock, [this]() { return count_ > 0; });
        --count_;
    }

   private:
    std::mutex mtx_;
    std::condition_variable cv_;
    int count_;
};

struct KalmanMsg {
    time_t time_stamp;  // 时间戳

    // the target in the image
    float x = 0;
    float y = 0;

    // the joint state and q[4], pitch, yaw angle
    float joint_state_x = 0;
    float joint_state_y = 0;
    float pitch_angle = 0;
    float yaw_angle = 0;
    float q[4] = {0, 0, 0, 0};
};

class Detector : public rclcpp::Node {
   public:
    Detector(CameraPublisher&& cam_node);

   private:
    void welcom();

    void detector_worker();  // 视觉工作线程
    void kf_worker();        // KF/控制线程（预留）
    void commu_worker();

    bool start();
    void stop();

    std::thread _th_worker;
    std::thread _th_kf;
    std::thread _th_commu;
    std::atomic<bool> _running{true};

    std::queue<KalmanMsg> _kalman_msg_queue;
    std::mutex _kalman_mgs_mutex;
    Semaphore _sem_kalman_msg;  // Initial count 0

    // 你的配置
    detector_config _config;
    double _center_x{}, _center_y{};
    int _cam_qos_keep_last{};
};
