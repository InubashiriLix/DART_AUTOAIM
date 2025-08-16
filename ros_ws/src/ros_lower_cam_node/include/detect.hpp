#pragma once

#include <atomic>
#include <opencv2/opencv.hpp>
#define LOWER_CAM_D_SCALER_X 1.0f
#define LOWER_CAM_D_SCALER_Y 1.0f

namespace lower_cam_detector {

typedef struct __attribute__((packed)) {
    float dx;
    float dy;
} point_2d;

class GreenSpotDetector final {
   public:
    explicit GreenSpotDetector(int cam_index_ = 0,
                               std::string windows_name_ = "lower cam -  Greeen Tracker");
    ~GreenSpotDetector();

    int start();
    void run();
    void stop();

    const point_2d get_center();

   private:
    int cam_index = -1;
    std::string windows_name = "untitled";

    cv::VideoCapture cap;

    point_2d center_target;

    std::atomic<bool> running_vision_thread{false};
};

}  // namespace lower_cam_detector
