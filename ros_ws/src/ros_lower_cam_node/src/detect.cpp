#include <detect.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>

const lower_cam_detector::point_2d lower_cam_detector::GreenSpotDetector::get_center() {
    return center_target;
}

int lower_cam_detector::GreenSpotDetector::start() {
    if (cam_index < 0) {
        std::cerr << "Camera index is invalid" << std::endl;
        return -1;
    }

    cap.open(cam_index);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open camera" << std::endl;
        return -1;
    }
    running_vision_thread = true;
    return 0;
}

lower_cam_detector::GreenSpotDetector::GreenSpotDetector(int cam_index_,
                                                         std::string windows_name_) {
    if (cam_index_ < 0) {
        std::cerr << "Camera index is invalid" << std::endl;
        return;
    }
    cam_index = cam_index_;
    windows_name = windows_name_;
}

void lower_cam_detector::GreenSpotDetector::run() {
    cv::Mat frame, hsv, mask;
    cv::namedWindow("Green Tracker", cv::WINDOW_AUTOSIZE);

    while (running_vision_thread) {
        cap >> frame;
        if (frame.empty()) break;

        int width = frame.cols;
        int height = frame.rows;
        cv::Point center_img(width / 2, height / 2);

        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // 绿色高亮范围（可以微调）
        cv::Scalar lower_green(40, 50, 200);
        cv::Scalar upper_green(80, 255, 255);
        cv::inRange(hsv, lower_green, upper_green, mask);

        // 去噪点
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            auto largest = *std::max_element(
                contours.begin(), contours.end(),
                [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                    return cv::contourArea(a) < cv::contourArea(b);
                });

            cv::Rect box = cv::boundingRect(largest);
            cv::Point center(box.x + box.width / 2, box.y + box.height / 2);
            int dx = center.x - center_img.x;
            int dy = center.y - center_img.y;

            // FIXME: why the dx and dy use int instead of float?
            center_target.dx = dx / LOWER_CAM_D_SCALER_X;
            center_target.dy = dy / LOWER_CAM_D_SCALER_Y;

            cv::rectangle(frame, box, cv::Scalar(0, 255, 0), 2);
            cv::circle(frame, center, 4, cv::Scalar(255, 0, 0), -1);
            cv::putText(frame, "dx: " + std::to_string(dx) + " dy: " + std::to_string(dy),
                        cv::Point(box.x, box.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(255, 255, 255), 2);
        }

        cv::circle(frame, center_img, 4, cv::Scalar(0, 0, 255), -1);
        cv::imshow("Green Tracker", frame);

        // // 按下 q 退出
        // if (cv::waitKey(1) == 'q') break;
    }

    running_vision_thread = false;
    cap.release();
    cv::destroyAllWindows();
}

lower_cam_detector::GreenSpotDetector::~GreenSpotDetector() {
    stop();

    if (cap.isOpened()) {
        cap.release();
    }
    // FIXME: destroy all? (lix 0404)
    cv::destroyAllWindows();
}

void lower_cam_detector::GreenSpotDetector::stop() { running_vision_thread = false; }
