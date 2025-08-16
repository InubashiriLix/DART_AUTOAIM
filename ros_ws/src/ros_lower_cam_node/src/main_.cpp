#include <iostream>
#include <opencv2/opencv.hpp>

cv::VideoCapture cap;

// 初始化摄像头
int detect_setup() {
    cap.open(0);  // ✅ 正确打开摄像头
    if (!cap.isOpened()) {
        return -1;
    }
    return 0;
}

// 检测绿色亮点
int detect() {
    cv::Mat frame, hsv, mask;
    cv::namedWindow("Green Tracker", cv::WINDOW_AUTOSIZE);

    while (true) {
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

            cv::rectangle(frame, box, cv::Scalar(0, 255, 0), 2);
            cv::circle(frame, center, 4, cv::Scalar(255, 0, 0), -1);
            cv::putText(frame, "dx: " + std::to_string(dx) + " dy: " + std::to_string(dy),
                        cv::Point(box.x, box.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(255, 255, 255), 2);
        }

        cv::circle(frame, center_img, 4, cv::Scalar(0, 0, 255), -1);
        cv::imshow("Green Tracker", frame);

        // 按下 q 退出
        if (cv::waitKey(1) == 'q') break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}

int main() {
    if (detect_setup() == -1) {
        std::cout << "老吴，哈！哈基摄像头找不到了." << std::endl;
        return -1;
    }

    detect();
    return 0;
}
