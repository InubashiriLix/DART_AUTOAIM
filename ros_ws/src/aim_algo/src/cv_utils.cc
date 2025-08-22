#include "detector.hpp"

void Detector::init_white_lamp_detector(const WhiteLampParams& p) {
    wl_ = p;
    if (wl_.k_open.area() > 0) kOpen_ = cv::getStructuringElement(cv::MORPH_RECT, wl_.k_open);
    if (wl_.k_close.area() > 0) kClose_ = cv::getStructuringElement(cv::MORPH_RECT, wl_.k_close);
    time_stamps.clear();
}

bool Detector::detect_white_lamp(const cv::Mat& bgr, cv::Point2f& center_px, cv::Rect& bbox,
                                 cv::Mat* debug) {
    // 颜色空间与通道
    cv::cvtColor(bgr, ycrcb_, cv::COLOR_BGR2YCrCb);
    cv::Mat ch[3];
    cv::split(ycrcb_, ch);  // ch[0]=Y, ch[1]=Cr, ch[2]=Cb
    Y_ = ch[0];
    Cr_ = ch[1];
    Cb_ = ch[2];

    // 阈值
    cv::threshold(Y_, mY_, wl_.Y_min, 255, cv::THRESH_BINARY);
    cv::absdiff(Cr_, cv::Scalar(128), tmpCr_);
    cv::absdiff(Cb_, cv::Scalar(128), tmpCb_);
    cv::threshold(tmpCr_, mCr_, wl_.Cr_tol, 255, cv::THRESH_BINARY_INV);
    cv::threshold(tmpCb_, mCb_, wl_.Cb_tol, 255, cv::THRESH_BINARY_INV);
    mask_ = mY_ & mCr_ & mCb_;

    // 形态学
    if (!kOpen_.empty()) cv::morphologyEx(mask_, mask_, cv::MORPH_OPEN, kOpen_);
    if (!kClose_.empty()) cv::morphologyEx(mask_, mask_, cv::MORPH_CLOSE, kClose_);

    // 轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    int best = -1;
    double best_score = -1.0;
    for (int i = 0; i < (int)contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area < wl_.min_area) continue;
        cv::Rect box = cv::boundingRect(contours[i]);
        double solidity = area / std::max(1.0, (double)box.area());
        if (solidity < wl_.min_solidity) continue;
        if (area > best_score) {
            best_score = area;
            best = i;
        }
    }

    // —— 没有目标：可以画调试图，但必须 return
    if (best < 0) {
        if (debug) {
            bgr.copyTo(*debug);
            // 也可以把 mask 侧显出来帮助调参（可选）：
            // cv::Mat mask_vis; cv::cvtColor(mask_, mask_vis, cv::COLOR_GRAY2BGR);
            // cv::addWeighted(*debug, 1.0, mask_vis, 0.3, 0, *debug);
            cv::putText(*debug, "NO TARGET", {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 0, 255},
                        2);
        }
        return false;
    }

    cv::Moments m = cv::moments(contours[best]);
    if (m.m00 <= 1e-3) {
        if (debug) {
            bgr.copyTo(*debug);
            cv::putText(*debug, "INVALID MOMENTS", {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                        {0, 0, 255}, 2);
        }
        return false;
    }

    center_px = {float(m.m10 / m.m00), float(m.m01 / m.m00)};
    bbox = cv::boundingRect(contours[best]);

    if (debug) {
        bgr.copyTo(*debug);
        cv::rectangle(*debug, bbox, {0, 255, 255}, 2);
        cv::circle(*debug, center_px, 4, {0, 0, 255}, -1);
        cv::putText(*debug, cv::format("(%.1f, %.1f)", center_px.x, center_px.y),
                    {bbox.x, std::max(0, bbox.y - 8)}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    {255, 255, 255}, 1);
    }
    return true;
}
