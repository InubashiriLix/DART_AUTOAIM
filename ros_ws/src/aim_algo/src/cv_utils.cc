#include "detector.hpp"

void Detector::init_white_lamp_detector(const WhiteLampParams& p) {
    wl_ = p;
    if (wl_.k_open.area() > 0) kOpen_ = cv::getStructuringElement(cv::MORPH_RECT, wl_.k_open);
    if (wl_.k_close.area() > 0) kClose_ = cv::getStructuringElement(cv::MORPH_RECT, wl_.k_close);
}

// 纯检测：返回是否找到；输出 center_px（像素坐标）与 bbox
bool Detector::detect_white_lamp(const cv::Mat& bgr, cv::Point2f& center_px, cv::Rect& bbox,
                                 cv::Mat* debug /*可选，传入则画可视化*/) {
    // 颜色空间与通道
    cv::cvtColor(bgr, ycrcb_, cv::COLOR_BGR2YCrCb);
    cv::Mat ch[3];
    cv::split(ycrcb_, ch);  // ch[0]=Y, ch[1]=Cr, ch[2]=Cb
    Y_ = ch[0];
    Cr_ = ch[1];
    Cb_ = ch[2];

    // 阈值：Y 高、Cr/Cb 接近 128（低色度）
    cv::threshold(Y_, mY_, wl_.Y_min, 255, cv::THRESH_BINARY);
    cv::absdiff(Cr_, cv::Scalar(128), tmpCr_);
    cv::absdiff(Cb_, cv::Scalar(128), tmpCb_);
    cv::threshold(tmpCr_, mCr_, wl_.Cr_tol, 255, cv::THRESH_BINARY_INV);
    cv::threshold(tmpCb_, mCb_, wl_.Cb_tol, 255, cv::THRESH_BINARY_INV);

    // 合并掩码
    mask_ = mY_ & mCr_ & mCb_;

    // 形态学（可选）
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
        // 简单用面积打分（也可加入亮度均值提高稳定性）
        if (area > best_score) {
            best_score = area;
            best = i;
        }
    }
    if (best < 0) return false;

    // 计算中心（质心比矩形中心稳）
    cv::Moments m = cv::moments(contours[best]);
    if (m.m00 <= 1e-3) return false;

    center_px = cv::Point2f(float(m.m10 / m.m00), float(m.m01 / m.m00));
    bbox = cv::boundingRect(contours[best]);

    // 可视化（可选）
    if (debug) {
        bgr.copyTo(*debug);
        cv::rectangle(*debug, bbox, cv::Scalar(0, 255, 255), 2);
        cv::circle(*debug, center_px, 4, cv::Scalar(0, 0, 255), -1);
        cv::putText(*debug, cv::format("(%.1f, %.1f)", center_px.x, center_px.y),
                    {bbox.x, std::max(0, bbox.y - 8)}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(255, 255, 255), 1);
    }
    return true;
}
