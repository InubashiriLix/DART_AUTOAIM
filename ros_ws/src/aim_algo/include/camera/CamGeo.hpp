#pragma once
#include <cmath>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "utils/logging.hpp"

namespace geom {

/**
 * 坐标系约定（OpenCV常用）：
 * - 图像像素：u 向右，v 向下
 * - 相机系：z 向前，x 向右，y 向下
 * - yaw>0 右转；pitch>0 向上（因此 pitch = atan2(-y, sqrt(x^2+z^2))）
 */
struct CameraGeometry {
    cv::Mat K;               // 3x3 内参，CV_64F
    cv::Mat D;               // 1xN 畸变，CV_64F（N=4/5/8...）
    bool rectified = false;  // 若图像已用 P 做了remap，则设 true，并把 K=P, D=0
    cv::Mat R_cam2gimbal;    // 3x3 相机→云台旋转，CV_64F（默认单位阵）

    int image_width = 0;
    int image_height = 0;

    CameraGeometry() {
        K = cv::Mat::eye(3, 3, CV_64F);
        D = cv::Mat::zeros(1, 5, CV_64F);
        R_cam2gimbal = cv::Mat::eye(3, 3, CV_64F);
    }

    void print(std::shared_ptr<spdlog::logger> logger) const;

    // 像素 → 归一化平面坐标 (x_n, y_n)（z=1）
    bool pixelToNormalized(const cv::Point2f& uv, cv::Point2d& xy_n) const;

    // 像素 → 相机系视线向量
    // unit=true 返回单位向量；否则返回 [x_n, y_n, 1]
    bool pixelToRayCam(const cv::Point2f& uv, cv::Vec3d& ray_cam, bool unit = true) const;

    // 像素 → 云台系视线向量（通过 R_cam2gimbal 旋转）
    bool pixelToRayGimbal(const cv::Point2f& uv, cv::Vec3d& ray_gim, bool unit = true) const;

    // 视线 → (yaw, pitch)（度）
    static inline void rayToYawPitchDeg(const cv::Vec3d& ray, double& yaw_deg, double& pitch_deg) {
        const double x = ray[0], y = ray[1], z = ray[2];
        yaw_deg = std::atan2(x, z) * 180.0 / CV_PI;
        pitch_deg = std::atan2(-y, std::sqrt(x * x + z * z)) * 180.0 / CV_PI;
    }

    // 直接：像素 → (yaw, pitch)（度，云台系）
    bool pixelToYawPitchDeg(const cv::Point2f& uv, double& yaw_deg, double& pitch_deg) const;

    // 用像素宽/高估距离（简单针孔）
    inline double estimateZFromPixelWidth(double pixel_width, double real_width_m) const {
        const double fx = K.at<double>(0, 0);
        if (pixel_width <= 1e-6) return -1.0;
        return fx * real_width_m / pixel_width;
    }
    inline double estimateZFromPixelHeight(double pixel_height, double real_height_m) const {
        const double fy = K.at<double>(1, 1);
        if (pixel_height <= 1e-6) return -1.0;
        return fy * real_height_m / pixel_height;
    }

    // 指定 Z 时，反投影像素 → 相机系点 (X,Y,Z) = (x_n*Z, y_n*Z, Z)
    bool pixelToPointCam_Z(const cv::Point2f& uv, double Z, cv::Point3d& Pt_cam) const;

    // 工具
    static inline cv::Vec3d normalize3(const cv::Vec3d& v) {
        const double n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        if (n <= 1e-12) return cv::Vec3d(0, 0, 0);
        return cv::Vec3d(v[0] / n, v[1] / n, v[2] / n);
    }
};

}  // namespace geom
