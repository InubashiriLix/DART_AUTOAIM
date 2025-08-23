#include <iostream>

#include "camera/CamGeo.hpp"

namespace geom {

void CameraGeometry::print() const {
    std::cout << "[CameraGeometry]\n";
    std::cout << "  rectified = " << (rectified ? "true" : "false") << "\n";
    std::cout << "  image: " << image_width << " x " << image_height << "\n";
    std::cout << "  K = \n" << K << "\n";
    std::cout << "  D = \n" << D << "\n";
    std::cout << "  R_cam2gimbal = \n" << R_cam2gimbal << "\n";
}

bool CameraGeometry::pixelToNormalized(const cv::Point2f& uv, cv::Point2d& xy_n) const {
    if (K.empty()) return false;

    if (!rectified) {
        std::vector<cv::Point2f> src(1, uv), dst;
        cv::undistortPoints(src, dst, K, D);  // 返回归一化坐标
        if (dst.empty()) return false;
        xy_n = cv::Point2d(dst[0].x, dst[0].y);
        return true;
    } else {
        const double fx = K.at<double>(0, 0);
        const double fy = K.at<double>(1, 1);
        const double cx = K.at<double>(0, 2);
        const double cy = K.at<double>(1, 2);
        if (std::abs(fx) < 1e-12 || std::abs(fy) < 1e-12) return false;
        xy_n.x = (uv.x - cx) / fx;
        xy_n.y = (uv.y - cy) / fy;
        return true;
    }
}

bool CameraGeometry::pixelToRayCam(const cv::Point2f& uv, cv::Vec3d& ray_cam, bool unit) const {
    cv::Point2d xy_n;
    if (!pixelToNormalized(uv, xy_n)) return false;
    cv::Vec3d dir(xy_n.x, xy_n.y, 1.0);
    ray_cam = unit ? normalize3(dir) : dir;
    return true;
}

bool CameraGeometry::pixelToRayGimbal(const cv::Point2f& uv, cv::Vec3d& ray_gim, bool unit) const {
    cv::Vec3d ray_cam;
    if (!pixelToRayCam(uv, ray_cam, unit)) return false;

    cv::Mat rc = (cv::Mat_<double>(3, 1) << ray_cam[0], ray_cam[1], ray_cam[2]);
    cv::Mat rg = R_cam2gimbal * rc;
    ray_gim = cv::Vec3d(rg.at<double>(0, 0), rg.at<double>(1, 0), rg.at<double>(2, 0));
    if (unit) ray_gim = normalize3(ray_gim);
    return true;
}

bool CameraGeometry::pixelToYawPitchDeg(const cv::Point2f& uv, double& yaw_deg,
                                        double& pitch_deg) const {
    cv::Vec3d ray_g;
    if (!pixelToRayGimbal(uv, ray_g, /*unit=*/true)) return false;
    rayToYawPitchDeg(ray_g, yaw_deg, pitch_deg);
    return true;
}

bool CameraGeometry::pixelToPointCam_Z(const cv::Point2f& uv, double Z, cv::Point3d& Pt_cam) const {
    if (Z <= 0) return false;
    cv::Point2d xy_n;
    if (!pixelToNormalized(uv, xy_n)) return false;
    Pt_cam = cv::Point3d(xy_n.x * Z, xy_n.y * Z, Z);
    return true;
}

}  // namespace geom
