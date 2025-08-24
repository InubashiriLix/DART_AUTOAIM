#include <iostream>
#include <vector>

#include "camera/CamGeo.hpp"
#include "kalman/kalman_delay_aware.hpp"

int shit1() {
    KalmanDelayAware kf("/home/inubashiri/17_DART_AUTOAIM/ros_ws/config.toml");  // 读 [kalman] 表

    // 假设我们每 33ms 来一帧观测（用 pitch/yaw 角度直接喂）
    std::int64_t t0 = 0;  // ms
    float gimbal_pitch = 0.0f, gimbal_yaw = 0.0f;

    for (int i = 1; i <= 20; ++i) {
        std::int64_t ts = t0 + i * 33;  // 图像时间戳 ms
        // 伪造一个平滑上升的目标：pitch(t)=0.01*t, yaw(t)=0.03*t (t单位为秒)
        float t = ts * 1e-3f;
        float meas_pitch =
            0.01f * t * 180.0f / float(M_PI);  // 只是演示：把弧度当作度写法没意义，这里随便给值
        float meas_yaw = 0.03f * t * 180.0f / float(M_PI);

        kf.updateAngles(meas_pitch, meas_yaw, ts);

        auto [dp, dy] = kf.getDeltaAnglesPD(gimbal_pitch, gimbal_yaw);
        gimbal_pitch += dp;
        gimbal_yaw += dy;

        float est_p, est_y, est_vp, est_vy;
        kf.getEstimate(est_p, est_y, est_vp, est_vy);

        std::cout << "t=" << ts << "ms | meas=(" << meas_pitch << "," << meas_yaw << ") | est=("
                  << est_p << "," << est_y << ") | gimbal=(" << gimbal_pitch << "," << gimbal_yaw
                  << ") | delta=(" << dp << "," << dy << ")\n";
    }
    return 0;
}

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

int shit() {
    // ====== 你的相机参数 ======
    const int ROI_w = 1024, ROI_h = 768;
    const int sensor_w = 1280, sensor_h = 1024;
    const int nBinning = 1;

    cv::Mat K =
        (cv::Mat_<double>(3, 3) << 1557.2 / nBinning, 0.2065,
         (638.7311 / nBinning) / ((double(sensor_w) / nBinning) / ROI_w), 0, 1557.5 / nBinning,
         (515.1176 / nBinning) / ((double(sensor_h) / nBinning) / ROI_h), 0, 0, 1);

    cv::Mat D = (cv::Mat_<double>(1, 5) << -0.1295, 0.0804, 4.85e-04, 6.37e-04, 0.2375);

    cv::Mat P3x3;
    {
        cv::Size img_size(ROI_w, ROI_h);
        P3x3 = cv::getOptimalNewCameraMatrix(K, D, img_size, 0.0);
    }

    // ====== 配置 CameraGeometry ======
    geom::CameraGeometry camgeo;
    // 如果你“没有做remap”（图像还是原始畸变的），就这样：
    camgeo.rectified = false;
    camgeo.K = K.clone();
    camgeo.D = D.clone();

    // 如果你“已经对图像做了remap到 P3x3”，请改用：
    // camgeo.rectified = true;
    // camgeo.K = P3x3.clone();
    // camgeo.D = cv::Mat::zeros(1,5,CV_64F);

    camgeo.image_width = ROI_w;
    camgeo.image_height = ROI_h;

    // 如果相机与云台轴有固定外参，在这里设置（默认单位阵）
    // 例如：绕X轴俯仰 +α（deg），绕Z轴偏航 +β（deg）
    auto deg2rad = [](double d) { return d * CV_PI / 180.0; };
    double alpha_deg = 0.0;  // pitch 偏置
    double beta_deg = 0.0;   // yaw 偏置
    cv::Mat Rx, Rz;
    {
        cv::Mat rvecX = (cv::Mat_<double>(3, 1) << deg2rad(alpha_deg), 0, 0);
        cv::Mat rvecZ = (cv::Mat_<double>(3, 1) << 0, 0, deg2rad(beta_deg));
        cv::Rodrigues(rvecX, Rx);
        cv::Rodrigues(rvecZ, Rz);
    }
    camgeo.R_cam2gimbal = Rz * Rx;

    camgeo.print();

    // ====== 某一帧检测出的像素点（比如质心/框中心） ======
    cv::Point2f center_px(512.0f, 384.0f);  // 随便举例，中心点

    // 输出 yaw/pitch（度）
    double yaw_deg = 0, pitch_deg = 0;
    if (camgeo.pixelToYawPitchDeg(center_px, yaw_deg, pitch_deg)) {
        std::cout << "Pixel (" << center_px.x << "," << center_px.y << ") => yaw=" << yaw_deg
                  << " deg, pitch=" << pitch_deg << " deg\n";
    } else {
        std::cout << "Failed to compute yaw/pitch\n";
    }

    // ====== 若要靠装甲板宽度估距（可选） ======
    double pixel_w = 120.0;  // 假设目标在图像中的像素宽
    double real_w_m = 0.11;  // 真实宽度 0.11 m
    double Z = camgeo.estimateZFromPixelWidth(pixel_w, real_w_m);
    if (Z > 0) {
        cv::Point3d pt_cam;
        if (camgeo.pixelToPointCam_Z(center_px, Z, pt_cam)) {
            std::cout << "Estimated Z=" << Z << " m, Pt_cam=" << pt_cam << "\n";
        }
    }

    return 0;
}
