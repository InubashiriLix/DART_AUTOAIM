#pragma once
#include <Eigen/Dense>

// 4D 恒速角度 KF（度制）
// x = [ pitch, pitch_rate, yaw, yaw_rate ]^T
// z = [ pitch_meas, yaw_meas ]
class SimpleAngleKF {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Vec4 = Eigen::Matrix<float, 4, 1>;
    using Mat4 = Eigen::Matrix<float, 4, 4>;
    using Vec2 = Eigen::Matrix<float, 2, 1>;
    using Mat2 = Eigen::Matrix<float, 2, 2>;
    using Mat24 = Eigen::Matrix<float, 2, 4>;
    using Mat42 = Eigen::Matrix<float, 4, 2>;

    SimpleAngleKF() {
        x_.setZero();
        P_.setIdentity();
        // 默认噪声（起步可用，再按实际调）
        sigma_a_pitch_ = 2.2f;  // deg/s^2
        sigma_a_yaw_ = 1.4f;    // deg/s^2
        r_pitch_ = 0.30f;       // (≈0.55deg)^2
        r_yaw_ = 0.20f;         // (≈0.45deg)^2
    }

    void setState(const Vec4& x0, const Mat4& P0) {
        x_ = x0;
        P_ = P0;
    }
    const Vec4& state() const { return x_; }
    const Mat4& covariance() const { return P_; }

    void setAccelStd(float sigma_pitch, float sigma_yaw) {
        sigma_a_pitch_ = sigma_pitch;
        sigma_a_yaw_ = sigma_yaw;
    }
    void setMeasNoise(float r_pitch, float r_yaw) {
        r_pitch_ = r_pitch;
        r_yaw_ = r_yaw;
    }
    // 供 NIS 门限使用：取与内部一致的 R
    Mat2 measR() const {
        Mat2 R;
        R.setZero();
        R(0, 0) = r_pitch_;
        R(1, 1) = r_yaw_;
        return R;
    }

    // 预测 dt 秒
    void predict(float dt) {
        if (dt <= 0.f) return;
        Mat4 F = Mat4::Identity();
        F(0, 1) = dt;  // pitch += pitch_rate*dt
        F(2, 3) = dt;  // yaw   += yaw_rate*dt

        // Q（分轴恒加速度白噪声离散）
        const float dt2 = dt * dt, dt3 = dt2 * dt;
        const float sa2p = sigma_a_pitch_ * sigma_a_pitch_;
        const float sa2y = sigma_a_yaw_ * sigma_a_yaw_;
        Mat4 Q = Mat4::Zero();
        // pitch 块
        Q(0, 0) = (dt3 / 3.0f) * sa2p;
        Q(0, 1) = (dt2 / 2.0f) * sa2p;
        Q(1, 0) = Q(0, 1);
        Q(1, 1) = dt * sa2p;
        // yaw 块
        Q(2, 2) = (dt3 / 3.0f) * sa2y;
        Q(2, 3) = (dt2 / 2.0f) * sa2y;
        Q(3, 2) = Q(2, 3);
        Q(3, 3) = dt * sa2y;

        x_ = F * x_;
        P_ = F * P_ * F.transpose() + Q;
    }

    // 更新 z=[pitch_meas, yaw_meas]
    void update(const Vec2& z) {
        Mat24 H;
        H << 1, 0, 0, 0, 0, 0, 1, 0;
        Mat2 R = measR();
        Mat2 S = H * P_ * H.transpose() + R;
        Mat42 K = P_ * H.transpose() * S.inverse();
        Vec2 y = z - H * x_;
        x_ += K * y;
        Mat4 I = Mat4::Identity();
        P_ = (I - K * H) * P_;
    }

   private:
    Vec4 x_;
    Mat4 P_;
    float sigma_a_pitch_{2.2f};
    float sigma_a_yaw_{1.4f};
    float r_pitch_{0.30f};
    float r_yaw_{0.20f};
};
