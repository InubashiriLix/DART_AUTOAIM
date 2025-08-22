#pragma once

#include <Eigen/Dense>
#include <cmath>

#ifndef EKF_USE_JOSEPH_FORM
#define EKF_USE_JOSEPH_FORM 1  // 1=数值更稳；0=更快
#endif

struct EKF9 {
    static constexpr int NX = 9;
    static constexpr int NZ = 4;

    using VecN = Eigen::Matrix<double, NX, 1>;
    using MatN = Eigen::Matrix<double, NX, NX>;
    using MatNNZ = Eigen::Matrix<double, NX, NZ>;
    using MatZN = Eigen::Matrix<double, NZ, NX>;
    using MatZ = Eigen::Matrix<double, NZ, NZ>;
    using VecZ = Eigen::Matrix<double, NZ, 1>;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // --- public state ---
    double dt{0.01};  // 外部可按需更新
    VecN x = VecN::Zero();
    MatN P = MatN::Identity();
    MatN Q = MatN::Zero();  // 过程噪声
    MatZ R = MatZ::Zero();  // 量测噪声

    EKF9();

    // 设置/读取
    inline void setState(const VecN& x0) { x = x0; }
    inline const VecN& state() const { return x; }
    inline const MatN& covariance() const { return P; }
    inline void setDt(double new_dt) { dt = new_dt; }

    // 仅设置对角
    void setProcessNoiseDiag(const VecN& qdiag);
    void setMeasurementNoiseDiag(const VecZ& rdiag);

    // 预测 / 更新
    const VecN& predict();            // 使用成员 dt
    const VecN& predict(double dt_);  // 同时更新 dt 并预测
    const VecN& update(const VecZ& z);

   private:
    // h(x) & H(x)
    inline VecZ h_of_x(const VecN& s) const;
    inline MatZ N4Identity() const { return MatZ::Identity(); }

    // 临时缓存（避免重复分配）
    MatN F_;  // 雅可比
    MatZN H_;
    MatZ S_;
    MatNNZ PHt_;  // P*H^T
    MatNNZ K_;    // 卡尔曼增益
};
