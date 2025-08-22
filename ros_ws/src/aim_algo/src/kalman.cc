#include "kalman.hpp"

EKF9::EKF9() {
    // 默认噪声：位置/角小，速度/角速中，半径很小（按需改）
    Q.setZero();
    Q.diagonal() << 1e-4, 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2, 1e-3, 1e-5;
    R.setZero();
    R.diagonal() << 5e-4, 5e-4, 5e-4, 5e-3;
}

void EKF9::setProcessNoiseDiag(const VecN& qdiag) {
    Q.setZero();
    Q.diagonal() = qdiag;
}
void EKF9::setMeasurementNoiseDiag(const VecZ& rdiag) {
    R.setZero();
    R.diagonal() = rdiag;
}

const EKF9::VecN& EKF9::predict(double dt_) {
    dt = dt_;
    return predict();
}

const EKF9::VecN& EKF9::predict() {
    // x = f(x)
    const double vxc = x(4), vyc = x(5), vzc = x(6), omg = x(7);
    x(0) += vxc * dt;
    x(1) += vyc * dt;
    x(2) += vzc * dt;
    x(3) += omg * dt;
    // vxc, vyc, vzc, omg, r 不变

    // F = ∂f/∂x
    F_.setIdentity();
    F_(0, 4) = dt;
    F_(1, 5) = dt;
    F_(2, 6) = dt;
    F_(3, 7) = dt;

    // P = F P Fᵀ + Q
    // 使用 noalias 避免临时
    P = F_ * P * F_.transpose();
    P.noalias() += Q;
    return x;
}

// h(x) = [xa, ya, za, ψ]ᵀ
inline EKF9::VecZ EKF9::h_of_x(const VecN& s) const {
    const double xc = s(0), yc = s(1), zc = s(2), psi = s(3), r = s(8);
    const double c = std::cos(psi), sn = std::sin(psi);
    VecZ z;
    z << xc - r * c, yc - r * sn, zc, psi;
    return z;
}

const EKF9::VecN& EKF9::update(const VecZ& z) {
    // H = ∂h/∂x
    const double psi = x(3), r = x(8);
    const double c = std::cos(psi), sn = std::sin(psi);
    H_.setZero();
    // xa row
    H_(0, 0) = 1.0;
    H_(0, 3) = r * sn;
    H_(0, 8) = -c;
    // ya row
    H_(1, 1) = 1.0;
    H_(1, 3) = -r * c;
    H_(1, 8) = -sn;
    // za row
    H_(2, 2) = 1.0;
    // yaw row
    H_(3, 3) = 1.0;

    // 创新 y = z - h(x)
    const VecZ zhat = h_of_x(x);
    const VecZ y = z - zhat;

    // 计算 K = P Hᵀ (H P Hᵀ + R)^-1
    // 先 PHt，再 S = H*PHt + R
    PHt_.noalias() = P * H_.transpose();  // (9x4)
    S_.noalias() = H_ * PHt_;             // (4x4)
    S_.noalias() += R;

    // LDLT 求解，不显式求逆：
    // Solve X in S * X = PHtᵀ  =>  X = S^{-1} * PHtᵀ
    // Then K = Xᵀ
    Eigen::LDLT<MatZ> ldlt(S_);
    const Eigen::Matrix<double, NZ, NX> X = ldlt.solve(PHt_.transpose());
    K_.noalias() = X.transpose();  // (9x4)

    // x = x + K y
    x.noalias() += K_ * y;

#if EKF_USE_JOSEPH_FORM
    // Joseph 形式：P = (I-KH)P(I-KH)ᵀ + K R Kᵀ  —— 稳定但略慢
    static const MatN I = MatN::Identity();
    MatN KH;
    KH.noalias() = K_ * H_;
    MatN A;
    A.noalias() = I - KH;
    MatN KRKt;
    KRKt.noalias() = K_ * R * K_.transpose();
    P = A * P * A.transpose();
    P.noalias() += KRKt;
#else
    // 经典形式：P = (I-KH)P  —— 更快，但可能略欠稳
    static const MatN I = MatN::Identity();
    MatN KH;
    KH.noalias() = K_ * H_;
    P.noalias() = (I - KH) * P;
#endif
    return x;
}
