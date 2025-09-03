#include "kalman/kalman.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef KALMAN_NO_TOML
#include <toml.hpp>
#endif

using Mat2 = Kalman::Mat2;
using Mat24 = Kalman::Mat24;
using Mat42 = Kalman::Mat42;
using Mat44 = Kalman::Mat44;
using Vec2 = Kalman::Vec2;
using Vec4 = Kalman::Vec4;

static inline float sqr(float v) { return v * v; }
static inline float deg2rad(float d) { return d * float(M_PI / 180.0); }
static inline float rad2deg(float r) { return r * float(180.0 / M_PI); }

float Kalman::wrap180(float a) {
    while (a > 180.f) a -= 360.f;
    while (a < -180.f) a += 360.f;
    return a;
}

Kalman::Kalman() { reset(); }

void Kalman::reset() {
    x_.setZero();
    P_.setZero();
    // 初值不确定性（保守一点）
    P_(0, 0) = P_(1, 1) = 36.f;   // deg^2
    P_(2, 2) = P_(3, 3) = 144.f;  // (deg/s)^2

    last_now_ms_ = 0;
    last_gimbal_pitch_ = 0.f;
    last_gimbal_yaw_ = 0.f;
    last_gimbal_wp_ = 0.f;
    last_gimbal_wy_ = 0.f;
    last_gimbal_ap_ = 0.f;
    last_gimbal_ay_ = 0.f;
}

void Kalman::setProcessSigmaA(float sp, float sy) {
    sigma_a_pitch_deg_s2_ = std::max(1e-6f, sp);
    sigma_a_yaw_deg_s2_ = std::max(1e-6f, sy);
}

void Kalman::setMeasurementR(float rp, float ry) {
    r_pitch_deg2_ = std::max(1e-6f, rp);
    r_yaw_deg2_ = std::max(1e-6f, ry);
}

bool Kalman::loadFromToml(const std::string& toml_path, const std::string& table) {
#ifdef KALMAN_NO_TOML
    (void)toml_path;
    (void)table;
    return false;
#else
    try {
        auto cfg = toml::parse_file(toml_path);
        const auto* t = cfg[table].as_table();
        if (!t) return false;

        if (auto* proc = (*t)["process"].as_table()) {
            setProcessSigmaA(
                float((*proc)["sigma_a_pitch_deg_s2"].value_or(double(sigma_a_pitch_deg_s2_))),
                float((*proc)["sigma_a_yaw_deg_s2"].value_or(double(sigma_a_yaw_deg_s2_))));
        }
        if (auto* meas = (*t)["measurement"].as_table()) {
            setMeasurementR(float((*meas)["r_pitch"].value_or(double(r_pitch_deg2_))),
                            float((*meas)["r_yaw"].value_or(double(r_yaw_deg2_))));
            chi2_gate_ = float((*meas)["chi2_gate"].value_or(double(chi2_gate_)));
        }
        if (auto* tim = (*t)["timing"].as_table()) {
            vision_latency_ms_ =
                float((*tim)["vision_latency_ms"].value_or(double(vision_latency_ms_)));
            max_meas_age_ms_ = float((*tim)["max_meas_age_ms"].value_or(double(max_meas_age_ms_)));
            future_slop_ms_ = float((*tim)["future_slop_ms"].value_or(double(future_slop_ms_)));
        }
        if (auto* rob = (*t)["robust"].as_table()) {
            jitter_var_gain_deg2_per_s2_ = float((*rob)["jitter_var_gain_deg2_per_s2"].value_or(
                double(jitter_var_gain_deg2_per_s2_)));
        }

        // 保护
        setVisionLatencyMs(vision_latency_ms_);
        setMaxMeasAgeMs(max_meas_age_ms_);
        setFutureSlopMs(future_slop_ms_);
        setJitterVarGain(jitter_var_gain_deg2_per_s2_);

        // 不改变 P_ 的设定（reset 时已置好）
        return true;
    } catch (const std::exception&) {
        return false;
    }
#endif
}

// —— 用云台加速度作为控制输入，推进到“现在” ——
void Kalman::predictWithGimbal_(float dt, float ap, float ay) {
    if (dt <= 0.f) return;

    Mat44 F = Mat44::Identity();
    F(0, 2) = dt;
    F(1, 3) = dt;

    // 控制输入：u = [-a_gimbal_pitch, -a_gimbal_yaw]
    Mat42 B = Mat42::Zero();
    B(0, 0) = 0.5f * dt * dt;
    B(2, 0) = dt;
    B(1, 1) = 0.5f * dt * dt;
    B(3, 1) = dt;

    Eigen::Matrix<float, 2, 1> u;
    u << -ap, -ay;

    const float dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt2 * dt2;
    const float sP2 = sqr(sigma_a_pitch_deg_s2_);
    const float sY2 = sqr(sigma_a_yaw_deg_s2_);

    Mat44 Q = Mat44::Zero();
    Q(0, 0) = 0.25f * dt4 * sP2;
    Q(0, 2) = 0.5f * dt3 * sP2;
    Q(2, 0) = Q(0, 2);
    Q(2, 2) = dt2 * sP2;
    Q(1, 1) = 0.25f * dt4 * sY2;
    Q(1, 3) = 0.5f * dt3 * sY2;
    Q(3, 1) = Q(1, 3);
    Q(3, 3) = dt2 * sY2;

    x_ = F * x_ + B * u;
    P_ = F * P_ * F.transpose() + Q;
}

void Kalman::predictForwardVirtual_(float dt, Vec4& x_out, Mat44& P_out, float ap, float ay) const {
    if (dt <= 0.f) {
        x_out = x_;
        P_out = P_;
        return;
    }

    Mat44 F = Mat44::Identity();
    F(0, 2) = dt;
    F(1, 3) = dt;

    Mat42 B = Mat42::Zero();
    B(0, 0) = 0.5f * dt * dt;
    B(2, 0) = dt;
    B(1, 1) = 0.5f * dt * dt;
    B(3, 1) = dt;

    Eigen::Matrix<float, 2, 1> u;
    u << -ap, -ay;

    const float dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt2 * dt2;
    const float sP2 = sqr(sigma_a_pitch_deg_s2_);
    const float sY2 = sqr(sigma_a_yaw_deg_s2_);

    Mat44 Q = Mat44::Zero();
    Q(0, 0) = 0.25f * dt4 * sP2;
    Q(0, 2) = 0.5f * dt3 * sP2;
    Q(2, 0) = Q(0, 2);
    Q(2, 2) = dt2 * sP2;
    Q(1, 1) = 0.25f * dt4 * sY2;
    Q(1, 3) = 0.5f * dt3 * sY2;
    Q(3, 1) = Q(1, 3);
    Q(3, 3) = dt2 * sY2;

    x_out = F * x_ + B * u;
    P_out = F * P_ * F.transpose() + Q;
}

// —— 在“现在”做一次观测更新（角度量测→相对量；陈旧量测膨胀 R 或丢弃） ——
void Kalman::updateNow_(const KalmanMsg& msg) {
    // fused measurement time
    std::int64_t fused_ms = msg.img_target_time_stamp + (std::int64_t)vision_latency_ms_;
    // 容忍少许“未来”，过多则夹回现在
    if (fused_ms > last_now_ms_ + (std::int64_t)future_slop_ms_) {
        fused_ms = last_now_ms_;
    }
    // 陈旧性评估
    float age_ms = float(last_now_ms_ - fused_ms);

    if (age_ms > max_meas_age_ms_) {
        // 太陈旧，直接丢弃
        return;
    }
    if (age_ms < 0.f) age_ms = 0.f;  // 负值视为 0

    // 把“绝对角”量测转成“相对角”（以现在的云台角为基准）
    float z_pitch_rel = wrap180(msg.y - msg.pitch_angle);
    float z_yaw_rel = wrap180(msg.x - msg.yaw_angle);

    // H（仅观测到相对角，不观测相对角速度）
    Mat24 H = Mat24::Zero();
    H(0, 0) = 1.f;
    H(1, 1) = 1.f;

    // R 膨胀：R_eff = R + age_s^2 * gain
    float age_s = age_ms * 1e-3f;
    Mat2 R = Mat2::Zero();
    R(0, 0) = r_pitch_deg2_ + sqr(age_s) * jitter_var_gain_deg2_per_s2_;
    R(1, 1) = r_yaw_deg2_ + sqr(age_s) * jitter_var_gain_deg2_per_s2_;

    Vec2 z;
    z << z_pitch_rel, z_yaw_rel;
    Vec2 y = z - H * x_;
    Mat2 S = H * P_ * H.transpose() + R;

    Eigen::LLT<Mat2> llt(S);
    if (llt.info() != Eigen::Success) {
        Mat2 S_inv = S.inverse();
        Mat42 K = P_ * H.transpose() * S_inv;
        x_ = x_ + K * y;
        Mat44 I = Mat44::Identity();
        P_ = (I - K * H) * P_;
        return;
    }

    if (chi2_gate_ > 0.f) {
        float nis = (y.transpose() * llt.solve(y))(0, 0);
        if (nis > chi2_gate_) {
            // 离群，丢弃本次更新
            return;
        }
    }

    Mat42 PHT = P_ * H.transpose();
    Mat24 X = llt.solve(PHT.transpose());
    Mat42 K = X.transpose();

    x_ = x_ + K * y;
    Mat44 I = Mat44::Identity();
    P_ = (I - K * H) * P_;
}

// —— 主循环入口 ——
// 每 tick 都会调用：先用云台输入前推到“现在”，若 has_measurement 再做一次更新
void Kalman::step(const KalmanMsg& msg, bool has_measurement) {
    const std::int64_t now_ms = msg.lower_machine_time_stamp;
    if (now_ms <= 0) return;

    if (last_now_ms_ == 0) {
        last_now_ms_ = now_ms;
        last_gimbal_pitch_ = msg.pitch_angle;
        last_gimbal_yaw_ = msg.yaw_angle;
        last_gimbal_wp_ = last_gimbal_wy_ = 0.f;
        last_gimbal_ap_ = last_gimbal_ay_ = 0.f;
        last_rx_seq_ = msg.rx_seq;  // ← 记住起始 seq
        return;
    }

    float dt = float(now_ms - last_now_ms_) * 1e-3f;
    if (dt <= 0.f) return;

    const bool has_new_rx = (msg.rx_seq != last_rx_seq_);

    float wp, wy, ap, ay;
    if (has_new_rx) {
        float d_pitch = angDiff(msg.pitch_angle, last_gimbal_pitch_);
        float d_yaw = angDiff(msg.yaw_angle, last_gimbal_yaw_);
        wp = d_pitch / dt;
        wy = d_yaw / dt;
        ap = (wp - last_gimbal_wp_) / dt;
        ay = (wy - last_gimbal_wy_) / dt;
    } else {
        // 没有新 RX：保守处理（两种任选一种或结合）
        wp = last_gimbal_wp_;
        wy = last_gimbal_wy_;
        ap = 0.f;
        ay = 0.f;  // 控制输入置 0，避免低通持续拉平
        // 或者在 predictWithGimbal_ 里通过缩放 sigma_a_* 来减小 Q
    }

    predictWithGimbal_(dt, ap, ay);
    last_now_ms_ = now_ms;

    if (has_new_rx) {
        last_gimbal_pitch_ = msg.pitch_angle;
        last_gimbal_yaw_ = msg.yaw_angle;
        last_gimbal_wp_ = wp;
        last_gimbal_wy_ = wy;
        const float a_alpha = 0.5f;
        last_gimbal_ap_ = (1.f - a_alpha) * last_gimbal_ap_ + a_alpha * ap;
        last_gimbal_ay_ = (1.f - a_alpha) * last_gimbal_ay_ + a_alpha * ay;
        last_rx_seq_ = msg.rx_seq;  // ← 只在有新 RX 时推进
    }
    // 若没有新 RX，也可以选择不动这些缓存，保持上一帧的“已知最好值”

    if (has_measurement) {
        updateNow_(msg);
    }
}

std::pair<float, float> Kalman::predictDeltaAt(const KalmanMsg& msg,
                                               std::int64_t horizon_ms) const {
    float dt = (horizon_ms <= 0 ? 0.f : float(horizon_ms) * 1e-3f);
    Vec4 xp;
    Mat44 Pp;
    // 使用最近估计的云台加速度做虚拟预测（无副作用）
    predictForwardVirtual_(dt, xp, Pp, last_gimbal_ap_, last_gimbal_ay_);
    // 相对角即为“需要再加多少”
    return {xp(0), xp(1)};
}

void Kalman::getState(float& pr, float& yr, float& wpr, float& wyr) const {
    pr = x_(0);
    yr = x_(1);
    wpr = x_(2);
    wyr = x_(3);
}

void Kalman::logParams_(std::shared_ptr<spdlog::logger> logger) const {
    if (!logger) return;
    logger->info(
        "\n"
        "░█▀▄░█▀▀░█░░░█▀█░█░█░░░█░█░█▀█░█░░░█▄█░█▀█░█▀█\n"
        "░█░█░█▀▀░█░░░█▀█░░█░░░░█▀▄░█▀█░█░░░█░█░█▀█░█░█\n"
        "░▀▀░░▀▀▀░▀▀▀░▀░▀░░▀░░░░▀░▀░▀░▀░▀▀▀░▀░▀░▀░▀░▀░▀\n");
    logger->info("============= kalman(rel) params =============");
    logger->info("[process] sigma_a_pitch={} deg/s^2, sigma_a_yaw={} deg/s^2",
                 sigma_a_pitch_deg_s2_, sigma_a_yaw_deg_s2_);
    logger->info("[measurement] R_pitch={} deg^2, R_yaw={} deg^2, chi2_gate={}", r_pitch_deg2_,
                 r_yaw_deg2_, chi2_gate_);
    logger->info("[timing] vision_latency_ms={}, max_meas_age_ms={}, future_slop_ms={}",
                 vision_latency_ms_, max_meas_age_ms_, future_slop_ms_);
    logger->info("[robust] jitter_var_gain_deg2_per_s2={}", jitter_var_gain_deg2_per_s2_);
    logger->info("==============================================");
}

bool Kalman::welcom(const std::string& toml_path, const std::string& table,
                    std::shared_ptr<spdlog::logger> logger) {
    if (!logger) {
        logger = spdlog::get("kalman");
        if (!logger) logger = spdlog::default_logger();
    }
    bool ok = loadFromToml(toml_path, table);
    logger->info("welcom to Kalman");
    logger->info("{} load config from: {} [{}]", (ok ? "[OK]" : "[ERR]"), toml_path, table);
    logParams_(logger);
    return ok;
}

void Kalman::welcom(std::shared_ptr<spdlog::logger> logger) const {
    if (!logger) {
        logger = spdlog::get("kalman");
        if (!logger) logger = spdlog::default_logger();
    }
    logger->info("welcom to Kalman");
    logParams_(logger);
}
