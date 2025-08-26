#include <cmath>
#include <iomanip>
#include <iostream>

#include "kalman/kalman_delay_aware.hpp"

bool KalmanDelayAware::loadFromToml(const std::string& toml_path, const std::string& table) {
    try {
        auto cfg = toml::parse_file(toml_path);

        // 支持 [kalman] 或者你传的 table 名
        const auto* tkal = cfg[table].as_table();
        if (!tkal) {
            std::cerr << "[Kalman] Missing table [" << table << "] in " << toml_path << "\n";
            return false;
        }

        // ---- output block [kalman.output] ----
        if (auto* out = (*tkal)["output"].as_table()) {
            kp_ = float((*out)["kp"].value_or(double(kp_)));
            kv_ = float((*out)["kv"].value_or(double(kv_)));
            preview_ms_ = float((*out)["preview_ms"].value_or(double(preview_ms_)));
            deadband_deg_ = float((*out)["deadband_deg"].value_or(double(deadband_deg_)));
            max_step_deg_ = float((*out)["max_step_deg"].value_or(double(max_step_deg_)));
        } else {
            // 也允许直接平铺
            kp_ = float((*tkal)["kp"].value_or(double(kp_)));
            kv_ = float((*tkal)["kv"].value_or(double(kv_)));
            preview_ms_ = float((*tkal)["preview_ms"].value_or(double(preview_ms_)));
            deadband_deg_ = float((*tkal)["deadband_deg"].value_or(double(deadband_deg_)));
            max_step_deg_ = float((*tkal)["max_step_deg"].value_or(double(max_step_deg_)));
        }

        // ---- process block [kalman.process] ----
        if (auto* proc = (*tkal)["process"].as_table()) {
            sigma_a_pitch_ = float((*proc)["sigma_a_pitch"].value_or(double(sigma_a_pitch_)));
            sigma_a_yaw_ = float((*proc)["sigma_a_yaw"].value_or(double(sigma_a_yaw_)));
            p0_angle_ = float((*proc)["p0_angle"].value_or(double(p0_angle_)));
            p0_rate_ = float((*proc)["p0_rate"].value_or(double(p0_rate_)));
        } else {
            sigma_a_pitch_ = float((*tkal)["sigma_a_pitch"].value_or(double(sigma_a_pitch_)));
            sigma_a_yaw_ = float((*tkal)["sigma_a_yaw"].value_or(double(sigma_a_yaw_)));
            p0_angle_ = float((*tkal)["p0_angle"].value_or(double(p0_angle_)));
            p0_rate_ = float((*tkal)["p0_rate"].value_or(double(p0_rate_)));
        }

        // ---- measurement block [kalman.measurement] ----
        if (auto* meas = (*tkal)["measurement"].as_table()) {
            r_pitch_ = float((*meas)["r_pitch"].value_or(double(r_pitch_)));
            r_yaw_ = float((*meas)["r_yaw"].value_or(double(r_yaw_)));
            chi2_gate_ = float((*meas)["chi2_gate"].value_or(double(chi2_gate_)));
        } else {
            r_pitch_ = float((*tkal)["r_pitch"].value_or(double(r_pitch_)));
            r_yaw_ = float((*tkal)["r_yaw"].value_or(double(r_yaw_)));
            chi2_gate_ = float((*tkal)["chi2_gate"].value_or(double(chi2_gate_)));
        }

        // ---- timing block [kalman.timing] ----
        if (auto* tim = (*tkal)["timing"].as_table()) {
            vision_latency_ms_ =
                float((*tim)["vision_latency_ms"].value_or(double(vision_latency_ms_)));
        } else {
            vision_latency_ms_ =
                float((*tkal)["vision_latency_ms"].value_or(double(vision_latency_ms_)));
        }

        // 合法性
        preview_ms_ = std::max(0.0f, preview_ms_);
        deadband_deg_ = std::max(0.0f, deadband_deg_);
        max_step_deg_ = std::max(0.0f, max_step_deg_);
        r_pitch_ = std::max(1e-6f, r_pitch_);
        r_yaw_ = std::max(1e-6f, r_yaw_);
        sigma_a_pitch_ = std::max(1e-6f, sigma_a_pitch_);
        sigma_a_yaw_ = std::max(1e-6f, sigma_a_yaw_);

        // 重新设定初值协方差
        P_.setZero();
        P_(0, 0) = P_(1, 1) = p0_angle_;
        P_(2, 2) = P_(3, 3) = p0_rate_;

        return true;
    } catch (const std::exception& e) {
        std::cerr << "[Kalman] loadFromToml error: " << e.what() << "\n";
        return false;
    }
}

void KalmanDelayAware::reset() {
    x_.setZero();
    P_.setZero();
    P_(0, 0) = P_(1, 1) = p0_angle_;
    P_(2, 2) = P_(3, 3) = p0_rate_;
    last_ts_ms_ = 0;
    last_delta_pitch_ = last_delta_yaw_ = 0.0f;
}

void KalmanDelayAware::predictTo(std::int64_t ts_ms) {
    if (last_ts_ms_ == 0) {
        last_ts_ms_ = ts_ms;
        return;
    }
    float dt = float(ts_ms - last_ts_ms_) * 1e-3f;
    if (dt <= 0.0f) return;
    last_ts_ms_ = ts_ms;

    // x_k+1 = F x_k
    Eigen::Matrix<float, 4, 4> F = Eigen::Matrix<float, 4, 4>::Identity();
    F(0, 2) = dt;
    F(1, 3) = dt;

    // Q 离散化（白噪声加速度到速度/角度）
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt2 * dt2;
    const float sP = sigma_a_pitch_ * sigma_a_pitch_;
    const float sY = sigma_a_yaw_ * sigma_a_yaw_;

    Eigen::Matrix<float, 4, 4> Q = Eigen::Matrix<float, 4, 4>::Zero();
    // pitch 2x2
    Q(0, 0) = 0.25f * dt4 * sP;
    Q(0, 2) = 0.5f * dt3 * sP;
    Q(2, 0) = Q(0, 2);
    Q(2, 2) = dt * sP;
    // yaw 2x2
    Q(1, 1) = 0.25f * dt4 * sY;
    Q(1, 3) = 0.5f * dt3 * sY;
    Q(3, 1) = Q(1, 3);
    Q(3, 3) = dt * sY;

    x_ = F * x_;
    P_ = F * P_ * F.transpose() + Q;
}

void KalmanDelayAware::updateAngles(float meas_pitch_deg, float meas_yaw_deg,
                                    std::int64_t meas_ts_ms) {
    // 视觉延迟：观测对应真实时刻 = meas_ts_ms + latency
    const std::int64_t fused_meas_ts = meas_ts_ms + static_cast<std::int64_t>(vision_latency_ms_);
    predictTo(fused_meas_ts);

    // 量测：z=[pitch, yaw]
    Eigen::Matrix<float, 2, 1> z;
    z << meas_pitch_deg, meas_yaw_deg;

    // H（2x4）
    Eigen::Matrix<float, 2, 4> H = Eigen::Matrix<float, 2, 4>::Zero();
    H(0, 0) = 1.0f;  // pitch
    H(1, 1) = 1.0f;  // yaw

    // R
    Eigen::Matrix<float, 2, 2> R = Eigen::Matrix<float, 2, 2>::Zero();
    R(0, 0) = r_pitch_;
    R(1, 1) = r_yaw_;

    // 创新 y、协方差 S
    Eigen::Matrix<float, 2, 1> y = z - H * x_;
    Eigen::Matrix<float, 2, 2> S = H * P_ * H.transpose() + R;

    // 用 LLT（更稳）来做求解
    Eigen::LLT<Eigen::Matrix<float, 2, 2>> llt(S);
    if (llt.info() != Eigen::Success) {
        // 极少数数值问题，退化为显式逆（仍然保护）
        // 注意：这里只是 fallback，正常不会走到
        Eigen::Matrix<float, 2, 2> S_inv = S.inverse();
        Eigen::Matrix<float, 4, 2> K = P_ * H.transpose() * S_inv;
        x_ = x_ + K * y;
        Eigen::Matrix<float, 4, 4> I = Eigen::Matrix<float, 4, 4>::Identity();
        P_ = (I - K * H) * P_;
        return;
    }

    // 卡方门限（可选）
    if (chi2_gate_ > 0.0f) {
        float nis = (y.transpose() * llt.solve(y))(0, 0);
        if (nis > chi2_gate_) {
            // 丢弃本次更新，只保留预测
            return;
        }
    }

    // K = P H^T S^{-1}  -> 先算 PHT，再用 LLT 解线性方程替代求逆
    Eigen::Matrix<float, 4, 2> PHT = P_ * H.transpose();
    // 解 S * X = PHT^T，得到 X = S^{-1} * PHT^T
    Eigen::Matrix<float, 2, 4> X = llt.solve(PHT.transpose());
    Eigen::Matrix<float, 4, 2> K = X.transpose();

    // 更新
    x_ = x_ + K * y;
    Eigen::Matrix<float, 4, 4> I = Eigen::Matrix<float, 4, 4>::Identity();
    P_ = (I - K * H) * P_;
}

std::pair<float, float> KalmanDelayAware::getDeltaAnglesPD(float gimbal_pitch_deg,
                                                           float gimbal_yaw_deg) const {
    const float preview_s = preview_ms_ * 1e-3f;

    const float pitch_preview = x_(0) + x_(2) * preview_s;
    const float yaw_preview = x_(1) + x_(3) * preview_s;

    float dp = kp_ * (pitch_preview - gimbal_pitch_deg) + kv_ * x_(2);
    float dy = kp_ * (yaw_preview - gimbal_yaw_deg) + kv_ * x_(3);

    dp = deadband(dp, deadband_deg_);
    dy = deadband(dy, deadband_deg_);

    dp = clampf(dp, -max_step_deg_, max_step_deg_);
    dy = clampf(dy, -max_step_deg_, max_step_deg_);

    last_delta_pitch_ = dp;
    last_delta_yaw_ = dy;
    return {dp, dy};
}

bool KalmanDelayAware::welcom(const std::string& toml_path, const std::string& table,
                              std::shared_ptr<spdlog::logger> logger) {
    if (!logger) {
        logger = perflog::get("kalman");
    }

    bool ok = loadFromToml(toml_path, table);

    logger->info("welcom to KalmanDelayAware");
    logger->info(
        "\n"
        "░█▀▄░█▀▀░█░░░█▀█░█░█░░░█░█░█▀█░█░░░█▄█░█▀█░█▀█\n"
        "░█░█░█▀▀░█░░░█▀█░░█░░░░█▀▄░█▀█░█░░░█░█░█▀█░█░█\n"
        "░▀▀░░▀▀▀░▀▀▀░▀░▀░░▀░░░░▀░▀░▀░▀░▀▀▀░▀░▀░▀░▀░▀░▀\n");
    logger->info("{} load config from: {} [{}]", (ok ? "[OK] " : "[ERR] "), toml_path, table);
    logger->info("============= kalman params =============");
    // logger->info("val={:.3f}", val);
    // logger->info("vec=({}, {}, {})", x, y, z);
    logger->info("[output]  kp={}, kv={}, preview_ms={}, deadband_deg={}, max_step_deg={}", kp_,
                 kv_, preview_ms_, deadband_deg_, max_step_deg_);
    logger->info("[process]  sigma_a_pitch={}, sigma_a_yaw={}, p0_angle={}, p0_rate={}",
                 sigma_a_pitch_, sigma_a_yaw_, p0_angle_, p0_rate_);
    logger->info("[measurement]  r_pitch={}, r_yaw={}, chi2_gate={}", r_pitch_, r_yaw_, chi2_gate_);
    logger->info("[timing]  vision_latency_ms={}", vision_latency_ms_);
    logger->info("=========================================");
    return ok;
}

void KalmanDelayAware::welcom(std::shared_ptr<spdlog::logger> logger) const {
    if (!logger) {
        logger = perflog::get("kalman");
    }

    logger->info("welcom to KalmanDelayAware");
    logger->info(
        "\n"
        "░█▀▄░█▀▀░█░░░█▀█░█░█░░░█░█░█▀█░█░░░█▄█░█▀█░█▀█\n"
        "░█░█░█▀▀░█░░░█▀█░░█░░░░█▀▄░█▀█░█░░░█░█░█▀█░█░█\n"
        "░▀▀░░▀▀▀░▀▀▀░▀░▀░░▀░░░░▀░▀░▀░▀░▀▀▀░▀░▀░▀░▀░▀░▀\n");
    logger->info("============= kalman params =============");
    logger->info("[output]  kp={}, kv={}, preview_ms={}, deadband_deg={}, max_step_deg={}", kp_,
                 kv_, preview_ms_, deadband_deg_, max_step_deg_);
    logger->info("[process]  sigma_a_pitch={}, sigma_a_yaw={}, p0_angle={}, p0_rate={}",
                 sigma_a_pitch_, sigma_a_yaw_, p0_angle_, p0_rate_);
    logger->info("[measurement]  r_pitch={}, r_yaw={}, chi2_gate={}", r_pitch_, r_yaw_, chi2_gate_);
    logger->info("[timing]  vision_latency_ms={}", vision_latency_ms_);
    logger->info("=========================================");
}
