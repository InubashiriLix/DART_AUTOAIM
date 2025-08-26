#pragma once
/**
 * @file kalman_rel.hpp
 * @brief Delay-aware relative Kalman filter for pitch/yaw tracking using gimbal IMU/encoder as
 * control input.
 *
 * 设计目标：
 *  - 仅支持角度量测（ANGLE_DEG）：KalmanMsg.x=绝对 yaw(°)，KalmanMsg.y=绝对 pitch(°)
 *  - 不涉及像素/相机内参/roll 去旋；不做 deadband / clamp（这些由上层 contact 处理）
 *  - 将下位机云台姿态流作为“已知输入”（角加速度）驱动预测，即使没有视觉量测也持续前推
 *  - 延迟鲁棒：视觉帧时延抖动（10–100 ms）时，自动对陈旧量测膨胀 R 或直接丢弃
 *  - 动态预瞄：predictDeltaAt(..., horizon_ms) 由调用方传入，类内不固定任何预瞄属性
 */

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cstdint>
#include <memory>

namespace spdlog {
class logger;
}

struct KalmanMsg {
    std::int64_t img_target_time_stamp{0};     // ms (视觉目标图像时刻)
    std::int64_t lower_machine_time_stamp{0};  // ms (当前“现在”的时刻，同一时钟域)

    // 视觉量测（角度模式，仅使用 x/y，为绝对角）
    float x{0.0f};  // absolute yaw (deg)
    float y{0.0f};  // absolute pitch (deg)

    // 当前云台姿态（度）
    float pitch_angle{0.0f};
    float yaw_angle{0.0f};
    float roll_angle{0.0f};  // 未使用，但保留

    // 可选四元数（未直接使用，保留）
    float q[4]{0, 0, 0, 0};

    uint64_t rx_seq = 0;
};

class Kalman {
   public:
    Kalman();

    // ——— 配置 / 运行期调参 ———
    bool loadFromToml(const std::string& toml_path, const std::string& table = "kalman");
    void reset();

    // 视觉链路总延迟（ms），可运行期动态设定
    void setVisionLatencyMs(float ms) { vision_latency_ms_ = (ms < 0.f ? 0.f : ms); }
    float getVisionLatencyMs() const { return vision_latency_ms_; }

    // 时戳鲁棒：最大可接受陈旧量测年龄（ms），超过直接丢弃
    void setMaxMeasAgeMs(float ms) { max_meas_age_ms_ = (ms < 0.f ? 0.f : ms); }

    // 时钟未来抖动容忍（ms）：fused_ts 可略超“现在”这么多，超出则夹回“现在”
    void setFutureSlopMs(float ms) { future_slop_ms_ = (ms < 0.f ? 0.f : ms); }

    // 陈旧量测的方差膨胀系数（deg^2 / s^2），R_eff = R + age_s^2 * jitter_var_gain_deg2_per_s2_
    void setJitterVarGain(float v) { jitter_var_gain_deg2_per_s2_ = (v < 0.f ? 0.f : v); }

    // 过程/量测噪声等
    void setProcessSigmaA(float sigma_pitch_deg_s2, float sigma_yaw_deg_s2);
    void setMeasurementR(float r_pitch_deg2, float r_yaw_deg2);
    void setChi2Gate(float chi2_gate) { chi2_gate_ = (chi2_gate < 0.f ? 0.f : chi2_gate); }

    // ——— 主循环接口 ———
    // 每个控制 tick 都调用；has_measurement 表示本 tick 是否带来了新的视觉量测（x/y + img_ts）
    void step(const KalmanMsg& msg, bool has_measurement);

    // 在“现在=msg.lower_machine_time_stamp”为起点，预测 horizon_ms 后相对角（增量）
    // 返回 {Δpitch, Δyaw} (deg)。不改变内部状态。
    std::pair<float, float> predictDeltaAt(const KalmanMsg& msg, std::int64_t horizon_ms) const;

    // 获取当前相对状态（deg / deg/s）
    void getState(float& pitch_rel, float& yaw_rel, float& wp_rel, float& wy_rel) const;

    // 打印参数
    bool welcom(const std::string& toml_path, const std::string& table,
                std::shared_ptr<spdlog::logger> logger = nullptr);
    void welcom(std::shared_ptr<spdlog::logger> logger = nullptr) const;

   public:
    using Mat2 = Eigen::Matrix<float, 2, 2>;
    using Mat24 = Eigen::Matrix<float, 2, 4>;
    using Mat42 = Eigen::Matrix<float, 4, 2>;
    using Mat44 = Eigen::Matrix<float, 4, 4>;
    using Vec2 = Eigen::Matrix<float, 2, 1>;
    using Vec4 = Eigen::Matrix<float, 4, 1>;

   private:
    // 工具
    static float wrap180(float a_deg);
    static float angDiff(float a, float b) { return wrap180(a - b); }

    // 在“现在”做一次观测更新（量测是绝对角 -> 现时相对角；对陈旧量测做 R 膨胀/丢弃）
    void updateNow_(const KalmanMsg& msg);

    // 用云台加速度作为控制输入，前推到“现在”（单步）
    void predictWithGimbal_(float dt, float ap_deg_s2, float ay_deg_s2);

    // 无副作用向前预测 horizon（使用最近一次的 gimbal 加速度）
    void predictForwardVirtual_(float dt, Vec4& x_out, Mat44& P_out, float ap_deg_s2,
                                float ay_deg_s2) const;

    // 日志
    void logParams_(std::shared_ptr<spdlog::logger> logger) const;

   private:
    // —— 参数 ——（最小集）
    float sigma_a_pitch_deg_s2_ = 60.f;  // 相对角的加速度噪声标准差（deg/s^2）
    float sigma_a_yaw_deg_s2_ = 80.f;
    float r_pitch_deg2_ = 0.20f;  // 量测噪声方差（deg^2）
    float r_yaw_deg2_ = 0.20f;
    float chi2_gate_ = 9.21f;         // 2 自由度 χ² 门限；<=0 关闭
    float vision_latency_ms_ = 25.f;  // 默认延迟（可动态改）

    // 延迟鲁棒相关
    float max_meas_age_ms_ = 120.f;             // 超过则丢弃
    float future_slop_ms_ = 5.f;                // 允许的“未来”抖动，>则夹回现在
    float jitter_var_gain_deg2_per_s2_ = 8.0f;  // 陈旧量测的额外方差 ~ age_s^2 * gain

    // —— 状态：相对云台 ——  x = [θr_p, θr_y, ωr_p, ωr_y]^T  （deg / deg/s）
    Vec4 x_{Vec4::Zero()};
    Mat44 P_{Mat44::Zero()};

    // —— 时间基准 / 云台输入缓存 ——
    std::int64_t last_now_ms_{0};  // 上一次 step 的“现在”时刻
    float last_gimbal_pitch_{0.f};
    float last_gimbal_yaw_{0.f};
    float last_gimbal_wp_{0.f};  // 上一次估计的云台角速度 (deg/s)
    float last_gimbal_wy_{0.f};
    float last_gimbal_ap_{0.f};  // 上一次估计的云台角加速度 (deg/s^2) —— 供虚拟预测使用
    float last_gimbal_ay_{0.f};

    uint64_t last_rx_seq_ = 0;
};
