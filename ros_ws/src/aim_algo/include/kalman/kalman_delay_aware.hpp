#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cstdint>
#include <iostream>
#include <string>
#include <utility>

#include "toml.hpp"
#include "utils/logging.hpp"

struct KalmanMsg {
    std::int64_t img_target_time_stamp{0};     // ms
    std::int64_t lower_machine_time_stamp{0};  // ms

    // image target (可选：若你在外部已映射成角度，直接用下面两个角度接口 updateAngles)
    float x{0.0f};
    float y{0.0f};

    // 当前云台姿态（度）
    float pitch_angle{0.0f};
    float yaw_angle{0.0f};
    float roll_angle{0.0f};
    float q[4]{0, 0, 0, 0};
};

class KalmanDelayAware {
   public:
    // 直接用 TOML 初始化（默认表名 [kalman]）
    explicit KalmanDelayAware(const std::string& toml_path, const std::string& table = "kalman") {
        loadFromToml(toml_path, table);
        reset();
    }

    bool welcom(const std::string& toml_path, const std::string& table = "kalman",
                std::shared_ptr<spdlog::logger> logger = nullptr);

    // 仅打印当前已生效的配置（不加载）
    void welcom(std::shared_ptr<spdlog::logger> logger) const;

    KalmanDelayAware() { reset(); }

    // 从 TOML 读取/更新参数
    bool loadFromToml(const std::string& toml_path, const std::string& table = "kalman");

    // 重置滤波器
    void reset();

    // 用“目标角度观测 + 时间戳（ms）”进行一次滤波（推荐）
    void updateAngles(float meas_pitch_deg, float meas_yaw_deg, std::int64_t meas_ts_ms);

    // 用原始结构体（会用 msg.pitch_angle / msg.yaw_angle 做量测，时间戳用 img_target_time_stamp）
    // 如果你在外部把 x,y 映射成角度，也可以改这里的选择逻辑。
    void update(const KalmanMsg& msg) {
        updateAngles(msg.pitch_angle, msg.yaw_angle, msg.img_target_time_stamp);
    }

    // 仅预测到指定时间（ms）
    void predictTo(std::int64_t ts_ms);

    // 返回 PD + 预瞄输出的“增量指令（度）”
    // 这里直接用类内 TOML 的 Kp/Kv/预瞄与死区/限幅
    std::pair<float, float> getDeltaAnglesPD(float gimbal_pitch_deg, float gimbal_yaw_deg) const;

    // 只读：当前估计（度 & 度/秒）
    void getEstimate(float& pitch_deg, float& yaw_deg, float& pitch_rate_dps,
                     float& yaw_rate_dps) const {
        pitch_deg = x_(0);
        yaw_deg = x_(1);
        pitch_rate_dps = x_(2);
        yaw_rate_dps = x_(3);
    }

    // 动态改参数（可选）
    void setDefaultPreviewMs(float ms) { preview_ms_ = std::max(0.0f, ms); }
    void setGains(float kp, float kv) {
        kp_ = kp;
        kv_ = kv;
    }

   private:
    // 状态: [pitch, yaw, pitch_rate, yaw_rate]
    Eigen::Matrix<float, 4, 1> x_;
    Eigen::Matrix<float, 4, 4> P_;

    // 过程/量测噪声（按白噪声加速度模型离散化）
    float sigma_a_pitch_{1.6f};  // (deg/s^2) 1σ
    float sigma_a_yaw_{1.0f};    // (deg/s^2) 1σ
    float r_pitch_{0.25f};       // (deg^2)  量测方差
    float r_yaw_{0.25f};         // (deg^2)

    // 初值
    float p0_angle_{1.0f};
    float p0_rate_{4.0f};

    // 输出整形（PD 与预瞄、死区、限幅）
    float kp_{1.0f};
    float kv_{0.05f};
    float preview_ms_{50.0f};
    float deadband_deg_{0.03f};
    float max_step_deg_{0.8f};

    // 门限（可选，用于检查创新是否过大；默认关闭=负值）
    float chi2_gate_{-1.0f};

    // 视觉延迟补偿（>0 表示“观测发生在当前时间之前的这么多毫秒”）
    float vision_latency_ms_{13.0f};

    // 记录
    mutable float last_delta_pitch_{0.0f}, last_delta_yaw_{0.0f};
    std::int64_t last_ts_ms_{0};  // 上一次状态时间（ms）

    // 工具
    static inline float clampf(float v, float lo, float hi) {
        return std::max(lo, std::min(hi, v));
    }
    static inline float deadband(float v, float dz) { return (std::abs(v) < dz) ? 0.0f : v; }
};
