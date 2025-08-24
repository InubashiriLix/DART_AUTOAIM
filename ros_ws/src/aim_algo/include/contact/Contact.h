#pragma once
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "config_parser.hpp"
#include "contact/CommPort.h"
#include "contact/Protocol.h"

class Contact {
   public:
    struct Target {
        float pitch_deg = 0.f;  // 相对 pitch (rad)
        float yaw_deg = 0.f;    // 相对 yaw   (rad)
        float dist_m = 0.f;     // >0 表示有效
        uint8_t shoot = 0;      // 0/1
    };

    // 控制回调：把“目标绝对角(度)”变成“最终下发绝对角(度)”
    // 输入: rx(当前姿态/角度, 度), yaw_target_deg, pitch_target_deg, dt(秒)
    // 输出: out_yaw_cmd_deg, out_pitch_cmd_deg（最终要下发的度数命令）
    using ControlFunc =
        std::function<void(const ProjectileRx& rx, float yaw_target_deg, float pitch_target_deg,
                           float dt, float& out_yaw_cmd_deg, float& out_pitch_cmd_deg)>;

    explicit Contact(const contact_config& cfg);
    ~Contact();

    /**
     * @brief open the comm port and start the control thread, pls use run() to block if
     * spawn_thread is false spawn_thread is false
     *
     * @param spawn_thread whether to spawn a thread to run the main loop, if false, pls call run()
     * to block
     * @return success
     */
    bool start(bool spawn_thread = true);
    /**
     * @brief run the main loop (blocking until stop() is called)
     */
    void run();
    /**
     * @brief stop the thread and close the serial port
     */
    void stop();

    /**
     * @brief update the relative target {pitch_rad, yaw_rad, dist_m, shoot}
     *
     * @param t the Target&
     */
    void update_target(const Target& t);

    /**
     * @brief set the control callback function, if not set, then only wrap and clamp the target
     *
     * @param f callback function
     */
    void set_control_func(ControlFunc f);

    /**
     * @brief pls use this to get the newest rx data
     *
     * @param out ProjectileRx&
     * @return whether there is a newest data
     */
    bool latest_rx(ProjectileRx& out) const;

    /**
     * @brief set the limits of pitch and yaw degree dynamically
     *
     * @param pitch_min_deg float
     * @param pitch_max_deg float
     * @param wrap_yaw whether to warp yaw to [-180,180)
     */
    void set_limits(float pitch_min_deg, float pitch_max_deg, bool wrap_yaw);

   private:
    static inline float rad2deg(float r) { return r * 57.2957795f; }
    static inline float clamp(float x, float lo, float hi) {
        return x < lo ? lo : (x > hi ? hi : x);
    }
    static inline float wrap_deg180(float a) {
        while (a >= 180.f) a -= 360.f;
        while (a < -180.f) a += 360.f;
        return a;
    }

    void loop_step_(float dt);  // single step

    contact_config cfg_;
    CommPort comm_;

    // the state from lower machine (rx)
    mutable std::mutex rx_mu_;
    ProjectileRx rx_{};
    std::atomic<bool> have_rx_{false};

    // the target
    mutable std::mutex tgt_mu_;
    Target tgt_{};

    // the control callback function
    mutable std::mutex ctl_mu_;
    ControlFunc ctl_;

    // thread
    std::thread th_;
    std::atomic<bool> running_{false};
};
