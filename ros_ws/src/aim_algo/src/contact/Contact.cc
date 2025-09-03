#include "contact/Contact.h"

#include <iostream>

#include "utils/logging.hpp"

Contact::Contact(const contact_config& cfg)
    : cfg_(cfg),
      comm_({cfg.serial_dev, cfg.baud, 0x3A /*header由CommPort/Protocol用于RX*/, 32, 20}) {}

Contact::~Contact() { stop(); }

bool Contact::start(bool spawn_thread) {
    if (running_.exchange(true)) return true;

    // 启动串口读线程（CommPort 内部处理断连/重连）
    // start the serial port thread
    if (!comm_.start(/*spawn_thread=*/true)) {
        running_ = false;
        std::cerr << "[AutoAim] serial open failed on " << cfg_.serial_dev << "\n";
        return false;
    }
    if (spawn_thread) {
        th_ = std::thread([this] { this->run(); });
    }
    return true;
}

void Contact::run() {
    using clock_t = std::chrono::steady_clock;
    const double period_s = 1.0 / (cfg_.rate_hz > 1.f ? cfg_.rate_hz : 1.f);
    auto period = std::chrono::duration<double>(period_s);

    auto next = clock_t::now();
    auto last = next;

    while (running_) {
        next += std::chrono::duration_cast<clock_t::duration>(period);
        auto now = clock_t::now();
        float dt = std::chrono::duration<float>(now - last).count();
        last = now;

        loop_step_(dt > 0.f ? dt : static_cast<float>(period_s));

        std::this_thread::sleep_until(next);
    }
}

void Contact::stop() {
    bool was = running_.exchange(false);
    if (was && th_.joinable()) th_.join();
    comm_.stop();
}

void Contact::update_target(const Target t) {
    std::lock_guard<std::mutex> lk(tgt_mu_);
    tgt_ = t;
}

void Contact::set_control_func(ControlFunc f) {
    std::lock_guard<std::mutex> lk(ctl_mu_);
    ctl_ = std::move(f);
}

bool Contact::latest_rx(ProjectileRx& out) const {
    std::lock_guard<std::mutex> lk(rx_mu_);
    if (!have_rx_.load(std::memory_order_acquire)) return false;
    out = rx_;
    return true;
}

void Contact::set_limits(float pitch_min_deg, float pitch_max_deg, bool wrap_yaw) {
    cfg_.pitch_min_deg = pitch_min_deg;
    cfg_.pitch_max_deg = pitch_max_deg;
    cfg_.wrap_yaw_deg = wrap_yaw;
}

void Contact::loop_step_(float dt) {
    ProjectileRx cur{};
    if (comm_.latest(cur)) {
        std::lock_guard<std::mutex> lk(rx_mu_);
        rx_ = cur;
        have_rx_.store(true, std::memory_order_release);
    }
    if (cfg_.require_rx_before_send && !have_rx_) return;

    // get the set target
    Target t;
    {
        std::lock_guard<std::mutex> lk(tgt_mu_);
        t = tgt_;
    }

    // calculate the target absolute angles (degree)
    // if there is no rx at initial, them use 0 as base
    float base_yaw_deg = have_rx_ ? cur.yaw : 0.f;
    float base_pitch_deg = have_rx_ ? cur.pitch : 0.f;

    // add the relative angle
    // NOTE: the output of autoaim should be in degree
    float yaw_target_deg = base_yaw_deg + t.yaw_deg;
    float pitch_target_deg = base_pitch_deg + t.pitch_deg;

    // the control strategy function (optional)
    float yaw_cmd_deg = yaw_target_deg;
    float pitch_cmd_deg = pitch_target_deg;
    // NOTE: well, callback strategy to hamdle the data, I prefer not to use, the unit in the system
    // should be unified if you use degree, then use it all the time, if you use radian, then use it
    // all the time.
    {
        std::lock_guard<std::mutex> lk(ctl_mu_);
        if (ctl_) {
            ctl_(have_rx_ ? cur : ProjectileRx{}, yaw_target_deg, pitch_target_deg, dt, yaw_cmd_deg,
                 pitch_cmd_deg);
        }
    }

    // limit and wrap
    if (cfg_.wrap_yaw_deg) yaw_cmd_deg = wrap_deg180(yaw_cmd_deg);
    pitch_cmd_deg = clamp(pitch_cmd_deg, cfg_.pitch_min_deg, cfg_.pitch_max_deg);

    // push to the lower machine
    ProjectileTx tx{};
    tx.header = cfg_.header;
    tx.found = (t.dist_m > 0.f) ? 1 : 0;
    tx.pitch = pitch_cmd_deg;  // 度
    tx.yaw = yaw_cmd_deg;      // 度
    tx.shoot_or_not = t.shoot ? 1 : 0;
    tx.done_fitting = 1;
    tx.is_updated = 1;
    // tx.done_fitting / tx.patrolling / tx.is_updated / ...

    (void)comm_.send(tx);
}
