#pragma once

#include <serial/serial.h>  // 你的 serial 库

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "contact/Protocol.h"  // 定义 ProjectileRx / ProjectileTx（32B 帧）
                               // 并包含包头字节 0x3A（若没有，就按下面默认值走）

// 可选：如果用不到 spdlog，去掉这两行，并把 .cc 里的日志改成 std::cerr
#include <spdlog/spdlog.h>

#include <memory>

class CommPort {
   public:
    struct Config {
        std::string preferred_dev = "/dev/ttyACM0";  // 首选设备
        uint32_t baud = 115200;
        uint8_t header = 0x3A;     // 帧头
        size_t packet_size = 32;   // 固定帧长（含 CRC）
        uint32_t timeout_ms = 20;  // read 超时（ms）
    };

    explicit CommPort(const Config& cfg);
    ~CommPort();

    // 启动：打开串口并可选择起读线程。若启动时找不到首选设备 -> 返回 false。
    bool start(bool spawn_thread = true);

    // 在当前线程运行读循环（阻塞，直到 stop）
    void run();

    // 停止读线程并关闭串口
    void stop();

    // 发送一帧（自动附 CRC，由调用方保证 ProjectileTx 填好）
    bool send(const ProjectileTx& tx);

    // 取最新接收帧（有更新返回 true）
    bool latest(ProjectileRx& out);

    // 状态
    bool is_open() const;

   private:
    // —— 内部工具 ——
    void reader_loop_();
    bool open_preferred_locked_();  // 仅打开首选设备（启动阶段用）
    bool try_reopen_locked_();      // 断开后重连：首选优先，其次扫描 /dev/ttyACM*
    static std::vector<std::string> scan_ttyacm_();

    // —— IO 与配置 ——
    Config cfg_;
    serial::Serial port_;
    mutable std::mutex port_mu_;

    // —— 读线程控制 ——
    std::thread th_;
    std::atomic<bool> running_{false};

    // —— 帧缓冲解析 ——
    std::vector<uint8_t> inbuf_;  // 累积缓冲
    void feed_bytes_and_parse_(const uint8_t* data, size_t n);

    // —— RX 最新值 ——（锁+序号，行级线程安全）
    std::mutex rx_mu_;
    ProjectileRx rx_{};
    std::atomic<uint64_t> rx_seq_{0};
    uint64_t rx_seen_{0};

    // —— 日志（可选）——
    std::shared_ptr<spdlog::logger> log_;
};
