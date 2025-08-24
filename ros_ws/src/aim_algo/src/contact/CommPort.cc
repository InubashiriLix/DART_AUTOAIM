#include "contact/CommPort.h"

#include <filesystem>
#include <iostream>

#include "contact/Checksum.h"

using namespace std::chrono_literals;

CommPort::CommPort(const Config& cfg) : cfg_(cfg) {
    inbuf_.reserve(cfg_.packet_size * 4);
    // 可选 spdlog：没有就用默认
    log_ = spdlog::get("cv_program_logger");
    if (!log_) log_ = spdlog::default_logger();
}

CommPort::~CommPort() { stop(); }

bool CommPort::is_open() const {
    std::lock_guard<std::mutex> lk(port_mu_);
    return port_.isOpen();
}

bool CommPort::start(bool spawn_thread) {
    if (running_.exchange(true)) return true;  // 已经在跑

    {
        std::lock_guard<std::mutex> lk(port_mu_);
        if (!open_preferred_locked_()) {
            running_ = false;
            if (log_)
                log_->critical("Serial start failed: preferred {} not present", cfg_.preferred_dev);
            else
                std::cerr << "[Serial] start failed: " << cfg_.preferred_dev << "\n";
            return false;  // 启动阶段严格要求首选存在
        }
    }

    if (spawn_thread) {
        th_ = std::thread([this] { this->run(); });
    }
    return true;
}

void CommPort::run() {
    // 固定小超时轮询，避免 busy-wait，也不会阻塞 stop()
    while (running_) {
        try {
            size_t n = 0;
            {
                std::lock_guard<std::mutex> lk(port_mu_);
                if (!port_.isOpen()) {
                    // 运行中断开：尝试重连（首选优先→扫描）
                    if (!try_reopen_locked_()) {
                        std::this_thread::sleep_for(200ms);
                        continue;
                    }
                }
                n = port_.available();
                if (n == 0) {
                    // 读一次以触发 timeout 退出，让出 CPU
                    uint8_t tmp;
                    serial::Timeout to = serial::Timeout::simpleTimeout(cfg_.timeout_ms);
                    port_.setTimeout(to);  // 注意：需要左值
                    size_t r = port_.read(&tmp, 0);
                    (void)r;
                    std::this_thread::sleep_for(1ms);
                    continue;
                }
            }

            // 读取数据（最多抓一批）
            std::vector<uint8_t> buf(std::min(n, cfg_.packet_size * 8));
            {
                std::lock_guard<std::mutex> lk(port_mu_);
                size_t r = port_.read(buf.data(), buf.size());
                buf.resize(r);
            }
            if (!buf.empty()) {
                feed_bytes_and_parse_(buf.data(), buf.size());
            }
        } catch (const std::exception& e) {
            if (log_)
                log_->error("Serial read exception: {}", e.what());
            else
                std::cerr << "[Serial] read exception: " << e.what() << "\n";
            std::this_thread::sleep_for(200ms);
            std::lock_guard<std::mutex> lk(port_mu_);
            try_reopen_locked_();
        }
    }
}

void CommPort::stop() {
    bool was = running_.exchange(false);
    if (was && th_.joinable()) th_.join();
    std::lock_guard<std::mutex> lk(port_mu_);
    if (port_.isOpen()) {
        try {
            port_.close();
        } catch (...) {
        }
    }
}

bool CommPort::send(const ProjectileTx& tx) {
    // 打 32B 包 + CRC（按你的协议：结构体已对齐到 32B，CRC 附在尾部）
    std::vector<uint8_t> out(cfg_.packet_size, 0);
    static_assert(sizeof(ProjectileTx) <= 32, "ProjectileTx exceeds packet_size");
    std::memcpy(out.data(), &tx, sizeof(ProjectileTx));
    // 你自己的 CRC 附加：对整包计算/附加；若 CRC 在特定偏移，请按需修改
    Crc8Append(out.data(), out.size());

    try {
        std::lock_guard<std::mutex> lk(port_mu_);
        if (!port_.isOpen()) {
            if (!try_reopen_locked_()) return false;
        }
        port_.write(out.data(), out.size());
        return true;
    } catch (const std::exception& e) {
        if (log_)
            log_->error("Serial write failed: {}", e.what());
        else
            std::cerr << "[Serial] write failed: " << e.what() << "\n";
        // 尝试重连，下次再写
        std::lock_guard<std::mutex> lk(port_mu_);
        try_reopen_locked_();
        return false;
    }
}

bool CommPort::latest(ProjectileRx& out) {
    uint64_t now = rx_seq_.load(std::memory_order_acquire);
    if (now == rx_seen_) return false;
    std::lock_guard<std::mutex> lk(rx_mu_);
    out = rx_;
    rx_seen_ = now;
    return true;
}

// —— 内部：仅在启动阶段打开首选设备；失败返回 false ——
bool CommPort::open_preferred_locked_() {
    try {
        // 确认设备存在
        if (!std::filesystem::exists(cfg_.preferred_dev)) return false;
        if (port_.isOpen()) port_.close();
        port_.setPort(cfg_.preferred_dev);
        port_.setBaudrate(cfg_.baud);
        serial::Timeout to = serial::Timeout::simpleTimeout(cfg_.timeout_ms);
        port_.setTimeout(to);  // 注意左值
        port_.open();
        if (log_) log_->info("Serial opened: {} @{}", cfg_.preferred_dev, cfg_.baud);
        return true;
    } catch (...) {
        return false;
    }
}

// —— 内部：运行中断开后的重连：首选优先，随后扫描 /dev/ttyACM* ——
bool CommPort::try_reopen_locked_() {
    // 1) 优先首选
    try {
        if (port_.isOpen()) port_.close();
    } catch (...) {
    }

    if (std::filesystem::exists(cfg_.preferred_dev)) {
        try {
            port_.setPort(cfg_.preferred_dev);
            port_.setBaudrate(cfg_.baud);
            serial::Timeout to = serial::Timeout::simpleTimeout(cfg_.timeout_ms);
            port_.setTimeout(to);
            port_.open();
            if (log_) log_->warn("Serial reconnected (preferred): {}", cfg_.preferred_dev);
            return true;
        } catch (...) {
            // 继续扫描
        }
    }

    // 2) 扫描 /dev/ttyACM*
    for (const auto& dev : scan_ttyacm_()) {
        try {
            port_.setPort(dev);
            port_.setBaudrate(cfg_.baud);
            serial::Timeout to = serial::Timeout::simpleTimeout(cfg_.timeout_ms);
            port_.setTimeout(to);
            port_.open();
            if (log_) log_->warn("Serial reconnected (scanned): {}", dev);
            return true;
        } catch (...) {
            continue;
        }
    }
    if (log_) log_->error("Serial reconnect failed: no ttyACM available");
    return false;
}

// —— 扫描 /dev 下的 ttyACM* ——
std::vector<std::string> CommPort::scan_ttyacm_() {
    std::vector<std::string> out;
    try {
        for (auto& p : std::filesystem::directory_iterator("/dev")) {
            if (!p.is_character_file()) continue;
            auto name = p.path().filename().string();
            if (name.rfind("ttyACM", 0) == 0) {
                out.push_back(p.path().string());
            }
        }
    } catch (...) {
    }
    // 首选放到最前（如果存在）
    std::sort(out.begin(), out.end());
    return out;
}

// —— 累积缓冲 + 帧解析（固定头+定长包；可按需加 CRC 验证）——
void CommPort::feed_bytes_and_parse_(const uint8_t* data, size_t n) {
    inbuf_.insert(inbuf_.end(), data, data + n);

    // 丢弃头部前的噪声
    auto ps = cfg_.packet_size;
    while (inbuf_.size() >= ps) {
        // 找到帧头
        if (inbuf_[0] != cfg_.header) {
            // 找下一个 0x3A
            auto it = std::find(inbuf_.begin() + 1, inbuf_.end(), cfg_.header);
            if (it == inbuf_.end()) {
                inbuf_.clear();
                return;
            }
            inbuf_.erase(inbuf_.begin(), it);
            if (inbuf_.size() < ps) return;
        }
        // 够长，取出一帧
        if (inbuf_.size() < ps) return;
        // 如果需要 CRC 验证，这里调用 Crc8Verify(inbuf_.data(), ps) 通过后再 memcpy
        ProjectileRx tmp{};
        std::memcpy(&tmp, inbuf_.data(), sizeof(ProjectileRx));
        {
            std::lock_guard<std::mutex> lk(rx_mu_);
            rx_ = tmp;
        }
        rx_seq_.fetch_add(1, std::memory_order_release);
        inbuf_.erase(inbuf_.begin(), inbuf_.begin() + ps);
    }
}
