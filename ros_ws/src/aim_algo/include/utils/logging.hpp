#pragma once
// High-perf async file logging with levels + {} & printf formats.
//
// 依赖: spdlog (header-only 即可)
//  - 仓库: https://github.com/gabime/spdlog
//  - 只需把 spdlog/ 目录放到 include 路径，无需链接任何库
//
// 用法:
//   perflog::init({
//       {"main", "logs/main.log"},
//       {"net",  "logs/net.log"},
//       {"algo", "logs/algo.log"},
//   });
//   auto lg = perflog::get("main");
//   lg->info("hello {} {}", "world", 42);          // {} 风格 (推荐，最快)
//   perflog::infof("net",  "pkt %d size=%zu", n, sz);  // printf 风格
//
// 编译建议: -O3 -DNDEBUG -pthread
// 运行时查看: 每个窗口分别 `tail -F logs/main.log` 等

#include <spdlog/async.h>
#include <spdlog/pattern_formatter.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/daily_file_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#ifndef PERLOG_DEFAULT_QUEUE
#define PERLOG_DEFAULT_QUEUE (256 * 1024)
#endif

#ifndef SPDLOG_ACTIVE_LEVEL
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_INFO
#endif

namespace perflog {

struct Channel {
    std::string name;
    std::string file;
};

inline spdlog::level::level_enum level_from_env() {
    const char* e = std::getenv("LOG_LEVEL");
    if (!e) return spdlog::level::info;
    std::string v(e);
    for (auto& c : v) c = (char)std::tolower((unsigned char)c);
    if (v == "trace") return spdlog::level::trace;
    if (v == "debug") return spdlog::level::debug;
    if (v == "info") return spdlog::level::info;
    if (v == "warn" || v == "warning") return spdlog::level::warn;
    if (v == "error") return spdlog::level::err;
    if (v == "critical") return spdlog::level::critical;
    if (v == "off") return spdlog::level::off;
    return spdlog::level::info;
}

inline void ensure_parent_dirs(const std::string& path) {
    try {
        std::filesystem::path p(path);
        if (p.has_parent_path()) std::filesystem::create_directories(p.parent_path());
    } catch (...) {
    }
}

inline void init(const std::vector<Channel>& channels, size_t queue_size = PERLOG_DEFAULT_QUEUE,
                 spdlog::level::level_enum lvl = level_from_env(),
                 const std::string& pattern = "[%H:%M:%S.%e] [%^%l%$] %v") {
    static bool pool_inited = false;
    if (!pool_inited) {
        auto nthreads = std::max(1u, std::thread::hardware_concurrency());
        spdlog::init_thread_pool(queue_size, nthreads);
        pool_inited = true;
        spdlog::flush_every(std::chrono::seconds(1));
        spdlog::flush_on(spdlog::level::err);
    }
    for (auto& ch : channels) {
        ensure_parent_dirs(ch.file);
        auto sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(ch.file, true);
        auto logger = std::make_shared<spdlog::async_logger>(
            ch.name, sink, spdlog::thread_pool(), spdlog::async_overflow_policy::overrun_oldest);
        logger->set_level(lvl);
        logger->set_pattern(pattern);
        spdlog::register_logger(logger);
    }
}

inline std::shared_ptr<spdlog::logger> get(const std::string& name) {
    auto lg = spdlog::get(name);
    if (!lg) throw std::runtime_error("perflog: logger not found: " + name);
    return lg;
}

// -------- printf 风格：一次 vsnprintf 到 string --------
inline std::string vprintf_to_string(const char* fmt, va_list ap) {
    va_list ap2;
    va_copy(ap2, ap);
    int n = std::vsnprintf(nullptr, 0, fmt, ap2);
    va_end(ap2);
    if (n <= 0) return {};
    std::string s;
    s.resize(static_cast<size_t>(n));
    std::vector<char> buf(static_cast<size_t>(n) + 1);
    std::vsnprintf(buf.data(), buf.size(), fmt, ap);
    std::memcpy(s.data(), buf.data(), static_cast<size_t>(n));
    return s;
}

inline void logf(const std::string& logger_name, spdlog::level::level_enum lvl, const char* fmt,
                 ...) {
    auto lg = get(logger_name);
    va_list ap;
    va_start(ap, fmt);
    std::string s = vprintf_to_string(fmt, ap);
    va_end(ap);
    lg->log(lvl, s);
}

inline void tracef(const std::string& n, const char* f, ...) {
    va_list ap;
    va_start(ap, f);
    std::string s = vprintf_to_string(f, ap);
    va_end(ap);
    get(n)->log(spdlog::level::trace, s);
}
inline void debugf(const std::string& n, const char* f, ...) {
    va_list ap;
    va_start(ap, f);
    std::string s = vprintf_to_string(f, ap);
    va_end(ap);
    get(n)->log(spdlog::level::debug, s);
}
inline void infof(const std::string& n, const char* f, ...) {
    va_list ap;
    va_start(ap, f);
    std::string s = vprintf_to_string(f, ap);
    va_end(ap);
    get(n)->log(spdlog::level::info, s);
}
inline void warnf(const std::string& n, const char* f, ...) {
    va_list ap;
    va_start(ap, f);
    std::string s = vprintf_to_string(f, ap);
    va_end(ap);
    get(n)->log(spdlog::level::warn, s);
}
inline void errorf(const std::string& n, const char* f, ...) {
    va_list ap;
    va_start(ap, f);
    std::string s = vprintf_to_string(f, ap);
    va_end(ap);
    get(n)->log(spdlog::level::err, s);
}
inline void critf(const std::string& n, const char* f, ...) {
    va_list ap;
    va_start(ap, f);
    std::string s = vprintf_to_string(f, ap);
    va_end(ap);
    get(n)->log(spdlog::level::critical, s);
}

inline void shutdown() { spdlog::shutdown(); }

}  // namespace perflog
