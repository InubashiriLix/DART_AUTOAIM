#include <atomic>
#include <chrono>
#include <ctime>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include "LatestChannel.hpp"
#include "Terminal.hpp"
#include "Tui.hpp"
#include "Window.hpp"

std::shared_ptr<tui::LtxChanStrMapType> kalman_channel = std::make_shared<tui::LtxChanStrMapType>();
std::shared_ptr<tui::LtxChanStrMapType> cam_channel = std::make_shared<tui::LtxChanStrMapType>();
std::shared_ptr<tui::LtxChanStrMapType> detector_channel =
    std::make_shared<tui::LtxChanStrMapType>();
std::shared_ptr<tui::LtxChanStrMapType> contact_channel =
    std::make_shared<tui::LtxChanStrMapType>();

std::thread sim_thread_handle;
std::atomic<bool> sim_running{true};

static std::shared_ptr<std::map<std::string, std::string>> make_msg(std::string header,
                                                                    int counter) {
    auto msg = std::make_shared<std::map<std::string, std::string>>();
    (*msg)["header"] = std::move(header);
    (*msg)["timestamp"] = std::to_string(std::time(nullptr));
    (*msg)["counter"] = std::to_string(counter);
    return msg;
}

void sim_thread() {
    int tick = 0;
    while (sim_running.load(std::memory_order_acquire)) {
        using namespace std::chrono_literals;

        // 演示：键集合稳定（header/timestamp/counter），值在变
        kalman_channel->publish(make_msg("kalman", tick));
        cam_channel->publish(make_msg("camera", tick));
        detector_channel->publish(make_msg("detector", tick));
        contact_channel->publish(make_msg("contact", tick));

        // 如果想测试“键集合变化”的慢路径，可以隔段时间多塞一个新键：
        // if (tick % 50 == 0) {
        //   auto msg = make_msg("kalman", tick);
        //   (*msg)["new_key"] = "appeared";
        //   kalman_channel->publish(msg); // 隐式转为 shared_ptr<const map> 没问题
        // }

        std::this_thread::sleep_for(100ms);
        ++tick;
    }
}

int main() {
    Terminal term;
    term.clear();

    tui::Tui tui(term, detector_channel, cam_channel, kalman_channel, contact_channel);
    tui.prepareWindows();

    sim_thread_handle = std::thread(sim_thread);  // 启动模拟发布线程
    tui.start();

    // 主循环：直到 TUI 停止
    while (tui.isRunning()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // 优雅退出模拟线程
    sim_running.store(false, std::memory_order_release);
    if (sim_thread_handle.joinable()) sim_thread_handle.join();

    return 0;
}
