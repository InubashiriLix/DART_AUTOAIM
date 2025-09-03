// main_localcam_demo.cpp
// 一个最简可跑的示例：启动相机生产者（LLocalCam）+ 消费者线程（读取 LatestChannel 并显示）。
// 构建：g++ -std=gnu++17 main_localcam_demo.cpp -o demo `pkg-config --cflags --libs opencv4`
// -lpthread 记得把你相机 SDK/封装（CamWrapperDH 等）和这两个头文件一起编译链接。

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>

#include "LatestChannel.hpp"
#include "LocalCam.hpp"

using namespace std;
using namespace std::chrono_literals;
using localcam::LLocalCam;
using localcam::LLocalCamConfig;
using localcam::MatChannel;

// 全局运行标记 + Ctrl-C 处理
static std::atomic<bool> g_running{true};
void handle_sigint(int) { g_running.store(false); }

int main() {
    try {
        std::signal(SIGINT, handle_sigint);
        std::cout << "[demo] starting...\n";

        // 1) 准备通道
        auto chan = std::make_shared<MatChannel>();

        // 2) 相机配置（按需改你的 SN、ROI、FPS 等）
        LLocalCamConfig cfg;
        cfg.SN = "FGV22100004";  // 空串=第一个设备
        cfg.FOR_PC = true;
        cfg.FPS = 120;
        cfg.frame_refresh_rate = 120;
        cfg.IS_ROTATE = false;
        cfg.ROI_width = 1280;
        cfg.ROI_height = 1024;
        cfg.DEEP_COPY_ON_PUBLISH = true;  // 强烈建议：避免 SDK 复用缓冲导致的悬挂

        // 3) 启动相机生产者
        LLocalCam cam(chan, cfg);
        if (!cam.start()) {
            std::cerr << "[demo] camera start failed.\n";
            return 1;
        }

        // 4) 启动消费者线程：阻塞等待新帧，显示/统计 FPS
        std::thread consumer([&] {
            auto tk = chan->ticket();

            // FPS 统计
            auto t_start = std::chrono::steady_clock::now();
            int frame_cnt = 0;

            cv::namedWindow("localcam_demo", cv::WINDOW_AUTOSIZE);
            while (g_running.load(std::memory_order_relaxed)) {
                auto res = chan->wait_next_for(tk, 500ms);
                if (!res) continue;  // 超时继续等
                auto [sp, ntk] = *res;
                tk = ntk;
                if (!sp) continue;

                // sp 是 shared_ptr<const std::pair<cv::Mat,int>>
                const auto& [frame, ts_ms] = *sp;  // 结构化绑定 pair
                if (frame.empty()) continue;

                // 简单显示（注意：如果是 MONO8 可以直接 imshow；如果是
                // 16UC1，你可以先做可视化映射）
                if (frame.type() == CV_16UC1) {
                    cv::Mat vis;
                    cv::Mat tmp;
                    frame.convertTo(tmp, CV_8U, 1.0 / 256.0);  // 简单压缩到 8bit
                    cv::applyColorMap(tmp, vis, cv::COLORMAP_JET);
                    cv::imshow("localcam_demo", vis);
                } else {
                    cv::imshow("localcam_demo", frame);
                }
                cv::waitKey(1);

                // FPS
                frame_cnt++;
                auto now = std::chrono::steady_clock::now();
                if (now - t_start >= 1s) {
                    std::cout << "[demo] FPS ~= " << frame_cnt << " | ts(ms)=" << ts_ms << "\n";
                    frame_cnt = 0;
                    t_start = now;
                }
            }
            cv::destroyWindow("localcam_demo");
        });

        // 5) 主线程等待退出
        while (g_running.load(std::memory_order_relaxed)) {
            std::this_thread::sleep_for(50ms);
        }

        // 6) 清理
        cam.stop();
        if (consumer.joinable()) consumer.join();

        std::cout << "[demo] bye.\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Unhandled exception: " << e.what() << std::endl;
        return 2;
    }
}
