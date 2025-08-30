#pragma once
#include <atomic>
#include <functional>
#include <mutex>
#include <stack>
#include <string>
#include <thread>

#include "SPSCQueue.hpp"
#include "Terminal.hpp"
#include "Window.hpp"

struct Menu {
    explicit Menu(Window& cmd, Window& detector, Window& cam, Window& kalman, Window& contact)
        : cmd_(cmd), detector_(detector), cam_(cam), kalman_(kalman), contact_(contact) {}

    Window& cmd_;
    Window& detector_;
    Window& cam_;
    Window& kalman_;
    Window& contact_;

    Window* current_;

    std::stack<std::reference_wrapper<Window>> history_;  // 存放引用包装器
};

class Tui {
   public:
    explicit Tui(Terminal& term) : term_(term) {}

    bool isRunning() const { return running_.load(); }

    bool start() {
        term_.clear();
        term_.print("TUI started.", 1, 1, "#00FF00");
        term_.print("size:" + std::to_string(term_.getTerminalSize().first) + ", " +
                        std::to_string(term_.getTerminalSize().second),
                    2, 1, "#00FF00");

        _input_thread_ = std::thread(&Tui::inputThreadWorker, this);

        running_.store(true);
        return true;
    }

    bool stop() {
        running_.store(false);
        return true;
    }

    void inputThreadWorker() {
        while (running_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            char ch = term_.nonblocking_input();
            if (ch != '\0') input_char_queue_.push(ch);
        }
    }
    void windowsThreadWorker() {
        while (running_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            cmd_win_.render();
            detector_win_.render();
            cam_win_.render();
            kalman_win_.render();
            contact_win_.render();
        }
    }
    void infoUpdateThreadWorker() {}

   private:
    Terminal& term_;

    Window cmd_win_{term_, "Command", 2, 2, 50, 12};
    Window detector_win_{term_, "Detector", 55, 2, 50, 12};
    Window cam_win_{term_, "Camera", 2, 15, 50, 12};
    Window kalman_win_{term_, "Kalman Filter", 55, 15, 50, 12};
    Window contact_win_{term_, "Contact", 2, 28, 50, 12};

    Menu menu_{cmd_win_, detector_win_, cam_win_, kalman_win_, contact_win_};

    SPSCQueue<char, 1024> input_char_queue_;

    std::thread _input_thread_;
    std::thread _windows_thread_;
    std::thread _info_update_thread_;
    std::atomic<bool> running_{false};
};
