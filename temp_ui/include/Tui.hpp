#pragma once
#include <atomic>
#include <functional>
#include <mutex>
#include <stack>
#include <string>
#include <thread>

#include "CommandWindow.hpp"
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
    ~Tui() { stop(); }

    bool isRunning() const { return running_.load(); }

    void prepareWindows() {
        {
            menu_.current_ = &cmd_win_;
            cmd_win_.setTabs(this->_tabs_vec_);
            cmd_win_.setActiveTab(0);

            cmd_win_.enableCursor(false);
            cmd_win_.setOnReturn([this] { stop(); });

            // Detector
            cmd_win_.setAction("D", [this](Window::Ctx& ctx) {
                this->menu_.current_ = &this->detector_win_;
                this->cmd_win_.setActiveTab(1);
            });
            // Camera
            cmd_win_.setAction("C", [this](Window::Ctx& ctx) {
                this->menu_.current_ = &this->cam_win_;
                this->cmd_win_.setActiveTab(2);
            });
            // kalman
            cmd_win_.setAction("K", [this](Window::Ctx& ctx) {
                this->menu_.current_ = &this->kalman_win_;
                this->cmd_win_.setActiveTab(3);
            });
            // contact
            cmd_win_.setAction("c", [this](Window::Ctx& ctx) {
                this->menu_.current_ = &this->contact_win_;
                this->cmd_win_.setActiveTab(4);
            });
            cmd_win_.setAction("Quit", [this](Window::Ctx& ctx) { stop(); });
        }

        {
            detector_win_.setAction("j", [](Window::Ctx& ctx) { ctx.moveDown(); });
            detector_win_.setAction("k", [](Window::Ctx& ctx) { ctx.moveUp(); });
            detector_win_.setAction("q", [this](Window::Ctx& ctx) {
                this->menu_.current_ = &this->cmd_win_;
                this->cmd_win_.setActiveTab(0);
            });
        }

        {
            cam_win_.setAction("j", [](Window::Ctx& ctx) { ctx.moveDown(); });
            cam_win_.setAction("k", [](Window::Ctx& ctx) { ctx.moveUp(); });
            cam_win_.setAction("q", [this](Window::Ctx& ctx) {
                this->menu_.current_ = &this->cmd_win_;
                this->cmd_win_.setActiveTab(0);
            });
        }

        {
            kalman_win_.setAction("j", [](Window::Ctx& ctx) { ctx.moveDown(); });
            kalman_win_.setAction("k", [](Window::Ctx& ctx) { ctx.moveUp(); });
            kalman_win_.setAction("q", [this](Window::Ctx& ctx) {
                this->menu_.current_ = &this->cmd_win_;
                this->cmd_win_.setActiveTab(0);
            });
        }

        {
            contact_win_.setAction("j", [](Window::Ctx& ctx) { ctx.moveDown(); });
            contact_win_.setAction("k", [](Window::Ctx& ctx) { ctx.moveUp(); });
            contact_win_.setAction("q", [this](Window::Ctx& ctx) {
                this->menu_.current_ = &this->cmd_win_;
                this->cmd_win_.setActiveTab(0);
            });
        }
    }

    bool start() {
        term_.clear();
        running_.store(true, std::memory_order_relaxed);
        term_.print("TUI started.", 1, 1, "#00FF00");
        term_.print("size:" + std::to_string(term_.getTerminalSize().first) + ", " +
                        std::to_string(term_.getTerminalSize().second),
                    2, 1, "#00FF00");

        _input_thread_ = std::thread(&Tui::inputThreadWorker, this);
        // 建议把渲染线程也开上，不然看不到刷新
        _windows_thread_ = std::thread(&Tui::windowsThreadWorker, this);

        return true;
    }

    bool stop() {
        // 只处理一次
        if (!running_.exchange(false, std::memory_order_relaxed)) return true;

        if (_input_thread_.joinable()) _input_thread_.join();
        if (_windows_thread_.joinable()) _windows_thread_.join();
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
            std::pair<unsigned short, unsigned short> size;
            size = term_.getTerminalSize();
            if (size.first < 120 || size.second < 40) {
                term_.clear();
                term_.print("Terminal size too small! Please resize to at least 120x40.", 1, 1,
                            "#FF0000");
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }

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

    CommandWindow cmd_win_{term_, "Command", 36, 1, 120, 5};
    Window detector_win_{term_, "Detector", 1, 1, 30, 35};
    Window cam_win_{term_, "Camera", 1, 31, 30, 35};
    Window kalman_win_{term_, "Kalman Filter", 1, 61, 30, 35};
    Window contact_win_{term_, "Contact", 1, 91, 30, 35};

    Menu menu_{cmd_win_, detector_win_, cam_win_, kalman_win_, contact_win_};

    SPSCQueue<char, 1024> input_char_queue_;

    std::thread _input_thread_;
    std::thread _windows_thread_;
    std::thread _info_update_thread_;
    std::atomic<bool> running_{false};

    std::vector<std::string> _tabs_vec_ = {"Command", "Detector", "Camera", "Kalman", "Contact"};
};
