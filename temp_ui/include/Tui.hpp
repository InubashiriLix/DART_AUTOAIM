// Tui.hpp
#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <ranges>
#include <stack>
#include <string>
#include <thread>

#include "CommandWindow.hpp"
#include "LatestChannel.hpp"
#include "SPSCQueue.hpp"
#include "Terminal.hpp"
#include "Window.hpp"

namespace tui {

using LtxChanStrMapType = LatestChannel<std::map<std::string, std::string>>;
using msg_ptr_type = std::shared_ptr<const std::map<std::string, std::string>>;

class Tui {
   public:
    explicit Tui(Terminal& term, std::shared_ptr<LtxChanStrMapType> detector_channel,
                 std::shared_ptr<LtxChanStrMapType> cam_channel,
                 std::shared_ptr<LtxChanStrMapType> kalman_channel,
                 std::shared_ptr<LtxChanStrMapType> contact_channel)
        : term_(term),
          detector_channel_(detector_channel),
          cam_channel_(cam_channel),
          kalman_channel_(kalman_channel),
          contact_channel_(contact_channel) {
        this->cmd_win_.setHints(this->cmd_win_.getHints());
        this->cmd_win_.setBorderColor("#FFFF00", "#000000");
    }
    ~Tui() { stop(); }

    bool isRunning() const { return running_.load(); }

    void prepareWindows() {
        {
            last_win_ = nullptr;
            current_win_ = &cmd_win_;
            single_target_ = &detector_win_;

            cmd_win_.setTabs(this->_tabs_vec_);
            cmd_win_.setActiveTab(0);

            cmd_win_.enableCursor(false);
            cmd_win_.setOnReturn([this] { request_stop(); });

            // Detector
            cmd_win_.setAction("D", [this](Window::Ctx& ctx) {
                switch_to(detector_win_, 1);  // 统一切换路径
            });
            // Camera
            cmd_win_.setAction("C", [this](Window::Ctx& ctx) { switch_to(cam_win_, 2); });
            // kalman
            cmd_win_.setAction("K", [this](Window::Ctx& ctx) { switch_to(kalman_win_, 3); });
            // contact
            cmd_win_.setAction("c", [this](Window::Ctx& ctx) { switch_to(contact_win_, 4); });
            cmd_win_.setAction("Q", [this](Window::Ctx& ctx) { request_stop(); });
            cmd_win_.setAction("t", [this](Window::Ctx& ctx) {
                toggle_display();
                cmd_dirty_.store(true, std::memory_order_release);
            });
        }

        // detector config
        {
            detector_win_.setAction("j", [](Window::Ctx& ctx) { ctx.moveDown(); });
            detector_win_.setAction("k", [](Window::Ctx& ctx) { ctx.moveUp(); });
            detector_win_.setAction("q", [this](Window::Ctx& ctx) { switch_to(cmd_win_, 0); });
        }

        // cam win config
        {
            cam_win_.setAction("j", [](Window::Ctx& ctx) { ctx.moveDown(); });
            cam_win_.setAction("k", [](Window::Ctx& ctx) { ctx.moveUp(); });
            cam_win_.setAction("q", [this](Window::Ctx& ctx) { switch_to(cmd_win_, 0); });
        }

        // kalman win config
        {
            kalman_win_.setAction("j", [](Window::Ctx& ctx) { ctx.moveDown(); });
            kalman_win_.setAction("k", [](Window::Ctx& ctx) { ctx.moveUp(); });
            kalman_win_.setAction("q", [this](Window::Ctx& ctx) { switch_to(cmd_win_, 0); });
        }

        // contact win config
        {
            contact_win_.setAction("j", [](Window::Ctx& ctx) { ctx.moveDown(); });
            contact_win_.setAction("k", [](Window::Ctx& ctx) { ctx.moveUp(); });
            contact_win_.setAction("q", [this](Window::Ctx& ctx) { switch_to(cmd_win_, 0); });
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
        // 启动时先画一次状态栏
        this->cmd_win_.render();
        cmd_dirty_.store(false, std::memory_order_release);
        _windows_thread_ = std::thread(&Tui::windowsThreadWorker, this);
        _info_update_thread_ = std::thread(&Tui::infoUpdateThreadWorker, this);

        return true;
    }

    void request_stop() { running_.store(false, std::memory_order_relaxed); }

    // 真正的停止 + join（建议主线程/析构里调用）
    bool stop() {
        running_.store(false, std::memory_order_relaxed);
        const auto self = std::this_thread::get_id();

        if (_input_thread_.joinable() && _input_thread_.get_id() != self) _input_thread_.join();
        if (_windows_thread_.joinable() && _windows_thread_.get_id() != self)
            _windows_thread_.join();
        if (_info_update_thread_.joinable() && _info_update_thread_.get_id() != self)
            _info_update_thread_.join();
        return true;
    }

    void inputThreadWorker() {
        while (running_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            char ch = term_.nonblocking_input();
            if (ch != '\0') (void)input_char_queue_.push(ch);
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

            char ch = '\0';
            if (input_char_queue_.pop(ch) && ch != '\0') {
                this->current_win_->runAction(std::string(1, ch));
                // std::cout << "get input: " << ch << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            if (full_redraw_.exchange(false, std::memory_order_acq_rel)) {
                term_.clear();
            }

            if (cmd_dirty_.exchange(false, std::memory_order_acq_rel)) {
                std::lock_guard<std::mutex> lk(cmd_bar_mtx_);
                cmd_win_.render();
            }

            auto single_dis = single_display_.load(std::memory_order_acquire);
            if (!single_dis) {
                detector_win_.render();
                cam_win_.render();
                kalman_win_.render();
                contact_win_.render();
            } else {
                Window* w = single_target_;
                if (w && w != &cmd_win_) {
                    w->render();
                }
            }
        }
    }

    void infoUpdateThreadWorker() {
        while (running_.load(std::memory_order_acq_rel)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            // detector
            auto detector_data = detector_channel_->get();
            conciseSetDisplayMap(detector_win_, detector_data, last_detector_data_);
            last_detector_data_ = detector_data;

            // cam
            auto cam_data = cam_channel_->get();
            conciseSetDisplayMap(cam_win_, cam_data, last_cam_data_);
            last_cam_data_ = cam_data;

            // kalman
            auto kalman_data = kalman_channel_->get();
            conciseSetDisplayMap(kalman_win_, kalman_data, last_kalman_data_);
            last_kalman_data_ = kalman_data;

            // contact
            auto contact_data = contact_channel_->get();
            conciseSetDisplayMap(contact_win_, contact_data, last_contact_data_);
            last_contact_data_ = contact_data;
        }
    }

   private:
    Terminal& term_;

    // WARNING: cause' the std::move, these two hints are null after initializing windows objs!!!
    // and also, the tui class should be long-lived, otherwise the references in windows are
    // dangling
    std::map<std::string, std::string> _cmd_hints_ = {{"D", "Detector"}, {"C", "Camera"},
                                                      {"K", "Kalman"},   {"c", "Contact"},
                                                      {"Q", "Quit UI"},  {"t", "Toggle Display"}};
    std::map<std::string, std::string> _common_hints_ = {{"j/k", "Move"}, {"q", "Back to Command"}};

    CommandWindow cmd_win_{term_, "Command", 36, 1, 120, 5, _cmd_hints_};
    Window detector_win_{term_, "Detector", 1, 1, 30, 35, _common_hints_};
    Window cam_win_{term_, "Camera", 1, 31, 30, 35, _common_hints_};
    Window kalman_win_{term_, "Kalman Filter", 1, 61, 30, 35, _common_hints_};
    Window contact_win_{term_, "Contact", 1, 91, 30, 35, _common_hints_};

    Window* current_win_ = &cmd_win_;
    Window* last_win_ = nullptr;
    Window* single_target_ = nullptr;

    SPSCQueue<char, 1024> input_char_queue_;

    std::thread _input_thread_;
    std::thread _windows_thread_;
    std::thread _info_update_thread_;
    std::atomic<bool> running_{false};

    // the channnel required to update the cannel;
    // the detector channel
    std::shared_ptr<LtxChanStrMapType> detector_channel_;
    // the cam channel
    std::shared_ptr<LtxChanStrMapType> cam_channel_;
    // the kalman channel
    std::shared_ptr<LtxChanStrMapType> kalman_channel_;
    // the contact channnel
    std::shared_ptr<LtxChanStrMapType> contact_channel_;

    std::shared_ptr<const std::map<std::string, std::string>> last_detector_data_;
    std::shared_ptr<const std::map<std::string, std::string>> last_cam_data_;
    std::shared_ptr<const std::map<std::string, std::string>> last_kalman_data_;
    std::shared_ptr<const std::map<std::string, std::string>> last_contact_data_;

    std::vector<std::string> _tabs_vec_ = {"Command", "Detector", "Camera", "Kalman", "Contact"};

    std::atomic<bool> single_display_ = false;

    std::atomic<bool> cmd_dirty_{true};
    std::mutex cmd_bar_mtx_;

    std::atomic<bool> full_redraw_{false};

    bool toggle_display() {
        auto temp = single_display_.exchange(!single_display_.load(std::memory_order_acquire),
                                             std::memory_order_release);
        auto single_dis = single_display_.load(std::memory_order_acquire);

        detector_win_.setPosition(1, single_dis ? 1 : 1);
        detector_win_.setSize(single_dis ? 120 : 30, 35);

        cam_win_.setPosition(1, single_dis ? 1 : 31);
        cam_win_.setSize(single_dis ? 120 : 30, 35);

        kalman_win_.setPosition(1, single_dis ? 1 : 61);
        kalman_win_.setSize(single_dis ? 120 : 30, 35);

        contact_win_.setPosition(1, single_dis ? 1 : 91);
        contact_win_.setSize(single_dis ? 120 : 30, 35);

        if (single_dis) {
            if (current_win_ != &cmd_win_) {
                single_target_ = current_win_;
            } else if (last_win_ && last_win_ != &cmd_win_) {
                single_target_ = last_win_;
            } else {
                single_target_ = &detector_win_;
            }
        }

        full_redraw_.store(true, std::memory_order_release);
        cmd_dirty_.store(true, std::memory_order_release);

        return temp;
    }

    void exchangeFocusWinColor(Window& from, Window& to) {
        from.setBorderColor("#FFFFFF", "#000000");
        to.setBorderColor("#FFFF00", "#000000");
    }

    void switch_to(Window& to, int tab_idx) {
        if (current_win_ != &to) last_win_ = current_win_;
        Window* from = current_win_;
        current_win_ = &to;

        {
            std::lock_guard<std::mutex> lk(cmd_bar_mtx_);
            cmd_win_.setActiveTab(tab_idx);
            cmd_win_.setHints(to.getHints());
            if (from) exchangeFocusWinColor(*from, to);
        }
        if (&to != &cmd_win_) {
            single_target_ = &to;
        }
        cmd_dirty_.store(true, std::memory_order_release);
    }

    inline bool same_keys20(const msg_ptr_type& a, const msg_ptr_type& b) {  // NOTE: require cpp20
        if (a == b) return true;
        if (!a || !b || a->size() != b->size()) return false;
        return std::ranges::equal(
            *a, *b, [](auto const& lhs, auto const& rhs) { return lhs.first == rhs.first; });
    }

    void conciseSetDisplayMap(Window& win, const msg_ptr_type& data,
                              const msg_ptr_type& last_data) {
        if (!data) return;
        if (same_keys20(data, last_data)) {
            win.setDisplayMapValuesOnly(data);
        } else {
            win.setDisplayMap(data);
        }
    }
};

}  // namespace tui
