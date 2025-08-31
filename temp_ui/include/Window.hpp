#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "Terminal.hpp"

// Terminal& term, std::string title, int x = 1, int y = 10, int width = 25,
//                     int height = 25, std::string border_color = "#FFFFFF",
//                     std::string text_color = "#FFFFFF"
class Window {
   public:
    struct Ctx {
        Window* w;
        void moveUp() { w->moveCursorUp(); }
        void moveDown() { w->moveCursorDown(); }
        void returnToLast() {
            if (w->on_return_) w->on_return_();
        }
        int cursorIndex() const { return w->cursor_idx_; }
        int itemCount() const { return (int)w->items_.size(); }
    };

    using Action = std::function<void(Ctx&)>;

    void setAction(std::string name, Action act) { actions_[std::move(name)] = std::move(act); }

    bool runAction(const std::string& name) {
        auto it = actions_.find(name);
        if (it == actions_.end()) return false;
        Ctx ctx{this};
        it->second(ctx);
        return true;
    }

    void setOnReturn(std::function<void()> cb) { on_return_ = std::move(cb); }

   private:
    std::unordered_map<std::string, Action> actions_;
    std::function<void()> on_return_;  // Callback for "return" action

   public:
    explicit Window(Terminal& term, std::string title, int x = 1, int y = 10, int width = 25,
                    int height = 25, std::string border_color = "#FFFFFF",
                    std::string text_color = "#FFFFFF")
        : term_(term),
          title_(std::move(title)),
          x_(x),
          y_(y),
          width_(width),
          height_(height),
          border_color_(std::move(border_color)),
          text_color_(std::move(text_color)) {
        inverted_border_color_ = invertHexColor(border_color_);
        inverted_text_color_ = invertHexColor(text_color_);
    }

    /**
     * @brief render this window
     */
    void render() {
        const int iw = innerW();
        const int ih = innerH();

        std::string title = title_;
        if ((int)title.size() > iw) title.resize(iw);
        std::string top = "┌" + title + repeat("─", iw - (int)title.size()) + "┐";
        term_.print(at(x_, y_) + top, 1, 1, border_color_, inverted_border_color_);

        int temp_idx = 0;
        for (temp_idx = 0; temp_idx < ih; ++temp_idx) {
            const int gi = scroll_top_ + temp_idx;
            std::string inner(iw, ' ');  // Default to empty line filled with spaces
            if (gi < (int)keys_.size()) {
                const std::string& key = keys_[gi];           // Use string as key
                const std::string& content = items_.at(key);  // Direct access to value
                std::string line = " " + key + " " + content;
                if ((int)line.size() > iw)
                    line.resize(iw);
                else
                    line.append(iw - (int)line.size(), ' ');  // Ensure fixed width
                inner.swap(line);                             // Place formatted content in the line
            }
            const bool is_cursor_line = (gi == cursor_idx_);
            const std::string line = "│" + inner + "│";
            term_.print(at(x_ + 1 + temp_idx - scroll_top_, y_) + line, 1, 1,
                        is_cursor_line ? inverted_text_color_ : text_color_,
                        is_cursor_line ? text_color_ : inverted_text_color_);
        }

        std::string inner = repeat("─", iw);
        if ((int)keys_.size() > ih && !keys_.empty()) {
            int percent = (int)std::lround(100.0 * (cursor_idx_ + 1) / (double)keys_.size());
            percent = std::clamp(percent, 0, 100);
            std::string tag = " " + std::to_string(percent) + "% ";
            const int pos = std::max(0, iw - (int)tag.size());
            for (int i = 0; i < (int)tag.size() && pos + i < iw; ++i) inner[pos + i] = tag[i];
        }
        std::string bottom = "└" + inner + "┘";
        term_.print(at(x_ + height_ - 1, y_) + bottom, 1, 1, border_color_, inverted_border_color_);
    }

    void enableCursor(bool enable) {
        // ugly impl, but at least it works
        if (enable)
            this->cursor_idx_ = 0;
        else
            this->cursor_idx_ = -1;
    }

    /**
     * @brief move the cursor up
     */
    void moveCursorUp() {
        if (items_.empty()) return;
        if (cursor_idx_ > 0) --cursor_idx_;
        if (cursor_idx_ < scroll_top_) scroll_top_ = cursor_idx_;
    }

    /**
     * @brief move the cursor down
     */
    void moveCursorDown() {
        if (items_.empty()) return;
        if (cursor_idx_ + 1 < (int)items_.size()) ++cursor_idx_;
        const int ih = innerH();
        if (cursor_idx_ >= scroll_top_ + ih) scroll_top_ = cursor_idx_ - ih + 1;
    }

    /**
     * @brief set display content using a map with string keys and string values
     *
     * @param m std::map<std::string, std::string>
     */
    void setDisplayMap(std::map<std::string, std::string>&& m) {
        items_ = std::move(m);
        rebuildKeys();
        clampCursorAndScroll();
    }

    /**
     * @brief add or update a line in the map by key
     *
     * @param key std::string
     * @param content std::string the content to set
     */
    void setLineByKey(const std::string& key, const std::string& content) {
        items_[key] = content;
        rebuildKeys();
        clampCursorAndScroll();
    }

    // Update content by key
    /**
     * @brief update the content by key
     *
     * @param key std::string
     * @param new_content std::string
     * @return whether the update was successful (false if key not found)
     */
    bool updateContentByKey(const std::string& key, const std::string& new_content) {
        auto it = items_.find(key);
        if (it == items_.end()) return false;
        it->second = new_content;
        return true;
    }

    /**
     * @brief remove a line by key
     *
     * @param key std::string
     * @return status of the removal (false if key not found)
     */
    bool eraseByKey(const std::string& key) {
        auto it = items_.find(key);
        if (it == items_.end()) return false;
        items_.erase(it);
        rebuildKeys();
        clampCursorAndScroll();
        return true;
    }

    bool setBorderColor(const std::string& new_color) {
        border_color_ = new_color;
        inverted_border_color_ = invertHexColor(border_color_);
        return true;
    }

    bool setTextColor(const std::string& new_color) {
        text_color_ = new_color;
        inverted_text_color_ = invertHexColor(text_color_);
        return true;
    }

    bool setTitle(const std::string& title) {
        if (title.size() > 2 && title.size() < 100) {
            title_ = title;
            return true;
        }
        return false;
    }

    bool setPosition(int x, int y) {
        if (x > 1) x_ = x;
        if (y > 1) y_ = y;
        return true;
    }

    bool setSize(int width, int height) {
        if (width != -1) {
            if (width < 3 || width > 300) return false;
            width_ = width;
        }
        if (height != -1) {
            if (height < 3 || height > 300) return false;
            height_ = height;
        }
        clampCursorAndScroll();
        return true;
    }

    std::string get_title() const { return title_; }
    std::pair<int, int> get_position() const { return {x_, y_}; }
    std::pair<int, int> get_size() const { return {width_, height_}; }

   private:
    // ---------- Helper Methods ----------
    static std::string invertHexColor(const std::string& hex) {
        if (hex.size() != 7 || hex[0] != '#') return "#000000";  // Safe return
        int r = std::stoi(hex.substr(1, 2), nullptr, 16);
        int g = std::stoi(hex.substr(3, 2), nullptr, 16);
        int b = std::stoi(hex.substr(5, 2), nullptr, 16);
        std::ostringstream oss;
        oss << "#" << std::uppercase << std::setfill('0') << std::hex << std::setw(2) << (255 - r)
            << std::setw(2) << (255 - g) << std::setw(2) << (255 - b);
        return oss.str();
    }

   protected:
    static std::string repeat(const std::string& s, int n) {
        if (n <= 0) return {};
        std::string result;
        result.reserve((int)s.size() * n);
        for (int i = 0; i < n; ++i) result += s;
        return result;
    }
    // ANSI absolute positioning
    static std::string at(int row, int col) {
        return "\033[" + std::to_string(row) + ";" + std::to_string(col) + "H";
    }

    int innerW() const { return std::max(0, width_ - 2); }
    int innerH() const { return std::max(0, height_ - 2); }

   private:
    void rebuildKeys() {
        keys_.clear();
        keys_.reserve(items_.size());
        for (auto& kv : items_) keys_.push_back(kv.first);  // map is ordered by key
        if (cursor_idx_ >= (int)keys_.size()) cursor_idx_ = std::max(0, (int)keys_.size() - 1);
    }

    void clampCursorAndScroll() {
        const int ih = innerH();
        if (ih <= 0) {
            scroll_top_ = 0;
            cursor_idx_ = 0;
            return;
        }
        cursor_idx_ = std::clamp(cursor_idx_, 0, std::max(0, (int)keys_.size() - 1));
        const int max_top = std::max(0, (int)keys_.size() - ih);
        scroll_top_ = std::clamp(scroll_top_, 0, max_top);
        if (cursor_idx_ < scroll_top_) scroll_top_ = cursor_idx_;
        if (cursor_idx_ >= scroll_top_ + ih) scroll_top_ = std::min(cursor_idx_, max_top);
    }

   protected:
    Terminal& term_;
    std::string title_;
    int x_, y_;

    // Cursor & scrolling
    int cursor_idx_ = 0;  // Absolute index in keys_
    int scroll_top_ = 0;  // Top of the visible window in keys_
    // Data: map & ordered key list
    std::map<std::string, std::string> items_;
    std::vector<std::string> keys_;

    int width_, height_;
    std::string border_color_ = "#FFFFFF";
    std::string inverted_border_color_ = "#000000";
    std::string text_color_ = "#FFFFFF";
    std::string inverted_text_color_ = "#000000";
};
