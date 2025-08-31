#pragma once

#include <map>
#include <vector>

#include "Terminal.hpp"
#include "Window.hpp"

class CommandWindow : public Window {
   public:
    explicit CommandWindow(Terminal& term, std::string title = "Commands", int x = 1, int y = 1,
                           int width = 25, int height = 10,
                           std::map<std::string, std::string> hint_map = {},
                           std::string border_color = "#FFFFFF", std::string text_color = "#FFFFFF")
        : Window(term, std::move(title), x, y, width, height, std::move(hint_map),
                 std::move(border_color), std::move(text_color)) {}

    // 注入：所有窗口名（按横向顺序显示）
    void setTabs(std::vector<std::string> tabs) { tabs_ = std::move(tabs); }
    // 注入：当前聚焦窗口索引（高亮）
    void setActiveTab(int idx) { active_idx_ = idx; }
    // 注入：提示（key 高亮）
    void setHints(std::map<std::string, std::string> hints) { hints_ = std::move(hints); }

    void render() {
        const int iw = innerW();
        const int ih = innerH();

        // 顶部边框
        std::string title = title_;
        if ((int)title.size() > iw) title.resize(iw);
        std::string top = "┌" + title + repeat("─", iw - (int)title.size()) + "┐";
        term_.print(at(x_, y_) + top, 1, 1, border_color_, inverted_border_color_);

        // 中间空白区（不显示 items）
        for (int row = 0; row < ih - 2; ++row) {
            const std::string line = "│" + std::string(iw, ' ') + "│";
            term_.print(at(x_ + 1 + row, y_) + line, 1, 1, text_color_, inverted_text_color_);
        }

        // ---- 倒数第二行：Hints（key 高亮，value 普通，有分隔） ----
        const int hint_row_abs = x_ + height_ - 3;  // inner 倒数第二行
        // 先画左右边框（只画‘│’，避免覆盖中间的彩色块）
        term_.print(at(hint_row_abs, y_) + "│", 1, 1, border_color_, inverted_border_color_);
        term_.print(at(hint_row_abs, y_ + width_ - 1) + "│", 1, 1, border_color_,
                    inverted_border_color_);

        {
            int pos = 0;  // 内部偏移（0..iw-1）
            bool first = true;
            for (const auto& kv : hints_) {
                const std::string key_token = "[" + kv.first + "]";
                const std::string val_token = kv.second;
                const std::string sep = first ? "" : "  ·  ";
                int need = (int)sep.size() + (int)key_token.size() + 1 + (int)val_token.size();
                if (pos + need > iw) break;

                if (!sep.empty()) {
                    term_.print(at(hint_row_abs, y_ + 1 + pos) + sep, 1, 1, text_color_,
                                inverted_text_color_);
                    pos += (int)sep.size();
                }
                // key 高亮（绿色前景；或你也可换成绿色背景）
                term_.print(at(hint_row_abs, y_ + 1 + pos) + key_token, 1, 1, "#00FF00",
                            inverted_text_color_);
                pos += (int)key_token.size();

                // 间隔空格
                term_.print(at(hint_row_abs, y_ + 1 + pos) + " ", 1, 1, text_color_,
                            inverted_text_color_);
                pos += 1;

                // value 普通
                term_.print(at(hint_row_abs, y_ + 1 + pos) + val_token, 1, 1, text_color_,
                            inverted_text_color_);
                pos += (int)val_token.size();

                first = false;
            }

            // 余下空间填空（可选，不填也行）
            if (pos < iw) {
                term_.print(at(hint_row_abs, y_ + 1 + pos) + std::string(iw - pos, ' '), 1, 1,
                            text_color_, inverted_text_color_);
            }
        }

        // ---- 倒数第一行：tmux 风格窗口条（仅当前窗口高亮） ----
        const int status_row_abs = x_ + height_ - 2;  // inner 最后一行
        term_.print(at(status_row_abs, y_) + "│", 1, 1, border_color_, inverted_border_color_);
        term_.print(at(status_row_abs, y_ + width_ - 1) + "│", 1, 1, border_color_,
                    inverted_border_color_);

        {
            int pos = 0;
            bool first = true;
            for (int i = 0; i < (int)tabs_.size(); ++i) {
                const std::string sep = first ? "" : "  ·  ";
                const std::string label =
                    tabs_[i];  // 只显示名称；需要索引可改成 std::to_string(i+1)+":"+label
                const int need =
                    (int)sep.size() + 2 + (int)label.size() + 2;  // “  label  ” 两侧空格
                if (pos + need > iw) break;

                if (!sep.empty()) {
                    term_.print(at(status_row_abs, y_ + 1 + pos) + sep, 1, 1, text_color_,
                                inverted_text_color_);
                    pos += (int)sep.size();
                }

                const std::string pad = " ";
                const std::string chunk = pad + label + pad;

                if (i == active_idx_) {
                    // 聚焦高亮：黑字绿底
                    term_.print(at(status_row_abs, y_ + 1 + pos) + chunk, 1, 1, "#000000",
                                "#00FF00");
                } else {
                    // 非聚焦：常规
                    term_.print(at(status_row_abs, y_ + 1 + pos) + chunk, 1, 1, text_color_,
                                inverted_text_color_);
                }
                pos += (int)chunk.size();

                first = false;
            }

            // 余下空白
            if (pos < iw) {
                term_.print(at(status_row_abs, y_ + 1 + pos) + std::string(iw - pos, ' '), 1, 1,
                            text_color_, inverted_text_color_);
            }
        }

        // 底部边框
        std::string bottom = "└" + repeat("─", iw) + "┘";
        term_.print(at(x_ + height_ - 1, y_) + bottom, 1, 1, border_color_, inverted_border_color_);
    }

   private:
    std::vector<std::string> tabs_;
    int active_idx_ = 0;
    std::map<std::string, std::string> hints_;
};
