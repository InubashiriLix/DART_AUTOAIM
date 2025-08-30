#pragma once

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>
#include <mutex>
#include <sstream>  // 添加头文件以避免错误

class Terminal {
   public:
    static void clear() { system("clear"); }

    void print(const std::string& text, int row, int col, const std::string& hex_color,
               const std::string& bg_hex_color = "#000000") {
        int r = 255, g = 255, b = 255;  // default text color: white
        int br = 0, bg = 0, bb = 0;     // default background color: black

        // Parse the text color (hex format: #RRGGBB)
        if (hex_color.size() == 7 && hex_color[0] == '#') {
            std::istringstream(hex_color.substr(1, 2)) >> std::hex >> r;
            std::istringstream(hex_color.substr(3, 2)) >> std::hex >> g;
            std::istringstream(hex_color.substr(5, 2)) >> std::hex >> b;
        }

        // Parse the background color (hex format: #RRGGBB)
        if (bg_hex_color.size() == 7 && bg_hex_color[0] == '#') {
            std::istringstream(bg_hex_color.substr(1, 2)) >> std::hex >> br;
            std::istringstream(bg_hex_color.substr(3, 2)) >> std::hex >> bg;
            std::istringstream(bg_hex_color.substr(5, 2)) >> std::hex >> bb;
        }

        {
            std::lock_guard<std::mutex> lock(term_mtx_);
            std::cout << "\033[" << row << ";" << col << "H"
                      << "\033[48;2;" << br << ";" << bg << ";" << bb
                      << "m"                                               // Set background color
                      << "\033[38;2;" << r << ";" << g << ";" << b << "m"  // Set text color
                      << text << "\033[0m"                                 // Reset colors
                      << std::flush;
        }
    }

    char nonblocking_input() {
        char input_ch;
        fd_set set;
        struct timeval timeout;

        // Set timeout to 0 to make it non-blocking
        timeout.tv_sec = 0;
        timeout.tv_usec = 0;

        FD_ZERO(&set);
        FD_SET(tty, &set);

        int res = select(tty + 1, &set, NULL, NULL, &timeout);
        if (res > 0) {
            // If data is available, read it
            read(tty, &input_ch, 1);
            return input_ch;
        }

        return '\0';  // No input
    }

    static std::pair<int, int> getTerminalSize() {
        struct winsize w;
        if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &w) == -1) {
            return {0, 0};  // 获取失败
        }
        return {w.ws_col, w.ws_row};  // {列数, 行数}
    }

   private:
    std::mutex term_mtx_;
    int tty = open("/dev/tty", O_RDONLY);  // Open control terminal
};
