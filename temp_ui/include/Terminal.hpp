#pragma once
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>
#include <mutex>
#include <sstream>

class Terminal {
   public:
    Terminal() {
        fd_ = STDIN_FILENO;
        if (!isatty(fd_)) {
            fd_ = ::open("/dev/tty", O_RDONLY | O_NONBLOCK);
            use_dup_ = (fd_ >= 0);
        }

        if (tcgetattr(fd_, &orig_) == 0) {
            raw_ = orig_;
            raw_.c_lflag &= ~(ICANON | ECHO);
            // 让 read 立即返回：没有字节就返回 0/EAGAIN
            raw_.c_cc[VMIN] = 0;
            raw_.c_cc[VTIME] = 0;
            tcsetattr(fd_, TCSANOW, &raw_);
            int flags = fcntl(fd_, F_GETFL, 0);
            fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
            raw_enabled_ = true;
        }
    }

    ~Terminal() {
        if (raw_enabled_) tcsetattr(fd_, TCSANOW, &orig_);
        if (use_dup_ && fd_ >= 0) ::close(fd_);
    }

    static void clear() { ::system("clear"); }

    void print(const std::string& text, int row, int col, const std::string& hex_color,
               const std::string& bg_hex_color = "#000000") {
        int r = 255, g = 255, b = 255;
        int br = 0, bg = 0, bb = 0;
        if (hex_color.size() == 7 && hex_color[0] == '#') {
            std::istringstream(hex_color.substr(1, 2)) >> std::hex >> r;
            std::istringstream(hex_color.substr(3, 2)) >> std::hex >> g;
            std::istringstream(hex_color.substr(5, 2)) >> std::hex >> b;
        }
        if (bg_hex_color.size() == 7 && bg_hex_color[0] == '#') {
            std::istringstream(bg_hex_color.substr(1, 2)) >> std::hex >> br;
            std::istringstream(bg_hex_color.substr(3, 2)) >> std::hex >> bg;
            std::istringstream(bg_hex_color.substr(5, 2)) >> std::hex >> bb;
        }
        std::lock_guard<std::mutex> lock(term_mtx_);
        std::cout << "\033[" << row << ";" << col << "H"
                  << "\033[48;2;" << br << ";" << bg << ";" << bb << "m"
                  << "\033[38;2;" << r << ";" << g << ";" << b << "m" << text << "\033[0m"
                  << std::flush;

        // std::cout << "\033[0;0H" << std::flush;
    }

    char nonblocking_input() {
        char ch;
        ssize_t n = ::read(fd_, &ch, 1);
        if (n == 1) return ch;
        if (n == -1 && (errno == EAGAIN || errno == EWOULDBLOCK)) return '\0';
        return '\0';
    }

    static std::pair<int, int> getTerminalSize() {
        struct winsize w{};
        if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &w) == -1) return {0, 0};
        return {w.ws_col, w.ws_row};
    }

   private:
    std::mutex term_mtx_;

    int fd_{STDIN_FILENO};
    bool use_dup_{false};
    bool raw_enabled_{false};
    struct termios orig_{}, raw_{};
};
