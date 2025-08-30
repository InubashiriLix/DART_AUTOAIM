#include <fcntl.h>

#include "Terminal.hpp"
#include "Window.hpp"

int main() {
    Terminal term;
    term.clear();

    // Window win(term, "Command", 2, 5, 50, 12);
    // win.setDisplayMap({
    //     {"help", "show help info"},
    //     {
    //         "quit",
    //         "exit program",
    //     },
    //     {"version", "show version info"},
    // });
    // win.render();
    //
    // for (;;) {
    //     win.render();
    //     sleep(1);
    // }
    //
    while (1) {
        auto ch = term.nonblocking_input();
        std::cout << char(ch) << std::endl;
    }

    return 0;
}
