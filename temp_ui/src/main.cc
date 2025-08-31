#include <fcntl.h>
#include <sys/ioctl.h>

#include "Terminal.hpp"
#include "Tui.hpp"
#include "Window.hpp"

int main() {
    Terminal term;
    term.clear();

    Tui tui(term);
    tui.prepareWindows();
    tui.start();
    while (tui.isRunning()) std::this_thread::sleep_for(std::chrono::milliseconds(50));
}
