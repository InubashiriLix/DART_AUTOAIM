#include "detector.hpp"

Detector::Detector() {
    this->center_x = config.center_x;
    this->center_y = config.center_y;
    welcom();
}

void Detector::welcom() {
    std::cout << "detector start" << std::endl;

    std::cout << "░▀█▀░█▀█░█░█░░░█▀▄░█▀▀░▀█▀░█▀▀░█▀▀░▀█▀░█▀█░█▀▄\n"
                 "░░█░░█░█░█░█░░░█░█░█▀▀░░█░░█▀▀░█░░░░█░░█░█░█▀▄\n"
                 "░▀▀▀░▀░▀░▀▀▀░░░▀▀░░▀▀▀░░▀░░▀▀▀░▀▀▀░░▀░░▀▀▀░▀░▀\n"
              << std::endl;
    std::cout << "============= configs ==============" << std ::endl;
    std::cout << "SHOW CV MONITOR: " << config.SHOW_CV_MONITOR_WINDOWS << std::endl;
    std::cout << "CENTER_X: " << config.center_x << std::endl;
    std::cout << "CENTER_Y: " << config.center_y << std::endl;
    std::cout << "=========== configs end ============" << std::endl;
}
