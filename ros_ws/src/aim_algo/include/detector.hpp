#include <iostream>
#include <opencv2/opencv.hpp>

#include "config_parser.hpp"

class Detector final {
   public:
    Detector();

    int start();
    int run();
    int stop();

   private:
    int center_x;
    int center_y;

    void welcom();
    detector_config config;
};
