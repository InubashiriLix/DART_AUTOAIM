#include <iostream>
#include <string>
#include <toml.hpp>

struct detector_config {
    bool SHOW_CV_MONITOR_WINDOWS = false;
    int camera_qos_keep_last;
    int center_x;
    int center_y;

    detector_config(
        const std::string toml_abs_path_str = "/home/orangepi/08_DART_AUTOAIM/ros_ws/config.toml") {
        auto config = toml::parse_file(toml_abs_path_str);

        try {
            const auto* detector = config["detector"].as_table();
            if (!detector) throw std::runtime_error("missing [detector]");
            SHOW_CV_MONITOR_WINDOWS = (*detector)["SHOW_CV_MONITOR_WINDOWS"].value_or(false);

            camera_qos_keep_last = (*detector)["camera_qos_keep_last"].value_or(3);

            center_x = (*detector)["center_x"].value_or(512);
            center_y = (*detector)["center_y"].value_or(384);
        } catch (const std::out_of_range& e) {
            // Handle missing key
            std::cerr << "Missing key: " << e.what() << std::endl;
            throw;
        } catch (const std::runtime_error& e) {
            // Handle wrong type
            std::cerr << "Type error: " << e.what() << std::endl;
            throw;
        }
    }
};
