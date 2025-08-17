#include <iostream>
#include <string>
#include <vector>

#include "toml.hpp"

struct toml_config {
    bool IS_ROTATE = true;
    bool FOR_PC = true;
    bool SHOW_CV_MONITOR_WINDOWS = false;
    std::vector<int> MONITOR_IMG_GAIN;
    std::string SN = "";
    int ROI_width = 1024;
    int ROI_height = 768;
    int sensor_width = 1280;
    int sensor_height = 1024;
    int nBinning = 1;
    float FPS = 100.0;

    toml_config(
        const std::string& sub_table_name = "camera1",
        const std::string& toml_abs_path = "/home/orangepi/08_DART_AUTOAIM/ros_ws/config.toml") {
        auto config = toml::parse_file(toml_abs_path);

        try {
            const auto* cam = config[sub_table_name].as_table();
            if (!cam) throw std::runtime_error("missing [" + sub_table_name + "]");

            IS_ROTATE = (*cam)["IS_ROTATE"].value_or(IS_ROTATE);
            FOR_PC = (*cam)["FOR_PC"].value_or(FOR_PC);
            SHOW_CV_MONITOR_WINDOWS =
                (*cam)["SHOW_CV_MONITOR_WINDOWS"].value_or(SHOW_CV_MONITOR_WINDOWS);
            if (const auto* arr = (*cam)["MONITOR_IMG_GAIN"].as_array()) {
                MONITOR_IMG_GAIN.clear();
                for (const auto& v : *arr) {
                    if (v.is_integer()) MONITOR_IMG_GAIN.push_back(int(v.value<int64_t>().value()));
                }
            }

            SN = (*cam)["SN"].value_or(SN);
            ROI_width = static_cast<int>((*cam)["ROI_width"].value_or(int64_t{ROI_width}));
            ROI_height = static_cast<int>((*cam)["ROI_height"].value_or(int64_t{ROI_height}));
            sensor_width = static_cast<int>((*cam)["sensor_width"].value_or(int64_t{sensor_width}));
            sensor_height =
                static_cast<int>((*cam)["sensor_height"].value_or(int64_t{sensor_height}));
            nBinning = static_cast<int>((*cam)["nBinning"].value_or(int64_t{nBinning}));
            FPS = static_cast<float>((*cam)["FPS"].value_or(double{FPS}));
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
