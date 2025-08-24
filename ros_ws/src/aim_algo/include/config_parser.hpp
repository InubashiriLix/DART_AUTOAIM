#pragma once

#include <iostream>
#include <string>
#include <toml.hpp>

struct camera_config {
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

    bool publish_image_msg = true;
    bool publish_camera_info = true;

    int avg_frame_delay_num = 300;

    camera_config(
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
            avg_frame_delay_num = static_cast<double>(
                (*cam)["avg_frame_delay_num"].value_or(int{avg_frame_delay_num}));

            publish_image_msg = (*cam)["publish_image_msg"].value_or(publish_image_msg);
            publish_camera_info = (*cam)["publish_camera_info"].value_or(publish_camera_info);

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

struct detector_config {
    bool SHOW_CV_MONITOR_WINDOWS = false;
    int camera_qos_keep_last;
    int center_x;
    int center_y;
    bool SHOW_CV_CAL_DELAY;
    size_t avg_frame_delay_num = 300;
    bool SHOW_TARGET_ANGLE = false;

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

            SHOW_CV_CAL_DELAY = (*detector)["SHOW_CV_CAL_DELAY"].value_or(false);
            avg_frame_delay_num = (*detector)["avg_frame_delay_num"].value_or(300);
            SHOW_TARGET_ANGLE = (*detector)["SHOW_TARGET_ANGLE"].value_or(false);

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
