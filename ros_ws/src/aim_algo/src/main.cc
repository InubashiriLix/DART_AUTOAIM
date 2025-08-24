#include "camera/CamNode.hpp"
#include "detector.hpp"
#include "utils/logging.hpp"

int main(int argc, char **argv) {
    // initialize the log system
    perflog::init({{"detector", "logs/detector.log"},
                   {"cam", "logs/cam.log"},
                   {"kalman", "logs/kalman.log"},
                   {"commu", "logs/commu.log"}});

    rclcpp::init(argc, argv);

    auto cam_node = std::make_shared<CameraPublisher>(argc, argv);
    std::cout << "Camera Node Start" << std::endl;
    if (!cam_node->start()) {
        std::cerr << "!!! cam_node start failed !!!" << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    auto detector_node = std::make_shared<Detector>(cam_node);
    std::cout << "Detector Node Start" << std::endl;
    if (!detector_node->start()) {
        std::cerr << "!!! detector_node start failed !!!" << std::endl;
        RCLCPP_ERROR(detector_node->get_logger(), "Failed to start detector node");
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(cam_node);
    std::cout << "cam node init done" << std::endl;
    exec.add_node(detector_node);
    std::cout << "detector node init done" << std::endl;

    exec.spin();

    cam_node->stop();
    detector_node->stop();
    rclcpp::shutdown();
    return 0;
}
