#include "camera/CamNode.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CameraPublisher>(argc, argv);
    if (!node->start()) {
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    node->stop();
    rclcpp::shutdown();
    return 0;
}
