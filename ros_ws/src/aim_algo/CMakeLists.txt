cmake_minimum_required(VERSION 3.8)
project(aim_algo)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
# cus dependencies
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

include_directories(${OpenCV_INCLUDE_DIRS})

add_subdirectory(serial/lib)

add_executable(aim_algo
    # dahen camera support
    src/camera/CamWrapper.cpp
    src/camera/CamNode.cc
    # dahen camera pixel 2 angle and other tools
    src/camera/CamGeo.cc
    # autoaim detector
    src/detector.cc
    src/cv_utils.cc
    # kalman with delay awareness
    src/kalman.cc
    # contact
    src/contact/Checksum.cc
    src/contact/CommPort.cc
    src/contact/Contact.cc
    # main entrance
    src/main.cc
)

ament_target_dependencies(aim_algo
    rclcpp
    std_msgs
    sensor_msgs
    cv_bridge
)

include_directories(
    include
)

target_link_libraries(aim_algo
    ${OpenCV_LIBS}
    serial
    Eigen3::Eigen
    spdlog::spdlog
    fmt::fmt
    -lglog
    -lgxiapi
)

install(
    TARGETS
    aim_algo
    DESTINATION lib/${PROJECT_NAME}
)


ament_package()
