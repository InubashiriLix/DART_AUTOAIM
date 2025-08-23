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

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

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

add_executable(aim_algo
    src/camera/CamWrapper.cpp
    src/camera/CamNode.cc
    src/camera/CamGeo.cc
    src/detector.cc
    src/cv_utils.cc
    src/kalman.cc
    src/tracker.cc
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
    Eigen3::Eigen
    -lglog
    -lgxiapi
)

install(
    TARGETS
    aim_algo
    DESTINATION lib/${PROJECT_NAME}
)


ament_package()
