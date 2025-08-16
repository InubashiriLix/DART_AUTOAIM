cmake_minimum_required(VERSION 3.5)
project(camera)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

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

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(camera
        src/CamWrapper_node.cpp
        src/CamWrapper.cpp
)

add_executable(camera1
        src/CamWrapper_node_.cpp
        src/CamWrapper.cpp
)

target_link_libraries(
        camera
        ${OpenCV_LIBRARIES}
        -lglog
        -lgxiapi
)

target_link_libraries(
        camera1
        ${OpenCV_LIBRARIES}
        -lglog
        -lgxiapi
)

ament_target_dependencies(
        camera
        rclcpp
        std_msgs
        sensor_msgs
        cv_bridge
)

ament_target_dependencies(
        camera1
        rclcpp
        std_msgs
        sensor_msgs
        cv_bridge
)

install(
  TARGETS 
  camera
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS 
  camera1
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()























