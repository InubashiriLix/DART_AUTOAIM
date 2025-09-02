CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
project(armor)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(
    include/
)

add_executable(armor
    src/camera/CamGeo.cc
    src/camera/CamWrapper.cpp
    src/main.cc
)

target_link_options(armor PRIVATE -no-pie)

target_link_libraries(armor
    ${OpenCV_LIBS}
    fmt::fmt
    -lglog
    -lgxiapi
)
