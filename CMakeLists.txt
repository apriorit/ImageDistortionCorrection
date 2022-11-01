cmake_minimum_required(VERSION 3.5)

project(ImageDistortion LANGUAGES CXX)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(ImageDistortion main.cpp)

target_link_libraries(ImageDistortion ${OpenCV_LIBS})
