cmake_minimum_required(VERSION 3.10)

project(canvas_demo)

set(CMAKE_CXX_STANDARD 20)

add_executable(canvas_demo main.cpp pystring.cpp glm-aabb/aabb.cpp simplifyPath.cpp)

target_include_directories(canvas_demo PRIVATE ${CMAKE_SOURCE_DIR}include "D:/libs/glm")