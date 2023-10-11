cmake_minimum_required(VERSION 3.10)
project(ME_491_PI)

set(CMAKE_CXX_STANDARD 20)
include(helper/helper.cmake)
create_executable(ME_491_PI_TUTORIAL src/main.cpp)
target_include_directories(ME_491_PI_TUTORIAL PUBLIC src/student_id.hpp)
