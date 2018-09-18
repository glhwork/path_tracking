cmake_minimum_required(VERSION 2.8.3)
project(path_tracking)

add_compile_options(-std=c++11)

include_directories(
  include
)

add_library(tracking_method
  src/PathTrack.cpp
)

add_executable(${PROJECT_NAME}_main src/main.cpp)
target_link_libraries(${PROJECT_NAME}_main
  tracking_method
)
