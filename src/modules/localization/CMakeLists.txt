cmake_minimum_required(VERSION 2.8.3)
project(localization)

find_package(catkin REQUIRED COMPONENTS roscpp)

include_directories(
  ${CMAKE_SOURCE_DIR}/
  ${CMAKE_SOURCE_DIR}/../devel/include/
)

add_subdirectory(proto)

