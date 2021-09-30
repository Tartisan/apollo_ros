cmake_minimum_required(VERSION 2.8.3)
project(perception)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

list(APPEND CMAKE_PREFIX_PATH "/usr/local/libtorch")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(${PROJECT_NAME}_CATKIN_DEPS
  roscpp
  pcl_ros
  sensor_msgs
  tf
  tf_conversions
  cv_bridge
)

find_package(catkin REQUIRED COMPONENTS
  ${${PROJECT_NAME}_CATKIN_DEPS}
)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CUDA REQUIRED)
find_package(Torch REQUIRED)
# find_package(absl REQUIRED)
# Resolve system dependency on yaml-cpp, which apparently does not provide 
# a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR NAMES yaml_cpp.h PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY NAMES YAML_CPP PATHS ${YAML_CPP_LIBRARIES})

set(MY_CUDA /usr/local/cuda/targets/x86_64-linux)
set(CUDA_INCLUDE_DIRS "${MY_CUDA}/include" ${CUDA_INCLUDE_DIRS})
set(CUDA_LIBRARIES ${CUDA_LIBRARIES} 
  "${MY_CUDA}/lib/libcudart.so" 
  "${MY_CUDA}/lib/stubs/libcuda.so" 
  "${MY_CUDA}/lib/libcublas.so"
  "${MY_CUDA}/lib/libcublas.so"
  "${MY_CUDA}/lib/libnppial.so"
  "${MY_CUDA}/lib/libnppicc.so"
  "${MY_CUDA}/lib/libnppicom.so"
  "${MY_CUDA}/lib/libnppidei.so"
  "${MY_CUDA}/lib/libnppif.so"
  "${MY_CUDA}/lib/libnppig.so"
  "${MY_CUDA}/lib/libnppim.so"
  "${MY_CUDA}/lib/libnppist.so"
  "${MY_CUDA}/lib/libnppisu.so"
  "${MY_CUDA}/lib/libnppitc.so")
set(CMAKE_CUDA_STANDARD 11)
link_directories(${MY_CUDA}/lib)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES perception
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
  # DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/../devel/include
  ${PROTOBUF_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

link_directories(
  # common/common_lib/glog/lib
  # ${CMAKE_SOURCE_DIR}/common/common_lib
  # ${CMAKE_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/../devel/lib/
)

add_subdirectory(proto)
add_subdirectory(base)
add_subdirectory(common)
add_subdirectory(lib)
add_subdirectory(inference)
add_subdirectory(camera)
# add_subdirectory(lidar)
# add_subdirectory(radar)

################# executable ################
add_executable(fusion_camera_detection 
  onboard/fusion_camera_detection_node.cc 
  onboard/fusion_camera_detection.cc
)
target_link_libraries(fusion_camera_detection 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${CUDA_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  perception_base
  perception_camera
  perception_common
  perception_inference
  perception_proto
  perception_lib
)
