cmake_minimum_required(VERSION 2.8.3)
project(sensor_visualizer)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  std_msgs
  drivers
)

catkin_package(
  CATKIN_DEPENDS roscpp
  DEPENDS roscpp std_msgs drivers
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/../devel/include/
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

link_directories(
  ${CMAKE_SOURCE_DIR}
  ${PCL_LIBRARY_DIRS}
  ${CMAKE_SOURCE_DIR}/../devel/lib/
)

add_executable(viz_conti_radar 
  viz_conti_radar.cpp 
)
target_link_libraries(viz_conti_radar
  ${catkin_LIBRARIES}
)
add_dependencies(viz_conti_radar
  drivers_generate_messages
)
