cmake_minimum_required(VERSION 2.8.3)
project(map)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(Eigen3 REQUIRED)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES perception
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
  # DEPENDS system_lib
)

include_directories(
  ${CMAKE_SOURCE_DIR}/
  ${CMAKE_SOURCE_DIR}/../devel/include/
  ${EIGEN3_INCLUDE_DIR}
)

add_subdirectory(proto)

add_library(hdmap_adapter
  # hdmap/adapter/
  hdmap/adapter/coordinate_convert_tool.cc
  hdmap/adapter/opendrive_adapter.cc
  hdmap/adapter/proto_organizer.cc
  # hdmap/adapter/xml_parser
  hdmap/adapter/xml_parser/header_xml_parser.cc
  hdmap/adapter/xml_parser/junctions_xml_parser.cc
  hdmap/adapter/xml_parser/lanes_xml_parser.cc
  hdmap/adapter/xml_parser/objects_xml_parser.cc
  hdmap/adapter/xml_parser/roads_xml_parser.cc
  hdmap/adapter/xml_parser/signals_xml_parser.cc
  hdmap/adapter/xml_parser/util_xml_parser.cc
)

add_library(map
  # hdmap
  hdmap/hdmap_common.cc
  hdmap/hdmap_impl.cc
  hdmap/hdmap_util.cc
  hdmap/hdmap.cc
)
target_link_libraries(map hdmap_adapter)