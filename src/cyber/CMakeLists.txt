cmake_minimum_required(VERSION 2.8.3)
project(cyber C ASM)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS roscpp)

include_directories(
  ${CMAKE_SOURCE_DIR}/
  ${CMAKE_SOURCE_DIR}/../devel/include/
)

add_subdirectory(proto)

if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
  set_property(SOURCE croutine/detail/swap_x86_64.S APPEND PROPERTY COMPILE_OPTIONS "-x" "assembler-with-cpp")
  set(SWAP_CPP croutine/detail/swap_x86_64.S)
else()
  set_property(SOURCE croutine/detail/swap_aarch64.S APPEND PROPERTY COMPILE_OPTIONS "-x" "assembler-with-cpp")
  set(SWAP_CPP croutine/detail/swap_aarch64.S)
endif()

add_library(${PROJECT_NAME} 
  binary.cc
  init.cc
  state.cc
  # common
  common/file.cc
  common/global_data.cc
  # croutine
  ${SWAP_CPP}
  croutine/detail/routine_context.cc
  croutine/croutine.cc
  # logger
  logger/async_logger.cc
  logger/log_file_object.cc
  logger/logger_util.cc
  logger/logger.cc
  # message
  message/protobuf_factory.cc
  # record
  record/file/record_file_base.cc
  record/file/record_file_reader.cc
  record/file/record_file_writer.cc
  record/header_builder.cc
  record/record_reader.cc
  record/record_viewer.cc
  record/record_writer.cc
  # scheduler
  scheduler/common/pin_thread.cc
  scheduler/policy/choreography_context.cc
  scheduler/policy/classic_context.cc
  scheduler/policy/scheduler_choreography.cc
  scheduler/policy/scheduler_classic.cc
  scheduler/processor_context.cc
  scheduler/processor.cc
  scheduler/scheduler_factory.cc
  scheduler/scheduler.cc
  # task
  task/task_manager.cc
  # time 
  time/clock.cc
  time/duration.cc
  time/rate.cc
  time/time.cc
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_proto
  glog
  atomic
)
