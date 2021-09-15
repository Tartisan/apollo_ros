
add_library(perception_lib
  config_manager/config_manager.cc
  registerer/registerer.cc
  thread/thread_pool.cc
  thread/thread_worker.cc
  thread/thread.cc
  utils/timer.cc
)
target_link_libraries(perception_lib 
  ${YAML_CPP_LIBRARIES}
  cyber
  perception_base
)
