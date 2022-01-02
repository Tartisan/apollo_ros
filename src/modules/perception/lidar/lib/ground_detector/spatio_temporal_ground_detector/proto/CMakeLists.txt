find_package(Protobuf REQUIRED)

set(pb_out_path ${CMAKE_SOURCE_DIR}/../devel/include/modules/perception/lidar/lib/ground_detector/spatio_temporal_ground_detector/proto) # 设置 .proto 生成文件的输出目录
file(MAKE_DIRECTORY ${pb_out_path})

find_program(PROTOC protoc) # 寻找 protoc
set(pb_file_root ${CMAKE_CURRENT_SOURCE_DIR}) # 设置 .proto 文件的根目录

function(generate_cpp_pb_file pb_path)
  set(pb_file_path ${pb_file_root}/${pb_path}) # 设置 .proto 实际所在的目录
  file(GLOB proto_files ${pb_file_path}/*.proto) # 找出所有的 .proto
  
  foreach(pb_file ${proto_files})
    get_filename_component(pb_file_name ${pb_file} NAME_WE) # 获取 .proto 文件名
    set(cur_out_file # 设置输出文件名
      ${pb_out_path}/${pb_path}/${pb_file_name}.pb.h
      ${pb_out_path}/${pb_path}/${pb_file_name}.pb.cc
    )
    set(pb_out_files ${pb_out_files} ${cur_out_file})

    add_custom_command(
      OUTPUT ${cur_out_file}
      COMMAND ${PROTOC} ${pb_file} --cpp_out=${CMAKE_SOURCE_DIR}/../devel/include --proto_path=${CMAKE_SOURCE_DIR} -I=./
      DEPENDS ${PROTOC} ${pb_file}
      WORKING_DIRECTORY ../
      # ${CMAKE_CURRENT_SOURCE_DIR}
    )
  endforeach(pb_file ${proto_files})

  set_source_files_properties(${pb_out_files} PROPERTIES GENERATED TRUE)

  link_directories(${CMAKE_SOURCE_DIR}/../devel/lib)
  add_library(perception_lidar_lib_ground_detector_spatio_temporal_ground_detector_proto
    ${pb_out_files}
  )
  target_link_libraries(perception_lidar_lib_ground_detector_spatio_temporal_ground_detector_proto
    ${catkin_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
  )
endfunction()

# set(proto_dirs ) # 添加 .proto 文件所在的相对路径，即 proto 文件夹下的各个子文件夹
# foreach(pb_dir ${proto_dirs})
#   generate_cpp_pb_file(${pb_dir})
# endforeach()
generate_cpp_pb_file("")