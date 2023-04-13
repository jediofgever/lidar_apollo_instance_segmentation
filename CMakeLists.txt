cmake_minimum_required(VERSION 3.8)
project(lidar_apollo_instance_segmentation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cuda_utils REQUIRED)
find_package(tensorrt_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(vox_nav_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

set(dependencies
cuda_utils
tensorrt_common
rclcpp
rclcpp_components
pcl_ros
pcl_conversions
vox_nav_msgs
tf2_eigen
)

include_directories(include)

if(NOT ${tensorrt_common_FOUND})
  message(WARNING "The tensorrt_common package is not found. Please check its dependencies.")
  # to avoid launch file missing without a gpu
  ament_package()
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
  return()
endif()

set(DATA_PATH "${CMAKE_CURRENT_SOURCE_DIR}/data")
if(NOT EXISTS "${DATA_PATH}")
  execute_process(COMMAND mkdir -p ${DATA_PATH})
endif()

function(download FILE_NAME FILE_HASH)
  message(STATUS "Checking and downloading ${FILE_NAME}")
  set(FILE_PATH ${DATA_PATH}/${FILE_NAME})
  set(STATUS_CODE 0)
  message(STATUS "start ${FILE_NAME}")
  if(EXISTS ${FILE_PATH})
    message(STATUS "found ${FILE_NAME}")
    file(MD5 ${FILE_PATH} EXISTING_FILE_HASH)
    if(${FILE_HASH} STREQUAL ${EXISTING_FILE_HASH})
      message(STATUS "same ${FILE_NAME}")
      message(STATUS "File already exists.")
    else()
      message(STATUS "diff ${FILE_NAME}")
      message(STATUS "File hash changes. Downloading now ...")
      file(DOWNLOAD
        https://awf.ml.dev.web.auto/perception/models/lidar_apollo_instance_segmentation/${FILE_NAME} ${FILE_PATH}
        STATUS DOWNLOAD_STATUS TIMEOUT 300
      )
      list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
      list(GET DOWNLOAD_STATUS 1 ERROR_MESSAGE)
    endif()
  else()
    message(STATUS "not found ${FILE_NAME}")
    message(STATUS "File doesn't exists. Downloading now ...")
    file(DOWNLOAD
      https://awf.ml.dev.web.auto/perception/models/lidar_apollo_instance_segmentation/${FILE_NAME} ${FILE_PATH}
      STATUS DOWNLOAD_STATUS TIMEOUT 300
    )
    list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
    list(GET DOWNLOAD_STATUS 1 ERROR_MESSAGE)
  endif()
  if(${STATUS_CODE} EQUAL 0)
    message(STATUS "Download completed successfully!")
  else()
    message(FATAL_ERROR "Error occurred during download: ${ERROR_MESSAGE}")
  endif()
endfunction()

download(vlp-16.onnx 63a5a1bb73f7dbb64cf70d04eca45fb4)
download(hdl-64.onnx 009745e33b1df44b68296431cc384cd2)
download(vls-128.onnx b2d709f56f73ae2518c9bf4d0214468f)

add_library(${PROJECT_NAME} SHARED
  src/node.cpp
  src/detector.cpp
  src/log_table.cpp
  src/feature_generator.cpp
  src/feature_map.cpp
  src/cluster2d.cpp
  src/debugger.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

#rclcpp_components_register_node(${PROJECT_NAME}
#  PLUGIN "lidar_apollo_instance_segmentation::LidarInstanceSegmentationNode"
#  EXECUTABLE ${PROJECT_NAME}_node
#)

add_executable(${PROJECT_NAME}_node src/node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node ${dependencies})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

ament_package()

install(
  DIRECTORY
    config
    data
    launch
  DESTINATION share/${PROJECT_NAME}
)