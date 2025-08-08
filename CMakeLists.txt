#
# Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.22)
project(ffmpeg_image_transport_tools)


add_compile_options(-Wall -Wextra -Wpedantic)
if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.24.0")
  option(CMAKE_COMPILE_WARNING_AS_ERROR "Treat compiler warnings as errors." ON)
  mark_as_advanced(CMAKE_COMPILE_WARNING_AS_ERROR)
else()
  add_compile_options(-Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ffmpeg_encoder_decoder REQUIRED)
find_package(ffmpeg_image_transport_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED core imgproc)

if(${rosbag2_cpp_VERSION} VERSION_GREATER_EQUAL "0.26.0")
  add_definitions(-DUSE_NEW_ROSBAG_WRITE_INTERFACE)
endif()

if(${cv_bridge_VERSION} VERSION_GREATER "3.3.0")
  add_definitions(-DUSE_CV_BRIDGE_HPP)
endif()

if(${rosbag2_storage_VERSION} VERSION_GREATER_EQUAL "0.26.1")
  add_definitions(-DUSE_ROSBAG2_STORAGE_RECV_TIME)
endif()


#
# -------- bag_to_file
#
add_executable(bag_to_file  src/bag_to_file.cpp)
target_link_libraries(bag_to_file
  cv_bridge::cv_bridge
  ffmpeg_encoder_decoder::ffmpeg_encoder_decoder
  ${ffmpeg_image_transport_msgs_TARGETS}
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage
  ${sensor_msgs_TARGETS})

target_compile_features(bag_to_file PRIVATE cxx_std_17)
target_include_directories(bag_to_file PRIVATE include)

#
# -------- bag_to_frames
#
add_executable(bag_to_frames  src/bag_to_frames.cpp)
target_link_libraries(bag_to_frames
  cv_bridge::cv_bridge
  ffmpeg_encoder_decoder::ffmpeg_encoder_decoder
  ${ffmpeg_image_transport_msgs_TARGETS}
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage
  opencv_core opencv_imgproc
  ${sensor_msgs_TARGETS})

target_compile_features(bag_to_frames PRIVATE cxx_std_17)
target_include_directories(bag_to_frames PRIVATE include)

#
# -------- compress_bag
#
add_executable(compress_bag  src/compress_bag.cpp)
target_link_libraries(compress_bag
  cv_bridge::cv_bridge
  ffmpeg_encoder_decoder::ffmpeg_encoder_decoder
  ${ffmpeg_image_transport_msgs_TARGETS}
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage
  opencv_core opencv_imgproc
  ${sensor_msgs_TARGETS})
target_compile_features(compress_bag PRIVATE cxx_std_17)
target_include_directories(compress_bag PRIVATE include)

#
# -------- uncompress_bag
#
add_executable(uncompress_bag  src/uncompress_bag.cpp)
target_link_libraries(uncompress_bag
  cv_bridge::cv_bridge
  ffmpeg_encoder_decoder::ffmpeg_encoder_decoder
  ${ffmpeg_image_transport_msgs_TARGETS}
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage
  ${sensor_msgs_TARGETS})
target_compile_features(uncompress_bag PRIVATE cxx_std_17)
target_include_directories(uncompress_bag PRIVATE include)


install(TARGETS
  bag_to_file
  bag_to_frames
  compress_bag
  uncompress_bag
  DESTINATION lib/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_black REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test test/test_1.cpp test/test_utils.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(${PROJECT_NAME}_test
    cv_bridge::cv_bridge
    ${sensor_msgs_TARGETS}
    rclcpp::rclcpp
    rosbag2_cpp::rosbag2_cpp
    rosbag2_storage::rosbag2_storage)
  ament_copyright()
  ament_cppcheck(LANGUAGE c++)
  ament_cpplint(FILTERS "-build/include,-runtime/indentation_namespace")
  ament_clang_format(CONFIG_FILE .clang-format)
  ament_black()
  ament_lint_cmake()
  ament_xmllint()
endif()

ament_package()
