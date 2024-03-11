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


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

set(ament_dependencies
  "rclcpp"
  "rosbag2_cpp"
  "cv_bridge"
  "ffmpeg_image_transport_msgs"
  "ffmpeg_image_transport"
  "sensor_msgs")

foreach(pkg ${ament_dependencies})
  find_package(${pkg} REQUIRED)
endforeach()

#
# find libav because the dependencies exported by ffmpeg_image_transport are
# screwed up.
#
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBAV IMPORTED_TARGET
    libavdevice
    libavfilter
    libavformat
    libavcodec
    libswresample
    libswscale
    libavutil
)


#
# -------- bag_to_file
#
add_executable(bag_to_file  src/bag_to_file.cpp)
ament_target_dependencies(bag_to_file ${ament_dependencies})
target_compile_features(bag_to_file PRIVATE cxx_std_17)

#
# -------- bag_to_frames
#
add_executable(bag_to_frames  src/bag_to_frames.cpp)
ament_target_dependencies(bag_to_frames ${ament_dependencies})
target_compile_features(bag_to_frames PRIVATE cxx_std_17)
target_include_directories(bag_to_frames PRIVATE include)

install(TARGETS
  bag_to_file
  bag_to_frames
  DESTINATION lib/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
#   find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
#   find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_copyright()
  ament_cppcheck(LANGUAGE c++)
  ament_cpplint(FILTERS "-build/include,-runtime/indentation_namespace")
  ament_clang_format(CONFIG_FILE .clang-format)
#  ament_flake8()
  ament_lint_cmake()
#  ament_pep257()
  ament_xmllint()
endif()

ament_package()