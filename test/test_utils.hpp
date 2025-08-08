// -*-c++-*--------------------------------------------------------------------
// Copyright 2025 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef TEST_UTILS_HPP_
#define TEST_UTILS_HPP_
#include <cstdint>
#include <map>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <vector>

namespace test_utils
{
using Image = sensor_msgs::msg::Image;
using TopicToImagesMap = std::map<std::string, std::vector<Image::ConstSharedPtr>>;

void makeTestBag(
  const std::string & bag_name, size_t num_images, uint32_t width, uint32_t height,
  const std::vector<std::string> & topics, const std::string & encoding);
std::string exec(const std::string & cmd);
TopicToImagesMap readImagesFromBag(
  const std::string & bag_name, const std::vector<std::string> & topics,
  const std::string & topic_type);
bool compareImages(
  const TopicToImagesMap & images_orig, const TopicToImagesMap & images_uncomp,
  const std::string & topic_suffix);
}  // namespace test_utils
#endif  // TEST_UTILS_HPP_
