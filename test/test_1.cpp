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

#include <gtest/gtest.h>

#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "test_utils.hpp"

static std::vector<std::string> append(
  const std::vector<std::string> & v, const std::string & suffix)
{
  std::vector<std::string> res;
  for (const auto & s : v) {
    res.push_back(s + suffix);
  }
  return res;
}

static std::string make_commands(const std::vector<std::string> & topics)
{
  std::string cmds;
  for (const auto & topic : topics) {
    cmds += " -t " + topic;
  }
  return cmds;
}

TEST(ffmpeg_image_transport_tools, test_1)
{
  rclcpp::init(0, nullptr);
  const std::string test_bag = "test_bag";
  const std::string comp_bag = "comp_bag";
  const std::string uncomp_bag = "uncomp_bag";
  const std::string opt_c = " -E encoder:libx264rgb -E crf:0";
  const std::vector<std::string> topics = {"/camera_0/image_raw", "/camera_1/image_raw"};
  const auto comp_topics = append(topics, "/ffmpeg");
  const auto uncomp_topics = append(comp_topics, "/decoded");
  const std::string cmd_base = "ros2 run ffmpeg_image_transport_tools ";
  const auto cmd_topics = make_commands(topics);
  const auto cmd_topics_c = make_commands(comp_topics);

  // create bag with test images
  test_utils::makeTestBag(test_bag, 10, 640, 480, topics, "bgr8");
  test_utils::exec("rm -rf " + comp_bag);
  // compress the bag
  const auto cmd_c =
    cmd_base + "compress_bag " + cmd_topics + " -i " + test_bag + " -o " + comp_bag + opt_c;
  int status = std::system((cmd_c + " > compress.txt").c_str());
  EXPECT_EQ(status, 0) << "compress_bag command failed: " << cmd_c;
  // uncompress the bag
  test_utils::exec("rm -rf " + uncomp_bag);
  const auto cmd_u =
    cmd_base + "uncompress_bag " + cmd_topics_c + " -i " + comp_bag + " -o " + uncomp_bag;
  status = std::system((cmd_u + " > uncompress.txt").c_str());
  EXPECT_EQ(status, 0) << "uncompress_bag command failed: " << cmd_u;

  const auto images_orig = test_utils::readImagesFromBag(test_bag, topics, "sensor_msgs/msg/Image");
  const auto images_uncomp =
    test_utils::readImagesFromBag(uncomp_bag, uncomp_topics, "sensor_msgs/msg/Image");
  const bool img_eq = test_utils::compareImages(images_orig, images_uncomp, "/ffmpeg/decoded");
  EXPECT_TRUE(img_eq) << "Images in original and uncompressed bag do not match!";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
