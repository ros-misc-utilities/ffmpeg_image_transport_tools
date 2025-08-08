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

#include "test_utils.hpp"

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <ffmpeg_image_transport_tools/bag_processor.hpp>
#include <iostream>
#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <stdexcept>

namespace test_utils
{
using Image = sensor_msgs::msg::Image;
rclcpp::Logger logger = rclcpp::get_logger("test_1");

static Image::SharedPtr makeImageMessage(
  uint8_t value, uint32_t w, uint32_t h, const std::string & encoding)
{
  auto img = std::make_shared<Image>();
  img->encoding = encoding;
  img->header.frame_id = "frame_id";
  img->header.stamp = rclcpp::Time(static_cast<int64_t>(value), RCL_ROS_TIME);
  img->height = h;
  img->width = w;
  img->is_bigendian = false;
  img->step = (sensor_msgs::image_encodings::bitDepth(img->encoding) / 8) * img->width *
              sensor_msgs::image_encodings::numChannels(img->encoding);
  img->data.resize(img->step * img->height, value);  // sets data!
  return (img);
}

void makeTestBag(
  const std::string & bag_name, size_t num_images, uint32_t width, uint32_t height,
  const std::vector<std::string> & topics, const std::string & encoding)
{
  test_utils::exec("rm -rf " + bag_name);
  auto writer = std::make_unique<rosbag2_cpp::Writer>();
  try {
    writer->open(bag_name);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(logger, "error opening output bag " << bag_name << ": " << e.what());
    throw(e);
  }
  const std::string topic_type = "sensor_msgs/msg/Image";
  for (const auto & topic : topics) {
    rosbag2_storage::TopicMetadata meta;
    meta.name = topic;
    meta.type = topic_type;
    meta.serialization_format = rmw_get_serialization_format();
    writer->create_topic(meta);
  }
  for (const auto & topic : topics) {
    for (size_t i = 0; i < num_images; i++) {
      auto smsg = std::make_shared<rclcpp::SerializedMessage>();
      rclcpp::Serialization<Image> serialization;
      auto m = makeImageMessage(static_cast<uint8_t>(i), width, height, encoding);
      serialization.serialize_message(m.get(), smsg.get());
      const rcutils_time_point_value_t t_recv = rclcpp::Time(m->header.stamp).nanoseconds() + 1;
#ifdef USE_NEW_ROSBAG_WRITE_INTERFACE
      const rcutils_time_point_value_t t_send = rclcpp::Time(m->header.stamp).nanoseconds() + 2;
      writer->write(smsg, topic, topic_type, t_recv, t_send);
#else
      writer->write(smsg, topic, topic_type, rclcpp::Time(t_recv));
#endif
    }
  }
  writer->close();
}

std::string exec(const std::string & cmd)
{
  std::array<char, 128> buffer;
  std::string result;
  std::unique_ptr<FILE, void (*)(FILE *)> pipe(popen(cmd.c_str(), "r"), [](FILE * f) -> void {
    // wrapper to ignore the return value from pclose() is needed with newer versions of gnu g++
    std::ignore = pclose(f);
  });
  if (!pipe) {
    throw std::runtime_error("popen() failed!");
  }
  while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr) {
    result += buffer.data();
  }
  return result;
}

class ImageReader : public ffmpeg_image_transport_tools::MessageProcessor<sensor_msgs::msg::Image>
{
public:
  const auto & getImages() const { return images_; }
  void process(
    rcutils_time_point_value_t, rcutils_time_point_value_t, const std::string & topic,
    const Image::ConstSharedPtr & m) final
  {
    auto it = images_.find(topic);
    if (it == images_.end()) {
      images_.insert({topic, std::vector<Image::ConstSharedPtr>()});
      it = images_.find(topic);
    }
    it->second.push_back(m);
  }

private:
  std::map<std::string, std::vector<Image::ConstSharedPtr>> images_;
};

TopicToImagesMap readImagesFromBag(
  const std::string & bag_name, const std::vector<std::string> & topics,
  const std::string & topic_type)
{
  bag_time_t start_time = std::numeric_limits<bag_time_t>::min();
  bag_time_t end_time = std::numeric_limits<bag_time_t>::max();

  ffmpeg_image_transport_tools::BagProcessor<sensor_msgs::msg::Image> bproc(
    logger, bag_name, topics, topic_type, start_time, end_time);
  ImageReader reader;
  bproc.process(&reader);
  return (reader.getImages());
}

static cv::Mat toCVMat(const Image::ConstSharedPtr & msg, const std::string & fmt)
{
  auto cv_img = cv_bridge::toCvShare(msg, fmt);
  if (!cv_img) {
    throw(cv_bridge::Exception("compress_bag:: cv_bridge cannot convert image"));
  }
  return (cv_img->image);
}

static bool compareImage(
  const sensor_msgs::msg::Image::ConstSharedPtr & img_orig,
  const sensor_msgs::msg::Image::ConstSharedPtr & img_uncomp)
{
  if (img_orig->height != img_uncomp->height || img_orig->width != img_uncomp->width) {
    RCLCPP_ERROR_STREAM(logger, "image size mismatch!");
    return false;
  }
  if (img_orig->encoding != img_uncomp->encoding) {
    RCLCPP_ERROR_STREAM(logger, "image encoding mismatch!");
    return false;
  }
  if (img_orig->data.size() != img_uncomp->data.size()) {
    RCLCPP_ERROR_STREAM(logger, "image data size mismatch!");
    return false;
  }
  auto img_original = toCVMat(img_orig, img_orig->encoding);
  auto img_decoded = toCVMat(img_uncomp, img_uncomp->encoding);

  if (img_original.channels() != img_decoded.channels()) {
    RCLCPP_ERROR_STREAM(
      logger, "number of channels changed: " << img_original.channels() << " -> "
                                             << img_decoded.channels());
    return (false);
  }
  if (img_original.rows != img_decoded.rows || img_original.cols != img_decoded.cols) {
    RCLCPP_ERROR_STREAM(
      logger, "decoded image size does not match. Decoded: "
                << img_decoded.cols << "x" << img_decoded.rows
                << " vs original: " << img_original.cols << "x" << img_original.rows);
    return (false);
  }
  cv::Mat diff;
  const int rows = std::min(img_original.rows, img_decoded.rows);
  const int cols = std::min(img_original.cols, img_decoded.cols);
  cv::Mat sub_decoded = img_decoded(cv::Rect(0, 0, cols, rows));
  cv::Mat sub_original = img_original(cv::Rect(0, 0, cols, rows));
  cv::absdiff(sub_original, sub_decoded, diff);
  cv::Scalar mean_diff = cv::mean(diff);
  double total_mean_diff = 0.0;
  for (int i = 0; i < img_original.channels(); i++) {
    total_mean_diff += mean_diff[i];
  }
  total_mean_diff /= img_original.channels();
  if (total_mean_diff > 0.0) {
    RCLCPP_ERROR_STREAM(
      logger, "mean difference between original and decoded image is " << total_mean_diff);
    return (false);
  }
  return true;
}

bool compareImages(
  const TopicToImagesMap & images_orig, const TopicToImagesMap & images_uncomp,
  const std::string & topic_suffix)
{
  if (images_orig.size() != images_uncomp.size()) {
    RCLCPP_ERROR(logger, "number of topics differ");
    return false;
  }
  for (const auto & topic : images_orig) {
    const auto it = images_uncomp.find(topic.first + topic_suffix);
    if (it == images_uncomp.end()) {
      RCLCPP_ERROR_STREAM(logger, "topic " << topic.first << " not found in uncompressed bag!");
      return false;
    }
    if (topic.second.size() != it->second.size()) {
      RCLCPP_ERROR_STREAM(
        logger, "number of messages for topic " << topic.first << " differ: " << topic.second.size()
                                                << " vs. " << it->second.size());
      return false;
    }
    // compare messages here
    for (size_t i = 0; i < topic.second.size(); i++) {
      const auto & img_orig = topic.second[i];
      const auto & img_uncomp = it->second[i];
      if (!compareImage(img_orig, img_uncomp)) {
        RCLCPP_ERROR_STREAM(
          logger, "image mismatch for topic " << topic.first << " at index " << i);
        return (false);
      }
    }
  }
  return true;
}

}  // namespace test_utils
