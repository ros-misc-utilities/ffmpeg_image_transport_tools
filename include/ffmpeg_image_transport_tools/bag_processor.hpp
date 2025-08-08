// -*-c++-*--------------------------------------------------------------------
// Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#ifndef FFMPEG_IMAGE_TRANSPORT_TOOLS__BAG_PROCESSOR_HPP_
#define FFMPEG_IMAGE_TRANSPORT_TOOLS__BAG_PROCESSOR_HPP_

#include <chrono>
#include <ffmpeg_image_transport_tools/message_processor.hpp>
#include <functional>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <set>
#include <vector>

using rclcpp::Time;
using bag_time_t = rcutils_time_point_value_t;

namespace ffmpeg_image_transport_tools
{
template <typename T>
class BagProcessor
{
public:
  BagProcessor(
    rclcpp::Logger logger, const std::string & bag_name, const std::vector<std::string> & topics,
    const std::string & topic_type, bag_time_t start_time, bag_time_t end_time)
  : logger_(logger), start_time_(start_time), end_time_(end_time)
  {
    RCLCPP_INFO_STREAM(logger, "opening bag: " << bag_name);
    reader_.open(bag_name);
    const auto meta = reader_.get_all_topics_and_types();
    for (const auto & topic : topics) {
      topics_.insert(topic);
      const auto it = std::find_if(
        meta.begin(), meta.end(), [topic, topic_type](const rosbag2_storage::TopicMetadata & m) {
          return (m.name == topic && m.type == topic_type);
        });
      if (it == meta.end()) {
        RCLCPP_ERROR_STREAM(
          logger_, "topic " << topic << " with type " << topic_type << " not found!");
        throw(std::runtime_error("topic not found!"));
      }
      RCLCPP_INFO_STREAM(logger_, "found topic: " << it->name << " of type: " << it->type);
      filter_.topics.push_back(topic);
    }
    reader_.set_filter(filter_);
  }

  size_t process(MessageProcessor<T> * mp)
  {
    const auto start = std::chrono::high_resolution_clock::now();
    if (start_time_ != std::numeric_limits<bag_time_t>::min()) {
      RCLCPP_INFO_STREAM(logger_, "seeking for start time: " << start_time_);
      reader_.seek(start_time_);
    }
    size_t message_number{0};

    while (reader_.has_next()) {
      auto msg = reader_.read_next();
      if (!msg || (topics_.find(msg->topic_name) == topics_.end())) {
        continue;
      }
      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
      typename T::SharedPtr m(new T());
      serialization_.deserialize_message(&serialized_msg, m.get());
#ifdef USE_ROSBAG2_STORAGE_RECV_TIME
      const auto t = Time(msg->recv_timestamp).nanoseconds();
#else
      const auto t = Time(msg->time_stamp).nanoseconds();
#endif
      if (t > end_time_) {
        break;
      }
#ifdef USE_ROSBAG2_STORAGE_RECV_TIME
      mp->process(msg->recv_timestamp, msg->send_timestamp, msg->topic_name, m);
#else
      mp->process(msg->time_stamp, msg->time_stamp, msg->topic_name, m);
#endif
      message_number++;
    }
    const auto stop = std::chrono::high_resolution_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    return (message_number);
  }

private:
  rclcpp::Logger logger_;
  rosbag2_cpp::Reader reader_;
  rosbag2_storage::StorageFilter filter_;
  std::set<std::string> topics_;
  bag_time_t start_time_;
  bag_time_t end_time_;
  rclcpp::Serialization<T> serialization_;
};
}  // namespace ffmpeg_image_transport_tools

#endif  // FFMPEG_IMAGE_TRANSPORT_TOOLS__BAG_PROCESSOR_HPP_
