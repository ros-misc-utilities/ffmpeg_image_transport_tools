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

using rclcpp::Time;
using bag_time_t = rcutils_time_point_value_t;

namespace ffmpeg_image_transport_tools
{
template <typename T>
class BagProcessor
{
public:
  BagProcessor(
    const std::string & bag_name, const std::string & topic, const std::string & topic_type,
    bag_time_t start_time, bag_time_t end_time)
  : topic_(topic), start_time_(start_time), end_time_(end_time)
  {
    std::cout << "opening bag: " << bag_name << std::endl;
    std::cout << "topic:       " << topic << std::endl;
    reader_.open(bag_name);
    const auto meta = reader_.get_all_topics_and_types();
    const auto it = std::find_if(
      meta.begin(), meta.end(), [topic, topic_type](const rosbag2_storage::TopicMetadata & m) {
        return (m.name == topic && m.type == topic_type);
      });
    if (it == meta.end()) {
      std::cerr << "topic " << topic << " with type " << topic_type << " not found!" << std::endl;
      throw(std::runtime_error("topic not found!"));
    }

    filter_.topics.push_back(topic);
    reader_.set_filter(filter_);
  }

  size_t process(MessageProcessor<T> * mp)
  {
    const auto start = std::chrono::high_resolution_clock::now();
    if (start_time_ != std::numeric_limits<bag_time_t>::min()) {
      std::cout << "seeking for start time: " << start_time_ << std::endl;
      reader_.seek(start_time_);
    }
    size_t message_number{0};

    while (reader_.has_next()) {
      auto msg = reader_.read_next();
      if (!msg || topic_ != msg->topic_name) {
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
      mp->process(t, m);
      message_number++;
    }
    const auto stop = std::chrono::high_resolution_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "messages processed: " << message_number << std::endl;
    std::cout << "total time for processing: " << total_duration.count() * 1e-6 << std::endl;
    return (message_number);
  }

private:
  rosbag2_cpp::Reader reader_;
  rosbag2_storage::StorageFilter filter_;
  std::string topic_;
  bag_time_t start_time_;
  bag_time_t end_time_;
  rclcpp::Serialization<T> serialization_;
};
}  // namespace ffmpeg_image_transport_tools

#endif  // FFMPEG_IMAGE_TRANSPORT_TOOLS__BAG_PROCESSOR_HPP_
