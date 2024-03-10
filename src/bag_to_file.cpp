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

#include <unistd.h>

#include <chrono>
#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>
#include <filesystem>
#include <fstream>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sstream>

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "bag_to_file -b input_bag -t topic -r rate [-o out_file] "
            << "[-T timestamp_file] [-s start_time] [-e end_time] " << std::endl;
}

using ffmpeg_image_transport_msgs::msg::FFMPEGPacket;
using Path = std::filesystem::path;
using rclcpp::Time;
using bag_time_t = rcutils_time_point_value_t;
static rclcpp::Logger get_logger()
{
  return (rclcpp::get_logger("bag_to_file"));
}

static void convertToMP4(const std::string & raw, const std::string & mp4, double rate)
{
  std::stringstream ss;
  ss << "ffmpeg -y -r " << rate << " -i " << raw << " -c:v copy " << mp4;
  int rc = std::system(ss.str().c_str());
  if (rc == -1) {
    std::cerr << " error command: " << ss.str() << std::endl;
  }
  rc = std::system(("rm " + raw).c_str());
  if (rc == -1) {
    std::cerr << " error removing file: " << raw << std::endl;
  }
}

size_t processBag(
  const std::string & out_file, const std::string & bag, const std::string & topic,
  const std::string & time_stamp_file, bag_time_t start_time, bag_time_t end_time)
{
  std::cout << "opening bag: " << bag << " topic: " << topic << std::endl;
  rosbag2_cpp::Reader reader;
  reader.open(bag);
  rosbag2_storage::StorageFilter filter;
  filter.topics.push_back(topic);
  reader.set_filter(filter);
  if (start_time != std::numeric_limits<bag_time_t>::min()) {
    std::cout << "seeking for start time stamp: " << start_time << std::endl;
    reader.seek(start_time);
  }
  rclcpp::Serialization<FFMPEGPacket> serialization;
  std::ofstream ts_file(time_stamp_file);
  std::fstream raw_file;
  raw_file.open(out_file, std::ios::app | std::ios::binary);

  size_t packet_number{0};
  while (reader.has_next()) {
    auto msg = reader.read_next();
    if (!msg || topic != msg->topic_name) {
      continue;
    }
    rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
    FFMPEGPacket::SharedPtr m(new FFMPEGPacket());
    serialization.deserialize_message(&serialized_msg, m.get());
    raw_file.write(reinterpret_cast<char *>(m->data.data()), m->data.size());
    const auto t_header = Time(m->header.stamp).nanoseconds();
    ts_file << packet_number << " " << Time(m->header.stamp).nanoseconds() << " "
            << Time(msg->time_stamp).nanoseconds() << std::endl;
    if (t_header > end_time) {
      break;
    }
    packet_number++;
  }
  return (packet_number);
}

int main(int argc, char ** argv)
{
  int opt;
  std::string bag;
  std::string out_file = "video.mp4";
  std::string topic;
  std::string time_stamp_file = "timestamps.txt";

  bag_time_t start_time = std::numeric_limits<bag_time_t>::min();
  bag_time_t end_time = std::numeric_limits<bag_time_t>::max();
  double rate(-1);

  while ((opt = getopt(argc, argv, "b:e:o:r:s:t:h")) != -1) {
    switch (opt) {
      case 'b':
        bag = optarg;
        break;
      case 'e':
        end_time = static_cast<bag_time_t>(atof(optarg) * 1e9);
        if (end_time < 0) {
          std::cout << "end time out of range, must be in seconds since start of epoch"
                    << std::endl;
          usage();
          return (-1);
        }
        break;
      case 'o':
        out_file = atof(optarg);
        break;
      case 'r':
        rate = atof(optarg);
        break;
      case 's':
        start_time = static_cast<bag_time_t>(atof(optarg) * 1e9);
        if (start_time < 0) {
          std::cout << "start time out of range, must be in seconds since start of epoch"
                    << std::endl;
          usage();
          return (-1);
        }
        break;
      case 't':
        topic = optarg;
        break;
      case 'h':
        usage();
        return (-1);
        break;
      default:
        std::cout << "unknown option: " << opt << std::endl;
        usage();
        return (-1);
        break;
    }
  }
  if (bag.empty()) {
    std::cout << "missing bag file argument!" << std::endl;
    usage();
    return (-1);
  }
  if (topic.empty()) {
    std::cout << "missing topic argument!" << std::endl;
    usage();
    return (-1);
  }
  if (rate <= 0) {
    std::cout << "missing rate argument!" << std::endl;
    usage();
    return (-1);
  }

  const auto start = std::chrono::high_resolution_clock::now();
  const std::string raw_file = out_file + ".h264";
  size_t num_packets = processBag(raw_file, bag, topic, time_stamp_file, start_time, end_time);
  convertToMP4(raw_file, out_file, rate);
  const auto stop = std::chrono::high_resolution_clock::now();
  auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  std::cout << "packets processed: " << num_packets << std::endl;
  std::cout << "total time for processing: " << total_duration.count() * 1e-6 << std::endl;
  return (0);
}
