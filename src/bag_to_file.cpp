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

#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>
#include <ffmpeg_image_transport_tools/bag_processor.hpp>
#include <ffmpeg_image_transport_tools/message_processor.hpp>
#include <filesystem>
#include <fstream>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout
    << "bag_to_file -i input_bag -t topic -r rate [-o out_file] "
    << "[-T timestamp_file] [-s start_time (sec since epoch)] [-e end_time (sec since epoch)] "
    << std::endl;
}

using ffmpeg_image_transport_msgs::msg::FFMPEGPacket;
using bag_time_t = rcutils_time_point_value_t;

rclcpp::Logger logger = rclcpp::get_logger("bag_to_file");

static void convertToMP4(const std::string & raw, const std::string & mp4, double rate)
{
  std::stringstream ss;
  ss << "ffmpeg -y -r " << rate << " -i " << raw << " -c:v copy " << mp4;
  int rc = std::system(ss.str().c_str());
  if (rc == -1) {
    RCLCPP_ERROR_STREAM(logger, " error command: " << ss.str());
  }
  rc = std::system(("rm " + raw).c_str());
  if (rc == -1) {
    RCLCPP_ERROR_STREAM(logger, " error removing file: " << raw);
  }
}

class FileWriter : public ffmpeg_image_transport_tools::MessageProcessor<FFMPEGPacket>
{
public:
  FileWriter(const std::string & raw_file, const std::string & ts_file)
  {
    RCLCPP_INFO_STREAM(logger, "writing to raw file: " << raw_file);
    raw_file_ = std::ofstream(raw_file, std::ios::out | std::ios::binary);
    ts_file_.open(ts_file);
  }

  void process(
    rcutils_time_point_value_t t_recv, rcutils_time_point_value_t t_send, const std::string &,
    const FFMPEGPacket::ConstSharedPtr & m) final
  {
    (void)t_send;
    const auto t_header = Time(m->header.stamp).nanoseconds();
    raw_file_.write(reinterpret_cast<const char *>(m->data.data()), m->data.size());
    ts_file_ << packet_number_++ << " " << m->pts << " " << t_header << " "
             << rclcpp::Time(t_recv).nanoseconds() << std::endl;
  }

private:
  std::ofstream raw_file_;
  std::ofstream ts_file_;
  size_t packet_number_{0};
};

int main(int argc, char ** argv)
{
  int opt;
  std::string bag;
  std::string out_file = "video";
  std::string topic;
  std::string time_stamp_file = "timestamps.txt";

  bag_time_t start_time = std::numeric_limits<bag_time_t>::min();
  bag_time_t end_time = std::numeric_limits<bag_time_t>::max();
  double rate(-1);

  while ((opt = getopt(argc, argv, "i:e:o:r:s:t:h")) != -1) {
    switch (opt) {
      case 'i':
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
        out_file = optarg;
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

  const std::string raw_file = out_file + ".h264";
  const std::string topic_type = "ffmpeg_image_transport_msgs/msg/FFMPEGPacket";
  std::vector<std::string> topics{topic};
  ffmpeg_image_transport_tools::BagProcessor<FFMPEGPacket> bproc(
    logger, bag, topics, topic_type, start_time, end_time);
  FileWriter fw(raw_file, time_stamp_file);
  bproc.process(&fw);
  convertToMP4(raw_file, out_file + ".mp4", rate);
}
