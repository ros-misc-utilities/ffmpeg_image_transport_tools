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
#ifdef USE_CV_BRIDGE_HPP
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <ffmpeg_encoder_decoder/decoder.hpp>
#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>
#include <ffmpeg_image_transport_tools/bag_processor.hpp>
#include <ffmpeg_image_transport_tools/message_processor.hpp>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <sstream>

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "bag_to_frames -b input_bag -t topic [-o out_dir] [-d decoder]"
            << "[-T timestamp_file] [-s start_time] [-e end_time] " << std::endl;
}

using ffmpeg_encoder_decoder::Decoder;
using ffmpeg_image_transport_msgs::msg::FFMPEGPacket;
using sensor_msgs::msg::Image;
using Path = std::filesystem::path;
using rclcpp::Time;
using bag_time_t = rcutils_time_point_value_t;
namespace fs = std::filesystem;

class FrameWriter : public ffmpeg_image_transport_tools::MessageProcessor<FFMPEGPacket>
{
public:
  FrameWriter(
    const std::string & decoder, const std::string & base_dir, const std::string & ts_file)
  : base_dir_(base_dir), decoder_type_(decoder)
  {
    if (!fs::is_directory(base_dir_) || !fs::exists(base_dir_)) {
      fs::create_directory(base_dir_);
    }
    ts_file_.open(Path(base_dir_) / Path(ts_file));
  }

  void process(uint64_t t, const FFMPEGPacket::ConstSharedPtr & m) final
  {
    if (!decoder_.isInitialized()) {
      std::string dtype = decoder_type_;
      if (dtype.empty()) {
        const auto & decoderMap = Decoder::getDefaultEncoderToDecoderMap();
        auto decTypeIt = decoderMap.find(m->encoding);
        if (decTypeIt == decoderMap.end()) {
          std::cerr << "unknown encoding: " << m->encoding << std::endl;
          throw(std::runtime_error("unknown encoding: " + m->encoding));
        }
        dtype = decTypeIt->second;
      }

      decoder_.initialize(
        m->encoding, std::bind(&FrameWriter::callback, this, std::placeholders::_1), dtype);
    }
    if (!decoder_.isInitialized()) {
      std::cerr << "cannot init codec" << std::endl;
      throw(std::runtime_error("cannot init codec"));
    }
    bool decodePacket(
      const std::string & encoding, const uint8_t * data, size_t size, uint64_t pts,
      const std::string & frame_id, const rclcpp::Time & stamp);

    decoder_.decodePacket(
      m->encoding, m->data.data(), m->data.size(), m->pts, m->header.frame_id, m->header.stamp);
    ts_file_ << packet_number_++ << " " << m->pts << " " << Time(m->header.stamp).nanoseconds()
             << " " << t << std::endl;
  }

private:
  std::string make_file_name(const uint64_t t)
  {
    std::stringstream ss;
    ss << "frame_" << std::setfill('0') << std::setw(9) << t << ".jpg";
    return (Path(base_dir_) / Path(ss.str()));
  }

  void callback(const Image::ConstSharedPtr & msg)
  {
    cv::Mat img;

    const uint64_t t = rclcpp::Time(msg->header.stamp).nanoseconds();
    cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image.copyTo(img);
    const auto fname = make_file_name(t);
    cv::imwrite(fname, img);
    if (++frame_number_ % 100 == 0) {
      std::cout << "wrote " << frame_number_ << " frames." << std::endl;
    }
  }

private:
  std::string base_dir_;
  std::string decoder_type_;
  std::ofstream ts_file_;
  size_t frame_number_{0};
  Decoder decoder_;
  size_t packet_number_{0};
};

int main(int argc, char ** argv)
{
  int opt;
  std::string bag;
  std::string out_dir = "frames";
  std::string topic;
  std::string time_stamp_file = "timestamps.txt";
  std::string decoder;

  bag_time_t start_time = std::numeric_limits<bag_time_t>::min();
  bag_time_t end_time = std::numeric_limits<bag_time_t>::max();

  while ((opt = getopt(argc, argv, "b:d:e:o:s:t:h")) != -1) {
    switch (opt) {
      case 'b':
        bag = optarg;
        break;
      case 'd':
        decoder = optarg;
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
        out_dir = atof(optarg);
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

  const std::string topic_type = "ffmpeg_image_transport_msgs/msg/FFMPEGPacket";
  ffmpeg_image_transport_tools::BagProcessor<FFMPEGPacket> bproc(
    bag, topic, topic_type, start_time, end_time);
  FrameWriter fw(decoder, out_dir, time_stamp_file);
  bproc.process(&fw);
}
