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
            << " [-f file_type] [-T timestamp_file] [-s start_time]" << " [-e end_time] "
            << std::endl;
}
//
// lossless encoding:
// ffmpeg -framerate 1 -pattern_type glob -i 'orig/*.png'
// -c:v libx264rgb -preset slow -crf 0 out.mp4 ;
// ffmpeg -i out.mp4  -r 1/1 new/$filename%03d.png
//
// -c:v h264_nvenc -preset:v p7 -tune:v lossless -profile:v high444p
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
    const std::vector<std::string> & decoders, const std::string & base_dir,
    const std::string & ts_file, const std::string & output_file_type = "jpg")
  : base_dir_(base_dir), decoder_names_(decoders), output_file_type_(output_file_type)
  {
    if (!fs::is_directory(base_dir_) || !fs::exists(base_dir_)) {
      fs::create_directory(base_dir_);
    }
    ts_file_.open(Path(base_dir_) / Path(ts_file));
  }

  void process(
    rcutils_time_point_value_t t_recv, rcutils_time_point_value_t t_send, const std::string &,
    const FFMPEGPacket::ConstSharedPtr & m) final
  {
    if (!decoder_.isInitialized()) {
      if (firstTime_) {
        firstTime_ = false;
        if (decoder_names_.empty()) {
          decoder_names_ = Decoder::findDecoders(m->encoding);
        }
      }
      while (!decoder_names_.empty()) {
        const auto name = *decoder_names_.begin();
        decoder_names_.erase(decoder_names_.begin());  // mark as used
        if (!decoder_.initialize(
              m->encoding, std::bind(&FrameWriter::callback, this, std::placeholders::_1), name)) {
          std::cerr << "cannot initialize decoder " << name << " for encoding: " << m->encoding
                    << std::endl;
        } else {
          waitForKeyFrame_ = true;
          break;
        }
      }
      if (!decoder_.isInitialized()) {
        std::cerr << "no valid decoder found for encoding: " << m->encoding << std::endl;
        throw(std::runtime_error("cannot init codec"));
      }
    }
    if (waitForKeyFrame_) {
      if (!(m->flags & 0x0001)) {
        if (!waitingForKeyFrame_) {
          std::cout << "skipping non-key frames starting with pts: " << m->pts << std::endl;
          waitingForKeyFrame_ = true;
        }
        return;
      }
      if (waitingForKeyFrame_) {
        std::cout << "skipped non-key frames until pts: " << m->pts << std::endl;
        waitingForKeyFrame_ = false;
      }
      waitForKeyFrame_ = false;
    }

    const bool ret = decoder_.decodePacket(
      m->encoding, m->data.data(), m->data.size(), m->pts, m->header.frame_id, m->header.stamp);
    if (!ret) {
      decoder_.reset();
    }
    ts_file_ << packet_number_++ << " " << m->pts << " " << Time(m->header.stamp).nanoseconds()
             << " " << rclcpp::Time(t_recv).nanoseconds() << std::endl;
  }

private:
  std::string make_file_name(const uint64_t t)
  {
    std::stringstream ss;
    ss << "frame_" << std::setfill('0') << std::setw(9) << t << "." << output_file_type_;
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
  std::vector<std::string> decoder_names_;
  std::ofstream ts_file_;
  size_t frame_number_{0};
  Decoder decoder_;
  size_t packet_number_{0};
  std::string output_file_type_{"jpg"};
  bool firstTime_{true};
  bool waitForKeyFrame_{false};
  bool waitingForKeyFrame_{false};
};

int main(int argc, char ** argv)
{
  int opt;
  std::string bag;
  std::string out_dir = "frames";
  std::string topic;
  std::string time_stamp_file = "timestamps.txt";
  std::string decoder;
  std::string file_type = "jpg";

  bag_time_t start_time = std::numeric_limits<bag_time_t>::min();
  bag_time_t end_time = std::numeric_limits<bag_time_t>::max();

  while ((opt = getopt(argc, argv, "b:d:e:o:s:f:t:h")) != -1) {
    switch (opt) {
      case 'b':
        bag = optarg;
        break;
      case 'd':
        decoder = optarg;
        break;
      case 'f':
        file_type = optarg;
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
        out_dir = optarg;
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
  const std::vector<std::string> topics{topic};
  const std::string topic_type = "ffmpeg_image_transport_msgs/msg/FFMPEGPacket";
  ffmpeg_image_transport_tools::BagProcessor<FFMPEGPacket> bproc(
    bag, topics, topic_type, start_time, end_time);
  std::vector<std::string> decoders;
  if (!decoder.empty()) {
    decoders.push_back(decoder);
  }
  FrameWriter fw(decoders, out_dir, time_stamp_file, file_type);
  bproc.process(&fw);
}
