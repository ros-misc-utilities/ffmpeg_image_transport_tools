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

#include <unistd.h>

#include <chrono>
#include <ffmpeg_encoder_decoder/encoder.hpp>
#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>
#include <ffmpeg_image_transport_tools/bag_processor.hpp>
#include <ffmpeg_image_transport_tools/message_processor.hpp>
#include <filesystem>
#include <fstream>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <sensor_msgs/msg/image.hpp>

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "compress_bag -i in_bag -o out_bag -t topic [-t topic ... ] "
               "[-O key=value] [-s start_time] [-e end_time] "
            << std::endl;
  std::cout << " -O encoding options:" << std::endl
            << "    encoder=<name_of_encoder>, defaults to libx264" << std::endl
            << "    preset" << std::endl
            << "    tune" << std::endl
            << "    delay" << std::endl
            << "    crf" << std::endl
            << "    pixel_format" << std::endl
            << "    qmax (quantization error, value of 1 is best quality)" << std::endl
            << "    bit_rate" << std::endl
            << "    gop_size" << std::endl
            << "    measure_performance" << std::endl;
}

using ffmpeg_image_transport_msgs::msg::FFMPEGPacket;
using sensor_msgs::msg::Image;
using bag_time_t = rcutils_time_point_value_t;
using namespace std::placeholders;

class Compressor;
class Encoder
{
public:
  Encoder(
    Compressor * c, const std::string & topic,
    const std::unordered_map<std::string, std::string> & options)
  : compressor_(c), topic_(topic)
  {
    for (const auto & [n, v] : options) {
      if (n == "encoder") {
        encoder_.setEncoder(v);
        std::cout << topic_ << " encoded with " << v << std::endl;
      } else if (n == "preset") {
        encoder_.setPreset(v);
      } else if (n == "tune") {
        encoder_.setTune(v);
      } else if (n == "delay") {
        encoder_.setDelay(v);
      } else if (n == "crf") {
        encoder_.setCRF(v);
      } else if (n == "pixel_format") {
        encoder_.setPixelFormat(v);
      } else if (n == "qmax") {
        encoder_.setQMax(std::stoi(v));
      } else if (n == "bit_rate") {
        encoder_.setBitRate(std::stoi(v));
      } else if (n == "gop_size") {
        encoder_.setGOPSize(std::stoi(v));
      } else if (n == "measure_performance") {
        encoder_.setMeasurePerformance(static_cast<bool>(std::stoi(v)));
      } else {
        std::cerr << "unknown parameter: " << n << std::endl;
      }
    }
  }

  void encode(uint64_t t, const Image::ConstSharedPtr & m)
  {
    const auto & msg = *m;
    if (!encoder_.isInitialized()) {
      if (!encoder_.initialize(
            msg.width, msg.height,
            std::bind(&Encoder::packetReady, this, _1, _2, _3, _4, _5, _6, _7, _8, _9))) {
        throw(std::runtime_error("cannot init encoder!"));
      }
    }
    last_recording_time_ = t;
    encoder_.encodeImage(msg);
  }

  void packetReady(
    const std::string & frame_id, const rclcpp::Time & stamp, const std::string & codec,
    uint32_t width, uint32_t height, uint64_t pts, uint8_t flags, uint8_t * data, size_t sz);

  void flush()
  {
    if (msg_) {
      encoder_.flush(msg_->header);
    }
  }

private:
  Compressor * compressor_;
  std::string topic_;
  std::shared_ptr<FFMPEGPacket> msg_;
  ffmpeg_encoder_decoder::Encoder encoder_;
  uint64_t last_recording_time_{0};
};

class Compressor : public ffmpeg_image_transport_tools::MessageProcessor<Image>
{
public:
  Compressor(
    const std::string & out_bag, const std::vector<std::string> & topics,
    const std::unordered_map<std::string, std::string> & options)
  {
    for (const std::string topic : topics) {
      encoders_.insert({topic, std::make_shared<Encoder>(this, topic + "/ffmpeg", options)});
    }
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    try {
      writer_->open(out_bag);
    } catch (const std::exception & e) {
      std::cerr << "error opening output bag " << out_bag << ": " << e.what() << std::endl;
      throw(e);
    }
    for (const auto & topic : topics) {
      rosbag2_storage::TopicMetadata meta;
      meta.name = topic + "/ffmpeg";
      meta.type = "ffmpeg_image_transport_msgs/msg/FFMPEGPacket";
      meta.serialization_format = rmw_get_serialization_format();
      writer_->create_topic(meta);
    }
    start_time_ = std::chrono::high_resolution_clock::now();
  }

  ~Compressor() override
  {
    for (auto & [topic, encoder] : encoders_) {
      encoder->flush();
    }
    std::cout << "closing bag file" << std::endl;
    writer_->close();
    const auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::high_resolution_clock::now() - start_time_)
                      .count();
    std::cout << "processed " << frame_number_ << " frames in " << 1e-3 * dt << "s" << std::endl;
    std::cout << "rate: " << (frame_number_ / (1e-3 * dt)) << " fps" << std::endl;
  }

  void process(uint64_t t, const std::string & topic, const Image::ConstSharedPtr & m) final
  {
    auto it = encoders_.find(topic);
    if (it == encoders_.end()) {
      throw(std::runtime_error("topic " + topic + " not found in encoders!"));
    }
    it->second->encode(t, m);
    frame_number_++;
  }

  template <typename MsgT>
  void write(
    const rclcpp::Time & t, const typename MsgT::ConstSharedPtr & m, const std::string & topic)
  {
    auto smsg = std::make_shared<rclcpp::SerializedMessage>();
    rclcpp::Serialization<MsgT> serialization;
    serialization.serialize_message(m.get(), smsg.get());
    const std::string topicType = "ffmpeg_image_transport_msgs/msg/FFMPEGPacket";
#ifdef USE_NEW_ROSBAG_WRITE_INTERFACE
    writer_->write(smsg, topic, topicType, t);
#else
    writer_->write(*smsg, topic, topicType, t);
#endif
  }

private:
  // -------------------- variables
  size_t frame_number_{0};
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
  std::unordered_map<std::string, std::shared_ptr<Encoder>> encoders_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
};

void Encoder::packetReady(
  const std::string & frame_id, const rclcpp::Time & stamp, const std::string & codec,
  uint32_t width, uint32_t height, uint64_t pts, uint8_t flags, uint8_t * data, size_t sz)
{
  msg_ = std::make_shared<FFMPEGPacket>();
  msg_->header.frame_id = frame_id;
  msg_->header.stamp = stamp;
  msg_->encoding = codec;
  msg_->width = width;
  msg_->height = height;
  msg_->pts = pts;
  msg_->flags = flags;
  msg_->data.resize(sz);
  memcpy(msg_->data.data(), data, sz);
  compressor_->write<FFMPEGPacket>(
    rclcpp::Time(last_recording_time_, RCL_SYSTEM_TIME), msg_, topic_);
}

int main(int argc, char ** argv)
{
  int opt;
  std::string in_bag;
  std::string out_bag;
  std::vector<std::string> topics;
  std::unordered_map<std::string, std::string> options;
  options["encoder"] = "libx264";

  bag_time_t start_time = std::numeric_limits<bag_time_t>::min();
  bag_time_t end_time = std::numeric_limits<bag_time_t>::max();
  double rate(-1);
  while ((opt = getopt(argc, argv, "i:o:t:O:s:e:h")) != -1) {
    switch (opt) {
      case 'i':
        in_bag = optarg;
        break;
      case 'o':
        out_bag = optarg;
        break;
      case 't':
        topics.push_back(optarg);
        break;
      case 'O': {
        std::string key_value(optarg);
        auto pos = key_value.find('=');
        if (pos == std::string::npos) {
          std::cout << "invalid option: " << optarg << ", must be in the form <key>=<value>"
                    << std::endl;
          usage();
          return (-1);
        }
        std::string key = key_value.substr(0, pos);
        std::string value = key_value.substr(pos + 1);
        options[key] = value;
      } break;
      case 's':
        start_time = static_cast<bag_time_t>(atof(optarg) * 1e9);
        if (start_time < 0) {
          std::cout << "start time out of range, must be in seconds since start of epoch"
                    << std::endl;
          usage();
          return (-1);
        }
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
  if (in_bag.empty()) {
    std::cout << "missing in_bag argument!" << std::endl;
    usage();
    return (-1);
  }
  if (out_bag.empty()) {
    std::cout << "missing out_bag argument!" << std::endl;
    usage();
    return (-1);
  }
  if (topics.empty()) {
    std::cout << "missing topics argument!" << std::endl;
    usage();
    return (-1);
  }

  const std::string topic_type = "sensor_msgs/msg/Image";
  ffmpeg_image_transport_tools::BagProcessor<Image> bproc(
    in_bag, topics, topic_type, start_time, end_time);
  Compressor comp(out_bag, topics, options);
  bproc.process(&comp);
}
