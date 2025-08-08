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
#include <ffmpeg_encoder_decoder/decoder.hpp>
#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>
#include <ffmpeg_image_transport_tools/bag_processor.hpp>
#include <ffmpeg_image_transport_tools/frame_info.hpp>
#include <ffmpeg_image_transport_tools/message_processor.hpp>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <sensor_msgs/msg/image.hpp>

static rclcpp::Logger logger = rclcpp::get_logger("uncompress_bag");

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout
    << "uncompress_bag -i in_bag -o out_bag -t topic [-t topic ... ] [options]\n"
       "options:\n"
       " -s start_time [in sec since epoch]\n"
       " -e end_time [in sec since epoch]\n"
       "  ------- decoder options ------\n"
       " -D <key:value> decoder options:\n"
       "    decoder:<name_of_decoder> (default: h264)\n"
       "    decoder_output_format:<ros image format> (defaults to cv_bridge_target_format)\n"
       "    <av_option:value> any libav AVOption that the decoder accepts"
    << std::endl;
}

using ffmpeg_image_transport_msgs::msg::FFMPEGPacket;
using FrameInfo = ffmpeg_image_transport_tools::FrameInfo<FFMPEGPacket>;
using sensor_msgs::msg::Image;
typedef rcutils_time_point_value_t bag_time_t;
using namespace std::placeholders;
using highres_clock = std::chrono::high_resolution_clock;

class UnCompressor;

class Decoder
{
public:
  Decoder(
    UnCompressor * c, const std::string & topic,
    const std::unordered_map<std::string, std::string> & options)
  : uncompressor_(c), topic_(topic)
  {
    for (const auto & [n, v] : options) {
      if (n == "decoder") {
        decoder_name_ = v;
      } else if (n == "measure_performance") {
        decoder_.setMeasurePerformance(static_cast<bool>(std::stoi(v)));
      } else if (n == "decoder_output_format") {
        RCLCPP_INFO_STREAM(logger, "setting decoder output format: " << v);
        decoder_.setOutputMessageEncoding(v);
      } else {
        decoder_.addAVOption(n, v);
      }
    }
  }

  void decode(bag_time_t t_recv, bag_time_t t_send, const FFMPEGPacket::ConstSharedPtr & m)
  {
    stamp_to_image_.insert({rclcpp::Time(m->header.stamp), FrameInfo(m, t_recv, t_send)});
    if (!decoder_.isInitialized()) {
      RCLCPP_INFO_STREAM(logger, "decoding packets for codec: " << m->encoding);
      if (!decoder_.initialize(
            m->encoding, std::bind(&Decoder::frameReady, this, _1, _2, _3), decoder_name_)) {
        RCLCPP_ERROR_STREAM(
          logger,
          "cannot initialize decoder " << decoder_name_ << " for encoding: " << m->encoding);
        throw(std::runtime_error("cannot init codec"));
      }
      RCLCPP_INFO_STREAM(logger, "using decoder: " << decoder_name_);
      wait_for_key_frame_ = true;
    }
    if (wait_for_key_frame_) {
      if (!(m->flags & 0x0001)) {
        if (!waiting_for_key_frame_) {
          RCLCPP_INFO_STREAM(logger, "skipping non-key frames starting with pts: " << m->pts);
          waiting_for_key_frame_ = true;  // log message only onece
        }
        return;
      }
      if (waiting_for_key_frame_) {
        RCLCPP_INFO_STREAM(logger, "skipped non-key frames until pts: " << m->pts);
        waiting_for_key_frame_ = false;
      }
      wait_for_key_frame_ = false;  // found keyframe, good to go!
    }
    const auto t0 = highres_clock::now();
    if (!decoder_.decodePacket(
          m->encoding, m->data.data(), m->data.size(), m->pts, m->header.frame_id,
          m->header.stamp)) {
      RCLCPP_ERROR_STREAM(logger, "failed to decode packet for topic: " << topic_);
      throw(std::runtime_error("failed to decode packet for topic: " + topic_));
    }
    total_time_ +=
      std::chrono::duration_cast<std::chrono::milliseconds>(highres_clock::now() - t0).count();
  }

  void flush()
  {
    RCLCPP_INFO_STREAM(logger, "flushing encoder for topic: " << topic_);
    const auto t0 = highres_clock::now();
    decoder_.flush();
    total_time_ +=
      std::chrono::duration_cast<std::chrono::milliseconds>(highres_clock::now() - t0).count();
  }

  int64_t getDecodingTime() const
  {
    return (static_cast<int64_t>(total_time_) - static_cast<int64_t>(total_time_write_));
  }

  double getDecodingTimePerFrame() const
  {
    if (num_frames_decoded_ == 0) {
      return (0.0);
    }
    return (static_cast<double>(getDecodingTime()) / num_frames_decoded_);
  }

  double getDecodingRate() const
  {
    const double t = getDecodingTime();
    if (t <= 0) {
      return (0.0);
    }
    return (num_frames_decoded_ / (t * 1e-3));
  }

private:
  const FrameInfo & getFrameInfo(const rclcpp::Time & t)
  {
    const auto it = stamp_to_image_.find(t);
    if (it == stamp_to_image_.end()) {
      std::cerr << "decoded frame with stamp " << t.nanoseconds()
                << " not found in stamp_to_image_ map!" << std::endl;
      throw(std::runtime_error("decoded frame not found in stamp_to_image_ map!"));
    }
    return (it->second);
  }
  void frameReady(
    const Image::ConstSharedPtr & msg, bool /* is_key_frame */, const std::string & av_pix_format);

  UnCompressor * uncompressor_;
  std::string topic_;
  std::string frame_id_;
  std::shared_ptr<FFMPEGPacket> msg_;
  size_t num_frames_decoded_{0};
  ffmpeg_encoder_decoder::Decoder decoder_;
  std::string decoder_name_;
  std::map<rclcpp::Time, FrameInfo> stamp_to_image_;
  bool warn_of_size_mismatch_{true};
  bool waiting_for_key_frame_{false};
  bool wait_for_key_frame_{false};
  uint64_t total_time_{0};        // in milliseconds
  uint64_t total_time_write_{0};  // in milliseconds
};

class UnCompressor : public ffmpeg_image_transport_tools::MessageProcessor<FFMPEGPacket>
{
public:
  UnCompressor(
    const std::string & out_bag, const std::vector<std::string> & topics,
    const std::unordered_map<std::string, std::string> & options)
  : out_bag_(out_bag)
  {
    for (const std::string & topic : topics) {
      decoders_.insert({topic, std::make_shared<Decoder>(this, topic + "/decoded", options)});
    }
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    try {
      writer_->open(out_bag);
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(logger, "error opening output bag " << out_bag << ": " << e.what());
      throw(e);
    }
    for (const auto & topic : topics) {
      rosbag2_storage::TopicMetadata meta;
      meta.name = topic + "/decoded";
      meta.type = "sensor_msgs/msg/Image";
      meta.serialization_format = rmw_get_serialization_format();
      writer_->create_topic(meta);
    }
    start_time_ = highres_clock::now();
  }

  ~UnCompressor() override
  {
    for (auto & [topic, decoder] : decoders_) {
      decoder->flush();
    }
    RCLCPP_INFO_STREAM(logger, "closing bag file " << out_bag_);
    writer_->close();
    const auto dt =
      std::chrono::duration_cast<std::chrono::milliseconds>(highres_clock::now() - start_time_)
        .count();
    RCLCPP_INFO_STREAM(
      logger, "processed " << frame_number_ << " frames in " << 1e-3 * dt
                           << "s = " << (frame_number_ / (1e-3 * dt)) << " fps");
    for (const auto & [topic, decoder] : decoders_) {
      RCLCPP_INFO_STREAM(
        logger, "decoding topic " << topic << " took " << decoder->getDecodingTimePerFrame()
                                  << " ms/frame, rate: " << decoder->getDecodingRate() << " fps.");
    }
  }

  void process(
    bag_time_t t_recv, bag_time_t t_send, const std::string & topic,
    const FFMPEGPacket::ConstSharedPtr & m) final
  {
    auto it = decoders_.find(topic);
    if (it == decoders_.end()) {
      throw(std::runtime_error("topic " + topic + " not found in decoders!"));
    }
    it->second->decode(t_recv, t_send, m);
    frame_number_++;
  }

  template <typename MsgT>
  void write(
    bag_time_t t_recv, bag_time_t t_send, const typename MsgT::ConstSharedPtr & m,
    const std::string & topic)
  {
    auto smsg = std::make_shared<rclcpp::SerializedMessage>();
    rclcpp::Serialization<MsgT> serialization;
    serialization.serialize_message(m.get(), smsg.get());
    const std::string topicType = "ffmpeg_image_transport_msgs/msg/FFMPEGPacket";
#ifdef USE_NEW_ROSBAG_WRITE_INTERFACE
    writer_->write(smsg, topic, topicType, t_recv, t_send);
#else
    (void)t_send;
    writer_->write(smsg, topic, topicType, rclcpp::Time(t_recv));
#endif
    num_msgs_written_++;
  }

private:
  // -------------------- variables
  size_t frame_number_{0};
  std::chrono::time_point<highres_clock> start_time_;
  std::unordered_map<std::string, std::shared_ptr<Decoder>> decoders_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::string out_bag_;
  size_t num_msgs_written_{0};
};

void Decoder::frameReady(
  const Image::ConstSharedPtr & msg, bool /* is_key_frame */, const std::string & /*pix fmt*/)
{
  const auto fr = getFrameInfo(msg->header.stamp);
  const auto t0 = highres_clock::now();
  uncompressor_->write<Image>(fr.t_recv, fr.t_send, msg, topic_);
  num_frames_decoded_++;
  total_time_write_ +=
    std::chrono::duration_cast<std::chrono::milliseconds>(highres_clock::now() - t0).count();
}

int main(int argc, char ** argv)
{
  int opt;
  std::string in_bag;
  std::string out_bag;
  std::vector<std::string> topics;
  std::unordered_map<std::string, std::string> options;
  options["decoder"] = "h264";

  bag_time_t start_time = std::numeric_limits<bag_time_t>::min();
  bag_time_t end_time = std::numeric_limits<bag_time_t>::max();

  while ((opt = getopt(argc, argv, "i:o:t:D:s:e:h")) != -1) {
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
      case 'D': {
        std::string key_value(optarg);
        auto pos = key_value.find(':');
        if (pos == std::string::npos) {
          std::cout << "invalid option: " << optarg << ", must be in the form <key>:<value>"
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

  const std::string topic_type = "ffmpeg_image_transport_msgs/msg/FFMPEGPacket";
  ffmpeg_image_transport_tools::BagProcessor<FFMPEGPacket> bproc(
    logger, in_bag, topics, topic_type, start_time, end_time);
  UnCompressor unc(out_bag, topics, options);
  bproc.process(&unc);
}
