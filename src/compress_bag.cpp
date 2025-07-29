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
#include <ffmpeg_encoder_decoder/encoder.hpp>
#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>
#include <ffmpeg_image_transport_tools/bag_processor.hpp>
#include <ffmpeg_image_transport_tools/message_processor.hpp>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <sensor_msgs/msg/image.hpp>
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

// #define DEBUG_ERROR_HISTOGRAMS
#define DEBUG_ERROR_DETAILS
#define DEBUG_WRITE_IMAGES

#ifdef DEBUG_WRITE_IMAGES
std::string make_file_name(const std::string & prefix, const size_t n)
{
  std::stringstream ss;
  ss << prefix << std::setfill('0') << std::setw(4) << n << ".png";
  return (ss.str());
}
#endif

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "compress_bag -i in_bag -o out_bag -t topic [-t topic ... ] [-q (check quality)] "
               "[-m max_num_frames_to_keep] "
               "[-O key=value] [-s start_time] [-e end_time] "
            << std::endl;
  std::cout << " -O encoding options:" << std::endl
            << "    encoder=<name_of_encoder>, defaults to libx264" << std::endl
            << "    preset" << std::endl
            << "    tune" << std::endl
            << "    profile" << std::endl
            << "    delay" << std::endl
            << "    crf" << std::endl
            << "    pixel_format (used before encoding)" << std::endl
            << "    encoder_cv_bridge_target_format" << std::endl
            << "    qmax (quantization error, value of 1 is best quality)" << std::endl
            << "    bit_rate" << std::endl
            << "    gop_size" << std::endl
            << "    measure_performance" << std::endl;
  std::cout << " -O decoder options:" << std::endl
            << "    decoder=<name_of_decoder> (no default, use e.g. h264)" << std::endl;
}

using ffmpeg_image_transport_msgs::msg::FFMPEGPacket;
using sensor_msgs::msg::Image;
using bag_time_t = rcutils_time_point_value_t;
using namespace std::placeholders;

class Compressor;

struct FrameInfo
{
  FrameInfo(
    const Image::ConstSharedPtr & m, rcutils_time_point_value_t t_r, rcutils_time_point_value_t t_s)
  : msg(m), t_recv(t_r), t_send(t_s)
  {
  }
  Image::ConstSharedPtr msg;
  rcutils_time_point_value_t t_recv;
  rcutils_time_point_value_t t_send;
};

class EncoderDecoder
{
public:
  EncoderDecoder(
    Compressor * c, const std::string & topic, bool check_quality, size_t mnf,
    const std::unordered_map<std::string, std::string> & options)
  : compressor_(c), topic_(topic), check_quality_(check_quality), max_num_frames_to_keep_(mnf)
  {
    for (const auto & [n, v] : options) {
      if (n == "encoder") {
        encoder_.setEncoder(v);
        std::cout << topic_ << " encoding with " << v << std::endl;
      } else if (n == "decoder") {
        decoder_type_ = v;
      } else if (n == "encoder_pixel_format") {
        std::cout << "libav encoder uses pixel format to " << v << std::endl;
        encoder_.setAVSourcePixelFormat(v);
      } else if (n == "encoder_cv_bridge_target_format") {
        encoder_.setCVBridgeTargetFormat(v);
      } else if (n == "decoder_message_output_format") {
        decoder_.setOutputMessageEncoding(v);
        std::cout << "decoder uses output format: " << v << std::endl;
      } else if (n == "quality_message_pixel_format") {
        std::cout << "for quality check, uses pixel format: " << v << std::endl;
        qualityMessagePixelFormat_ = v;
      } else if (n == "qmax") {
        encoder_.setQMax(std::stoi(v));
      } else if (n == "bit_rate") {
        encoder_.setBitRate(std::stoi(v));
      } else if (n == "gop_size") {
        encoder_.setGOPSize(std::stoi(v));
      } else if (n == "measure_performance") {
        encoder_.setMeasurePerformance(static_cast<bool>(std::stoi(v)));
      } else {
        encoder_.addAVOption(n, v);
      }
    }

    if (check_quality_) {
      std::string s = topic_;
      std::replace(s.begin(), s.end(), '/', '_');
#ifdef DEBUG_ERROR_HISTOGRAMS
      histogram_file_.open(s + "_histogram.txt");
#endif
    }
  }
  void encode(
    rcutils_time_point_value_t t_recv, rcutils_time_point_value_t t_send,
    const Image::ConstSharedPtr & msg_color)
  {
#ifdef ENCODE_AS_GREY_FOR_DEBUG
    auto cv_grey_img = cv_bridge::toCvCopy(msg_color, sensor_msgs::image_encodings::MONO8);
    auto m = cv_grey_img->toImageMsg();
#else
    auto & m = msg_color;
#endif
    frame_id_ = m->header.frame_id;
    stamp_to_image_.insert({rclcpp::Time(m->header.stamp), FrameInfo(m, t_recv, t_send)});
    while (stamp_to_image_.size() > max_num_frames_to_keep_) {
      stamp_to_image_.erase(stamp_to_image_.begin());
    }
    const auto & msg = *m;
    if (!encoder_.isInitialized()) {
      if (!encoder_.initialize(
            msg.width, msg.height,
            std::bind(&EncoderDecoder::packetReady, this, _1, _2, _3, _4, _5, _6, _7, _8, _9))) {
        throw(std::runtime_error("cannot init encoder!"));
      }
    }
    const auto t0 = std::chrono::high_resolution_clock::now();
    encoder_.encodeImage(msg);
    total_time_encode_plus_write_ += std::chrono::duration_cast<std::chrono::milliseconds>(
                                       std::chrono::high_resolution_clock::now() - t0)
                                       .count();
    num_frames_encoded_++;
  }

  // this method only called for quality check
  void decodedFrameReady(const Image::ConstSharedPtr & msg, bool /* isKeyFrame */)
  {
    std::cout << "decoded frame: " << msg->encoding << " " << msg->width << "x" << msg->height
              << " sz: " << msg->data.size() << " per line: " << msg->data.size() / msg->height
              << " step: " << msg->step << std::endl;
    const auto & frame_info = getFrameInfo(rclcpp::Time(msg->header.stamp));
    const auto & orig_msg = frame_info.msg;

    if (
      (msg->width != orig_msg->width || msg->height != orig_msg->height) &&
      warn_of_size_mismatch_) {
      std::cerr << "WARNING: decoded image size mismatch! Decoded: " << msg->width << "x"
                << msg->height << " vs original: " << orig_msg->width << "x" << orig_msg->height
                << std::endl;
    }

    auto decoded_image = cv_bridge::toCvShare(msg, qualityMessagePixelFormat_);
    if (!decoded_image) {
      throw(std::runtime_error("cannot convert image to cv::Mat!"));
    }
    auto original_image = cv_bridge::toCvShare(orig_msg, qualityMessagePixelFormat_);
    if (!original_image) {
      throw(std::runtime_error("cannot convert image to cv::Mat!"));
    }
    const auto & img_original = original_image->image;
    const auto & img_decoded = decoded_image->image;
    if (img_original.channels() != img_decoded.channels()) {
      std::cerr << "number of channels changed: " << img_original.channels() << " -> "
                << img_decoded.channels() << std::endl;
      throw(std::runtime_error("decoded image channels does not match original!"));
    }
    if (
      (img_original.rows != img_decoded.rows || img_original.cols != img_decoded.cols) &&
      warn_of_size_mismatch_) {
      std::cerr << "decoded image size does not match. Decoded: " << img_decoded.cols << "x"
                << img_decoded.rows << " vs original: " << img_original.cols << "x"
                << img_original.rows << std::endl;
    }
    warn_of_size_mismatch_ = false;
    cv::Mat diff;
    const int rows = std::min(img_original.rows, img_decoded.rows);
    const int cols = std::min(img_original.cols, img_decoded.cols);
    cv::Mat sub_decoded = img_decoded(cv::Rect(0, 0, cols, rows));
    cv::Mat sub_original = img_original(cv::Rect(0, 0, cols, rows));

    cv::absdiff(sub_original, sub_decoded, diff);
    cv::Scalar mean_diff = cv::mean(diff);
    for (int i = 0; i < img_original.channels(); i++) {
      total_mean_diff_ += mean_diff[i];
    }

#if defined(DEBUG_ERROR_HISTOGRAMS) || defined(DEBUG_ERROR_DETAILS)
    std::vector<cv::Mat> channels;
    cv::split(diff, channels);
#endif
#ifdef DEBUG_ERROR_HISTOGRAMS
    for (int channel = 0; channel < img_original.channels(); ++channel) {
      const int histSize = 256;
      const float range[] = {0, 256};
      const float * histRange[] = {range};
      cv::Mat hist;
      cv::calcHist(
        &channels[channel], 1, 0, cv::Mat() /* mask */, hist, 1, &histSize, histRange,
        true /*uniform*/, false /*accumulate*/);
      for (int i = 0; i < histSize; ++i) {
        histogram_file_ << " " << hist.at<float>(i);
      }
      histogram_file_ << std::endl;
    }
#endif
#ifdef DEBUG_ERROR_DETAILS
    std::cout << std::setw(5) << num_frames_decoded_ << " error: ";
    for (int channel = 0; channel < img_original.channels(); ++channel) {
      double min_val, max_val;
      cv::Point min_loc, max_loc;
      cv::minMaxLoc(channels[channel], &min_val, &max_val, &min_loc, &max_loc);
      std::cout << " mean: " << mean_diff[channel] << " max: " << max_val << " at (" << max_loc.x
                << "," << max_loc.y << ")";
    }
    std::cout << std::endl;
#endif

#ifdef DEBUG_WRITE_IMAGES
    const auto ts = rclcpp::Time(msg->header.stamp).nanoseconds();
    cv::imwrite(make_file_name("orig_", ts), img_original);
    cv::imwrite(make_file_name("dec_", ts), img_decoded);
#endif
    num_frames_decoded_++;
  }

  void packetReady(
    const std::string & frame_id, const rclcpp::Time & stamp, const std::string & codec,
    uint32_t width, uint32_t height, uint64_t pts, uint8_t flags, uint8_t * data, size_t sz);

  void flush()
  {
    std::cout << "flushing encoder for topic: " << topic_ << std::endl;
    const auto t0 = std::chrono::high_resolution_clock::now();
    encoder_.flush();
    total_time_encode_plus_write_ += std::chrono::duration_cast<std::chrono::milliseconds>(
                                       std::chrono::high_resolution_clock::now() - t0)
                                       .count();
  }

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

  double getEncodingTime() const
  {
    return static_cast<double>(
             total_time_encode_plus_write_ - total_time_write_ - total_time_decode_) /
           num_frames_encoded_;
  }
  double getDecodingTime() const
  {
    return static_cast<double>(total_time_decode_) / num_frames_decoded_;
  }
  double getEncodingRate() const
  {
    const double dt = getEncodingTime();
    return (dt > 0.0 ? (1000.0 / dt) : -1.0);
  }
  double getMeanDiff() const
  {
    return (num_frames_decoded_ > 0 ? total_mean_diff_ / (num_frames_decoded_) : 0.0);
  }
  double getDecodingRate() const
  {
    const double dt = getDecodingTime();
    return (dt > 0.0 ? (1000.0 / dt) : -1.0);
  }

private:
  Compressor * compressor_;
  std::string topic_;
  std::string frame_id_;
  std::shared_ptr<FFMPEGPacket> msg_;
  ffmpeg_encoder_decoder::Encoder encoder_;
  ffmpeg_encoder_decoder::Decoder decoder_;
  std::string qualityMessagePixelFormat_{sensor_msgs::image_encodings::BGR8};
  uint64_t total_time_write_{0};  // in milliseconds
  uint64_t total_time_encode_plus_write_{0};
  uint64_t total_time_decode_{};
  size_t num_frames_encoded_{0};
  size_t num_frames_decoded_{0};
  std::string decoder_type_;
  bool check_quality_{false};  // check quality of encoded frames
  size_t max_num_frames_to_keep_{0};
  std::map<rclcpp::Time, FrameInfo> stamp_to_image_;
  std::ofstream histogram_file_;
  double total_mean_diff_{0.0};  // sum of mean differences for all decoded frame
  bool warn_of_size_mismatch_{true};
};

class Compressor : public ffmpeg_image_transport_tools::MessageProcessor<Image>
{
public:
  Compressor(
    const std::string & out_bag, const std::vector<std::string> & topics, bool check_quality,
    size_t max_num_frames_to_keep, const std::unordered_map<std::string, std::string> & options)
  : out_bag_(out_bag), check_quality_(check_quality)
  {
    for (const std::string & topic : topics) {
      encoderdecoders_.insert(
        {topic, std::make_shared<EncoderDecoder>(
                  this, topic + "/ffmpeg", check_quality, max_num_frames_to_keep, options)});
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
    for (auto & [topic, encoder] : encoderdecoders_) {
      encoder->flush();
    }
    std::cout << "closing bag file " << out_bag_ << std::endl;
    writer_->close();
    const auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::high_resolution_clock::now() - start_time_)
                      .count();
    std::cout << "processed " << frame_number_ << " frames in " << 1e-3 * dt
              << "s = " << (frame_number_ / (1e-3 * dt)) << " fps" << std::endl;
    for (const auto & [topic, encoder] : encoderdecoders_) {
      std::cout << "encoding for topic " << topic << " takes " << encoder->getEncodingTime()
                << "ms/frame, rate: " << encoder->getEncodingRate() << "fps";
      if (check_quality_) {
        std::cout << " decode rate: " << encoder->getDecodingRate()
                  << " fps, mean err: " << encoder->getMeanDiff();
      }
      std::cout << std::endl;
    }
  }

  void process(
    rcutils_time_point_value_t t_recv, rcutils_time_point_value_t t_send, const std::string & topic,
    const Image::ConstSharedPtr & m) final
  {
    auto it = encoderdecoders_.find(topic);
    if (it == encoderdecoders_.end()) {
      throw(std::runtime_error("topic " + topic + " not found in encoders!"));
    }
    it->second->encode(t_recv, t_send, m);
    frame_number_++;
  }

  template <typename MsgT>
  void write(
    rcutils_time_point_value_t t_recv, rcutils_time_point_value_t t_send,
    const typename MsgT::ConstSharedPtr & m, const std::string & topic)
  {
    auto smsg = std::make_shared<rclcpp::SerializedMessage>();
    rclcpp::Serialization<MsgT> serialization;
    serialization.serialize_message(m.get(), smsg.get());
    const std::string topicType = "ffmpeg_image_transport_msgs/msg/FFMPEGPacket";
#ifdef USE_NEW_ROSBAG_WRITE_INTERFACE
    writer_->write(smsg, topic, topicType, t_recv, t_send);
#else
    writer_->write(*smsg, topic, topicType, t_recv);
#endif
    num_msgs_written_++;
  }

private:
  // -------------------- variables
  size_t frame_number_{0};
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
  std::unordered_map<std::string, std::shared_ptr<EncoderDecoder>> encoderdecoders_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::string out_bag_;
  bool check_quality_{false};
  size_t num_msgs_written_{0};
};

void EncoderDecoder::packetReady(
  const std::string & frame_id, const rclcpp::Time & stamp, const std::string & codec,
  uint32_t width, uint32_t height, uint64_t pts, uint8_t flags, uint8_t * data, size_t sz)
{
  const auto fr = getFrameInfo(stamp);

  const auto t0 = std::chrono::high_resolution_clock::now();
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

  compressor_->write<FFMPEGPacket>(fr.t_recv, fr.t_send, msg_, topic_);
  total_time_write_ += std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::high_resolution_clock::now() - t0)
                         .count();
  if (check_quality_) {
    if (!decoder_.isInitialized()) {
      if (decoder_type_.empty()) {
        throw(std::runtime_error("no decoder specified!"));
      }
      if (!decoder_.initialize(
            codec, std::bind(&EncoderDecoder::decodedFrameReady, this, _1, _2), decoder_type_)) {
        throw(std::runtime_error("cannot init decoder!"));
      }
    }
    const auto t1 = std::chrono::high_resolution_clock::now();
    if (!decoder_.decodePacket(
          msg_->encoding, msg_->data.data(), msg_->data.size(), msg_->pts, msg_->header.frame_id,
          msg_->header.stamp)) {
      std::cerr << "error decoding packet for topic " << topic_ << std::endl;
    }
    total_time_decode_ += std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::high_resolution_clock::now() - t1)
                            .count();
  }
}

int main(int argc, char ** argv)
{
  int opt;
  std::string in_bag;
  std::string out_bag;
  std::vector<std::string> topics;
  bool check_quality = false;
  size_t max_num_frames_to_keep = 100;
  std::unordered_map<std::string, std::string> options;
  options["encoder"] = "libx264";

  bag_time_t start_time = std::numeric_limits<bag_time_t>::min();
  bag_time_t end_time = std::numeric_limits<bag_time_t>::max();

  while ((opt = getopt(argc, argv, "i:o:t:O:s:e:m:qh")) != -1) {
    switch (opt) {
      case 'i':
        in_bag = optarg;
        break;
      case 'o':
        out_bag = optarg;
        break;
      case 'm':
        max_num_frames_to_keep = static_cast<size_t>(std::stoi(optarg));
        break;
      case 'q':
        check_quality = true;
        break;
      case 't':
        topics.push_back(optarg);
        break;
      case 'O': {
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

  const std::string topic_type = "sensor_msgs/msg/Image";
  ffmpeg_image_transport_tools::BagProcessor<Image> bproc(
    in_bag, topics, topic_type, start_time, end_time);
  Compressor comp(out_bag, topics, check_quality, max_num_frames_to_keep, options);
  bproc.process(&comp);
}
