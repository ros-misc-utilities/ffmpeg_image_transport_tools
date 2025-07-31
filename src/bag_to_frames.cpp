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
#include <ffmpeg_encoder_decoder/utils.hpp>
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
  std::cout
    << "usage:\n\n"
    << "bag_to_frames -i input_bag -t topic [options]\n\n"
    << "options:\n"
    << " -o out_dir         name of the output directory (defaults to \"frames\").\n"
    << " -d decoder         name of the libav decoder (hevc_cuvid, libx264 etc).\n"
    << " -O output_format   ros encoding ('bgr8', 'mono',..) to convert to before writing image.\n"
    << " -f file_type       frame file type ('png', 'jpeg'). Defaults to jpeg.\n"
    << " -T timestamp_file  name of time stamp file.\n"
    << " -s start_time      time in sec since epoch.\n"
    << " -e end_time        time in sec since epoch." << std::endl;
}

using ffmpeg_encoder_decoder::Decoder;
using ffmpeg_image_transport_msgs::msg::FFMPEGPacket;
using sensor_msgs::msg::Image;
using Path = std::filesystem::path;
using rclcpp::Time;
using bag_time_t = rcutils_time_point_value_t;
using ffmpeg_encoder_decoder::utils::split_by_char;

using namespace std::placeholders;
namespace fs = std::filesystem;

rclcpp::Logger logger = rclcpp::get_logger("bag_to_frames");

class FrameWriter : public ffmpeg_image_transport_tools::MessageProcessor<FFMPEGPacket>
{
public:
  FrameWriter(
    const std::vector<std::string> & decoders, const std::string & base_dir,
    const std::string & ts_file, const std::string & output_format = std::string(),
    const std::string & output_file_type = "jpg")
  : base_dir_(base_dir),
    decoder_names_(decoders),
    output_format_(output_format),
    output_file_type_(output_file_type)
  {
    if (!fs::is_directory(base_dir_) || !fs::exists(base_dir_)) {
      fs::create_directory(base_dir_);
    }
    ts_file_.open(Path(base_dir_) / Path(ts_file));
    if (!output_format_.empty()) {
      RCLCPP_INFO_STREAM(logger, "forcing decoder output format to be: " << output_format_);
      decoder_.setOutputMessageEncoding(output_format);
    }
  }

  void process(
    rcutils_time_point_value_t t_recv, rcutils_time_point_value_t, const std::string &,
    const FFMPEGPacket::ConstSharedPtr & m) final
  {
    if (!decoder_.isInitialized()) {
      if (firstTime_) {
        RCLCPP_INFO_STREAM(logger, "decoding packets for codec: " << m->encoding);
        firstTime_ = false;
        if (decoder_names_.empty()) {
          decoder_names_ =
            split_by_char(Decoder::findDecoders(split_by_char(m->encoding, ';')[0]), ',');
        }
      }
      while (!decoder_names_.empty()) {
        const auto name = *decoder_names_.begin();
        decoder_names_.erase(decoder_names_.begin());  // mark as used
        if (!decoder_.initialize(
              m->encoding, std::bind(&FrameWriter::callback, this, _1, _2, _3), name)) {
          RCLCPP_ERROR_STREAM(
            logger, "cannot initialize decoder " << name << " for encoding: " << m->encoding);
        } else {
          RCLCPP_INFO_STREAM(logger, "using decoder: " << name);
          waitForKeyFrame_ = true;
          break;
        }
      }
      if (!decoder_.isInitialized()) {
        RCLCPP_ERROR_STREAM(logger, "no valid decoder found for encoding: " << m->encoding);
        throw(std::runtime_error("cannot init codec"));
      }
    }
    if (waitForKeyFrame_) {
      if (!(m->flags & 0x0001)) {
        if (!waitingForKeyFrame_) {
          RCLCPP_INFO_STREAM(logger, "skipping non-key frames starting with pts: " << m->pts);
          waitingForKeyFrame_ = true;
        }
        return;
      }
      if (waitingForKeyFrame_) {
        RCLCPP_INFO_STREAM(logger, "skipped non-key frames until pts: " << m->pts);
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
  std::string makeFileName(const uint64_t t)
  {
    std::stringstream ss;
    ss << "frame_" << std::setfill('0') << std::setw(9) << t << "." << output_file_type_;
    return (Path(base_dir_) / Path(ss.str()));
  }

  cv::Mat toCVMat(const Image::ConstSharedPtr & msg)
  {
    // Very few encoders support monochrome encoding, and
    // as of 2025 none support bayer images. To hack around this,
    // images are encoded as nv12 (a popular hw accelerated format)
    // with a zero-set color channel. When decoding we force "nv12"
    // back to monochrome by dropping the color channel.
    if (msg->encoding == "nv12") {
      // there are extra rows at the bottom with (0) Cr and Cb channels
      const uint32_t num_rows = msg->height + msg->height / 2;
      cv::Mat img_nv12(
        num_rows, msg->width, cv::DataType<uint8_t>::type, const_cast<uint8_t *>(msg->data.data()),
        msg->step / 2);
      cv::Mat img_gray = img_nv12(cv::Rect(0, 0, msg->width, msg->height));
      return (img_gray);
    } else {
      auto cv_img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      if (!cv_img) {
        throw(cv_bridge::Exception("compress_bag:: cv_bridge cannot convert image"));
      }
      return (cv_img->image);
    }
  }

  void callback(const Image::ConstSharedPtr & msg, bool, const std::string & avPixFmt)
  {
    const uint64_t t = rclcpp::Time(msg->header.stamp).nanoseconds();
    cv::Mat img = toCVMat(msg);
    if (!printedHeader_) {
      RCLCPP_INFO_STREAM(
        logger, "ros encoding: " << msg->encoding << " channels: " << img.channels() << " "
                                 << msg->width << "x" << msg->height << " step: " << msg->step
                                 << " libav pix fmt: " << avPixFmt);
      RCLCPP_INFO_STREAM(
        logger, "writing file type " << output_file_type_ << " to folder: " << base_dir_);
      printedHeader_ = true;
    }
    const auto fname = makeFileName(t);
    cv::imwrite(fname, img);
    if (++frame_number_ % 100 == 0) {
      RCLCPP_INFO_STREAM(logger, "wrote " << frame_number_ << " frames.");
    }
  }

private:
  std::string base_dir_;
  std::vector<std::string> decoder_names_;
  std::ofstream ts_file_;
  size_t frame_number_{0};
  Decoder decoder_;
  size_t packet_number_{0};
  std::string output_format_;
  std::string output_file_type_{"jpg"};
  bool firstTime_{true};
  bool waitForKeyFrame_{false};
  bool waitingForKeyFrame_{false};
  bool printedHeader_{false};
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
  std::string output_format;

  bag_time_t start_time = std::numeric_limits<bag_time_t>::min();
  bag_time_t end_time = std::numeric_limits<bag_time_t>::max();

  while ((opt = getopt(argc, argv, "i:d:e:o:O:s:f:t:h")) != -1) {
    switch (opt) {
      case 'i':
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
      case 'O':
        output_format = optarg;
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
    logger, bag, topics, topic_type, start_time, end_time);
  std::vector<std::string> decoders;
  if (!decoder.empty()) {
    decoders.push_back(decoder);
  }
  FrameWriter fw(decoders, out_dir, time_stamp_file, output_format, file_type);
  bproc.process(&fw);
}
