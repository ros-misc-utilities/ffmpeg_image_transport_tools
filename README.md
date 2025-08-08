# Tools for the ffmpeg\_image\_transport

This repository hosts code for handling data streams produced by the [ffmpeg_image_transport](https://github.com/ros-misc-utilities/ffmpeg_image_transport.git).

## Supported systems

Continuous integration is tested under Ubuntu with the following ROS2 distros:

 [![Build Status](https://build.ros2.org/buildStatus/icon?job=Hdev__ffmpeg_image_transport_tools__ubuntu_jammy_amd64&subject=Humble)](https://build.ros2.org/job/Hdev__ffmpeg_image_transport_tools__ubuntu_jammy_amd64/)
 [![Build Status](https://build.ros2.org/buildStatus/icon?job=Jdev__ffmpeg_image_transport_tools__ubuntu_noble_amd64&subject=Jazzy)](https://build.ros2.org/job/Jdev__ffmpeg_image_transport_tools__ubuntu_noble_amd64/)
 [![Build Status](https://build.ros2.org/buildStatus/icon?job=Rdev__ffmpeg_image_transport_tools__ubuntu_noble_amd64&subject=Rolling)](https://build.ros2.org/job/Rdev__ffmpeg_image_transport_tools__ubuntu_noble_amd64/)

## How to install

### From packages

```bash
sudo apt-get install ros-${ROS_DISTRO}-ffmpeg-image-transport-tools
```

### From source

Set the following shell variables:
```bash
repo=ffmpeg_image_transport_tools
url=https://github.com/ros-misc-utilities/${repo}.git
```
and follow the [instructions here](https://github.com/ros-misc-utilities/.github/blob/master/docs/build_ros_repository.md).

## About encoders, decoders, and pixel formats

The tools in this package often have many parameters related to encoders, decoders, and pixel formats.
It is important to understand the terminology, and how single-channel images (like ``mono8`` and Bayer images) are handled.
Please consult the [ffmpeg\_encoder\_decoder repo](https://github.com/ros-misc-utilities/ffmpeg_encoder_decoder) for more information.

## Programs for processing rosbags

### bag\_to\_file
The ``bag_to_file`` tool concatenates the FFMPEGPacket message content from a rosbag into a file, and runs the ``ffmpeg`` tool to embed the stream into a an ``mp4`` container.
```
bag_to_file -i input_bag -t topic -r rate [-o out_file] [-T timestamp_file] [-s start_time (sec since epoch)] [-e end_time (sec since epoch)]
```

The ``rate`` determines the fps used by ffmpeg when producing the output.
The video is not transcoded, so what you get is the original stream, just in an mp4 container.
To get correct playing speed the rate must match the rate at which the stream was originally recorded.

For example the following line produces a file ``video.mp4`` and ``timestamps.txt`` from a rosbag:
```
ros2 run ffmpeg_image_transport_tools bag_to_file -t /cam1/image_raw/ffmpeg -r 40 -i ./my_rosbag/ -e 1710085164.466
```

The ``timestamp.txt`` file facilitates correlating ROS time stamps with frame numbers. It has the following entries:
```
# packet no, pts, header_stamp  recording_stamp
0 0 1710085154473057750 1710085156001866724
1 1 1710085155950467594 1710085156024209913
```
A H264 packet typically corresponds to a frame so the packet number should conincide with the  frame number.

### bag\_to\_frames
The ``bag_to_frames`` program  decodes the ffmpeg-generated packets from a rosbag into frames:
```
bag_to_frames -i input_bag -t topic [options]

options:
 -o out_dir         name of the output directory (defaults to "frames")
 -d decoder         name of the libav decoder (hevc_cuvid, libx264 etc)
 -O output_format   ros encoding ('bgr8', 'mono', ...) to convert to before writing image.
 -f file_type       frame file type ('png', 'jpeg'). Defaults to jpeg.
 -T timestamp_file  name of time stamp file.
 -s start_time      time in sec since epoch.
 -e end_time        time in sec since epoch.
```
If not specified, the decoder will be automatically (not necessarily correctly) picked. Not all decoders for a codec can handle all encoded image formats.
For example if the codec is ``hevc`` and a bayer image (``bayer_rggb8``) has been encoded with image format ``nv12``, then the ``h265`` will throw the following error:
```
[INFO] [1753428586.070137592] [bag_to_frames]: using decoder: hevc
[WARN] [1753428586.406598438] [Decoder]: hardware frame transfer failed for pixel format yuv420p
```

Before the image is written a final conversion to the ``output_format`` is performed. The default is ``bgr8``.
The format string must follow [ROS convention](https://docs.ros.org/en/jazzy/p/sensor_msgs/generated/program_listing_file_include_sensor_msgs_image_encodings.hpp.html) rather than the libav convention, i.e. ``bgr8`` rather than ``bgr24``.


### compress\_bag
Use ``ros2 run ffmpeg_image_transport_tools compress_bag`` to encode a video stored as Image messages into FFMPEGPacket format.
Usage is as follows:
```
compress_bag -i in_bag -o out_bag -t topic [-t topic ... ] [options]
options:
 -q enable quality check
 -I write debug images
 -m max_num_frames_to_keep (for matching encoded/decoded packets. defaults to 100)
 -s start_time [in sec since epoch]
 -e end_time [in sec since epoch]
  ------- encoder options -------
 -E <key:value> encoding options:
    encoder:<name_of_encoder> (defaults to libx264)
    cv_bridge_target_format:<name_of_ros_format> (defaults to rgb8)
    av_source_pixel_format:<name_of_libav_pixel_format> (fed into the encoder)
    qmax:<qmax> (no default, quantization error, value of 1 is best quality)
    bit_rate:<bit_rate>
    max_b_frames:<max_b_frames>
    gop_size:<gop_size>
    measure_performance:<0 or 1>
    (any libav option like tune, profile, crf, delay, following key:value syntax)
  ------- decoder options ------
 -D <key:value> decoder options:
    decoder:<name_of_decoder> (no default, use e.g. h264)
    decoder_output_format:<ros image format> (defaults to cv_bridge_target_format)
  ------- quality check options ------
 -Q <key:value> quality check options:
    quality_check_format:<ros image format for quality check> (default rgb8)
```
NOTE: the header timestamps within each topic must be unique, i.e. images with the same topic cannot have identical time stamps!

You can also check for the quality of the encoding by immediately decoding the encoded packet with the ``-q`` switch (quality check).
This will slow down the conversion quite a bit though.
The error printed out is computed by taking the absolute of the difference of original and encoded image separate for each channel.

The following example will compress lossless and check the quality:
```
 ros2 run ffmpeg_image_transport_tools compress_bag -i <input_bag_name> -o <output_bag_name> -t <topic_name> -E encoder:libx264rgb -E crf:0 -D decoder:h264 -q
```
If you watch the console log you may notice that the ``cv_bridge_target_format`` is ``bgr8``, which could be different from the original ROS image encoding of the rosbag images.
This means the images are converted to ``bgr8`` before they are even encoded.
To force encoding in the original image format, explicitly specify the format with the
``-E cv_bridge_target_format:format`` option.

Here is an example for lossless encoding and testing bayer images in ``bayer_rggb8`` format.
Without the ``-E cv_bridge_target_format:bayer_rggb8`` the images would be converted to ``rgb8`` by default, then maybe converted again to ``nv12`` or ``yuv420p`` (lossy!) before encoding.
```
ros2 run ffmpeg_image_transport_tools compress_bag  -i <input_bag> -o <output_bag> -t <topic_1> -t <topic_2> -E encoder:hevc_nvenc -E tune:lossless -D decoder:hevc -E cv_bridge_target_format:bayer_rggb8 -q
```
Note that the final check of image quality (``-q``) by default happens in the (default) format of ``rgb8`` again.
To perform the check in the original (e.g. ``bayer_rggb8``) format, use the decoder option ``-Q quality_check_format:bayer_rggb8``.
```
ros2 run ffmpeg_image_transport_tools compress_bag  -i <input_bag> -o <output_bag> -t <topic_1> -t <topic_2> -E encoder:hevc_nvenc -E tune:lossless -D decoder:hevc -E cv_bridge_target_format:bayer_rggb8 -Q quality_check_format:bayer_rggb8 -q
```

### uncompress\_bag
The ``uncompress\_bag`` program can be used to convert a bag with ffmpeg-encoded images (FFMPEGPacket message type) into a bag with regular (sensor_msgs/Image) messages.
```
uncompress_bag -i in_bag -o out_bag -t topic [-t topic ... ] [options]
options:
 -s start_time [in sec since epoch]
 -e end_time [in sec since epoch]
  ------- decoder options ------
 -D <key:value> decoder options:
    decoder:<name_of_decoder> (default: h264)
    decoder_output_format:<ros image format> (defaults to original encoding format)
    <av_option:value> any libav AVOption that the decoder accepts
```
Example usage for decompressing a bag that was encoded with ``hevc``:
```
ros2 run ffmpeg_image_transport_tools uncompress_bag -i <bag_with_compressed_bayer_rggb8> -o <uncompressed_bag> -t <camera_topic/ffmpeg> -D decoder:hevc
```

## License

This software is issued under the Apache License Version 2.0.
