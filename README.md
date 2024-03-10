# Tools for the ffmpeg\_image\_transport

This repository hosts code for handling data streams produced by the [ffmpeg_image_transport](https://github.com/ros-misc-utilities/ffmpeg_image_transport.git).

## Supported platforms

Should run on ROS2 distributions Humble and later.

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
and follow the [instructions here](https://github.com/ros-misc-utilities/.github/blob/master/docs/build_ros_repository.md)

## Processing rosbags

### Extract an mp4 file from a rosbag
The ``bag_to_file`` merges the ffmpeg-generated packets from a rosbag into a file:
```
bag_to_file -b input_bag -t topic -r rate [-o out_file] [-T timestamp_file] [-s start_time] [-e end_time]
```
For example the following line produces a file ``video.mp4`` and ``timestamps.txt`` from a rosbag:

```
ros2 run ffmpeg_image_transport_tools bag_to_file -t /cam1/image_raw/ffmpeg -r 40 -b ./my_rosbag/ -e 1710085164.466
```
The ``end_time`` is given in seconds since the epoch.
The ``timestamp.txt`` file has the following entries:
```
# packet, header_stamp  recording_stamp
0 1710085154473057750 1710085156001866724
1 1710085155950467594 1710085156024209913
```
A H264 packet corresponds to a frame so the packet number corresponds to frame number.

## License

This software is issued under the Apache License Version 2.0.
