# Tools for the ffmpeg\_image\_transport

This repository hosts code for handling data streams produced by the [ffmpeg_image_transport](https://github.com/ros-misc-utilities/ffmpeg_image_transport.git).

## Supported systems

Continuous integration is tested under Ubuntu with the following ROS2 distros:

 [![Build Status](https://build.ros2.org/buildStatus/icon?job=Hdev__ffmpeg_image_transport_tools__ubuntu_jammy_amd64&subject=Humble)](https://build.ros2.org/job/Hdev__ffmpeg_image_transport_tools__ubuntu_jammy_amd64/)
 [![Build Status](https://build.ros2.org/buildStatus/icon?job=Idev__ffmpeg_image_transport_tools__ubuntu_jammy_amd64&subject=Iron)](https://build.ros2.org/job/Idev__ffmpeg_image_transport_tools__ubuntu_jammy_amd64/)
 [![Build Status](https://build.ros2.org/buildStatus/icon?job=Rdev__ffmpeg_image_transport_tools__ubuntu_jammy_amd64&subject=Rolling)](https://build.ros2.org/job/Rdev__ffmpeg_image_transport_tools__ubuntu_jammy_amd64/)

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

The ``rate`` determines the fps used by ffmpeg when producing the
output. The video is not transcoded, so what you get is the original
stream, just in a mp4 container. To get correct playing speed the rate must
match the rate at which the stream was originally recorded.

The ``start_time`` and ``end_time`` is given in seconds since the
epoch, like the times you see when using ``ros2 bag info``. 

For example the following line produces a file ``video.mp4`` and ``timestamps.txt`` from a rosbag:
```
ros2 run ffmpeg_image_transport_tools bag_to_file -t /cam1/image_raw/ffmpeg -r 40 -b ./my_rosbag/ -e 1710085164.466
```

The ``timestamp.txt`` file has the following entries:
```
# packet no, pts, header_stamp  recording_stamp
0 0 1710085154473057750 1710085156001866724
1 1 1710085155950467594 1710085156024209913
```
A H264 packet typically corresponds to a frame so the packet number
conincides with the  frame number.

### Extract frames from a rosbag
The ``bag_to_frames`` decodes the ffmpeg-generated packets from a rosbag into frames:
```
bag_to_frames -b input_bag -t topic [-o out_dir] [-d decoder][-T timestamp_file] [-s start_time] [-e end_time]
```
The frames are written to ``out_dir`` with the ros header stamps embedded in the file name. A suitable decoder is usually guessed from the encoding used in the packet, but you can specify a valid ffmpeg decoder by using the ``-d decoder`` option. For start and stop times and the timestamp file see ``bag_to_file``.

## License

This software is issued under the Apache License Version 2.0.
