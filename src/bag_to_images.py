#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Copyright 2022 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#

"""
grab images from bag.
"""

import rosbag2_py
import argparse
import rclpy
import cv2
import pdb
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def get_rosbag_options(path, serialization_format="cdr"):
    storage_options = rosbag2_py.StorageOptions(uri=path)

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format,
    )

    return storage_options, converter_options


def main(args):
    in_bag_name = str(args.in_bag)
    storage_options, converter_options = get_rosbag_options(in_bag_name)
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    # Create a map for quicker lookup
    topic_types = reader.get_all_topics_and_types()
    type_map = {
        topic_types[i].name: topic_types[i].type for i in range(len(topic_types))
    }

    # Set filter for topic of string type
    storage_filter = rosbag2_py.StorageFilter(topics=[args.topic])
    reader.set_filter(storage_filter)
    bridge = CvBridge()
    cnt = 0
    while reader.has_next():
        (topic, data, t_rec) = reader.read_next()
        if topic == args.topic:
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            stamp = msg.header.stamp
            # print(f'{cnt} {stamp.sec} {stamp.nanosec}')
            cnt += 1
            img = bridge.imgmsg_to_cv2(msg, "bgr8")
            ts = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds
            print(f"{cnt:5d} converted image at time: {ts}")
            cv2.imwrite(f"{args.out_dir}/frame_{ts}.png", img)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="grab single image from bag")
    parser.add_argument(
        "--in_bag",
        "-i",
        action="store",
        default=None,
        required=True,
        help="bag file to images from",
    )
    parser.add_argument(
        "--topic", "-t", action="store", required=True, help="image topic"
    )
    parser.add_argument(
        "--out_dir",
        "-o",
        action="store",
        default=None,
        required=True,
        help="name of output directory",
    )
    main(parser.parse_args())
