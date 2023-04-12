#!/usr/bin/env python

import cv2
import argparse
import os
from tqdm import tqdm
import rosbag

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="combine_gopro_bags",
        description="Come left and right gopro bags into stereo",
        add_help=True,
    )
    parser.add_argument("--left_bag", "-l", type=str, help="path to left gopro bag")
    parser.add_argument("--right_bag", "-r", type=str, help="path to right gopro bag")
    parser.add_argument("--output_bag", "-o", type=str, help="path to output bag")

    args = parser.parse_args()
    left_bag = args.left_bag
    right_bag = args.right_bag
    output_bag = args.output_bag

    if left_bag is None or right_bag is None or output_bag is None:
        print("Please specify all bag paths")
        exit(1)

    left = rosbag.Bag(left_bag, "r")
    right = rosbag.Bag(right_bag, "r")
    out = rosbag.Bag(output_bag, "w")

    for topic, msg, t in left.read_messages():
        if topic == "/gopro/image_raw":
            out.write("/gopro/left/image_raw", msg, t)
        elif topic == "/gopro/image_raw/compressed":
            out.write("/gopro/left/image_raw/compressed", msg, t)
        elif topic == "/gopro/imu":
            out.write("/gopro/left/imu", msg, t)
        elif topic == "/gopro/magnetic_field":
            out.write("/gopro/left/magnetic_field", msg, t)

    for topic, msg, t in right.read_messages():
        if topic == "/gopro/image_raw":
            out.write("/gopro/right/image_raw", msg, t)
        elif topic == "/gopro/image_raw/compressed":
            out.write("/gopro/right/image_raw/compressed", msg, t)
        elif topic == "/gopro/imu":
            out.write("/gopro/right/imu", msg, t)
        elif topic == "/gopro/magnetic_field":
            out.write("/gopro/right/magnetic_field", msg, t)

    left.close()
    right.close()
    out.close()
