#!/usr/bin/env python

import os
import cv2
import rospy
from cv_bridge import CvBridge
import numpy as np
import rosbag
from message_filters import TimeSynchronizer, SimpleFilter
from tqdm import tqdm

cv_bridge = CvBridge()


def read_camera_intrinsics(cv_node: cv2.FileNode):  # type: ignore
    # Read camera intrinsics from cv_node

    camera_matrix = np.ones((3, 3))
    distortion_coefficients = np.ones((1, 4))

    camera_matrix = cv_node.getNode("K").mat()
    distortion_coefficients = cv_node.getNode("D").mat()

    print("Camera Matrix: ", camera_matrix)
    print("Distortion Coefficients: ", distortion_coefficients)

    return camera_matrix, distortion_coefficients


class ColmapStereo:
    def __init__(
        self,
        input_bag: str,
        root_folder: str,
        left_topic: str,
        right_topic: str,
        delay: float,
        scale=1.0,
        config_file=None,
        undistort=False,
    ):
        self.scale = scale
        self.undistort = undistort

        if not os.path.exists(root_folder):
            os.makedirs(root_folder)
        image_folder = os.path.join(root_folder, "images")
        if not os.path.exists(image_folder):
            os.mkdir(image_folder)

        self.left_img_folder = os.path.join(image_folder, "left")
        if not os.path.exists(self.left_img_folder):
            os.mkdir(self.left_img_folder)

        self.right_img_folder = os.path.join(image_folder, "right")
        if not os.path.exists(self.right_img_folder):
            os.mkdir(self.right_img_folder)

        self.last_timestamp = -1
        self.delay = delay
        self.indx = 0

        opencv_config = cv2.FileStorage(config_file, cv2.FILE_STORAGE_READ)  # type: ignore
        left_node = opencv_config.getNode("left")
        right_node = opencv_config.getNode("right")
        left_K, left_dist = read_camera_intrinsics(left_node)
        right_K, right_dist = read_camera_intrinsics(right_node)

        height = int(opencv_config.getNode("image_height").real())
        width = int(opencv_config.getNode("image_width").real())
        print(f"Width: {width}, Height: {height}")

        self.size = (width, height)

        self.left_map_x, self.left_map_y = cv2.initUndistortRectifyMap(  # type: ignore
            left_K, left_dist, np.eye(3), left_K, self.size, cv2.CV_32FC1  # type: ignore
        )
        self.right_map_x, self.right_map_y = cv2.initUndistortRectifyMap(  # type: ignore
            right_K, right_dist, np.eye(3), right_K, self.size, cv2.CV_32FC1  # type: ignore
        )

        self.topics = [left_topic, right_topic]
        print(self.topics)
        filters = [SimpleFilter() for _ in self.topics]
        stereo_filter = TimeSynchronizer(filters, 100)

        stereo_filter.registerCallback(self.stereo_callback)

        with rosbag.Bag(input_bag, "r") as ibag:
            for topic, msg, t in tqdm(ibag.read_messages(), total=ibag.get_message_count()):  # type: ignore
                if topic in self.topics:
                    filters[self.topics.index(topic)].signalMessage(msg)

    def stereo_callback(self, left_img_msg, right_img_msg):
        rospy.loginfo_once("Stereo callback called")
        left_timestamp = left_img_msg.header.stamp.to_nsec()
        right_timestamp = right_img_msg.header.stamp.to_nsec()

        assert left_timestamp == right_timestamp
        if ((left_timestamp - self.last_timestamp) * 1e-9) >= self.delay:
            if left_img_msg._type == "sensor_msgs/CompressedImage":
                left_image = cv_bridge.compressed_imgmsg_to_cv2(
                    left_img_msg, desired_encoding="bgr8"
                )
            elif left_img_msg._type == "sensor_msgs/Image":
                left_image = cv_bridge.imgmsg_to_cv2(
                    left_img_msg, desired_encoding="bgr8"
                )
            else:
                raise ValueError("Unknown left image type")

            if self.undistort:
                left_image_undistorted = cv2.remap(  # type: ignore
                    left_image,
                    self.left_map_x,
                    self.left_map_y,
                    interpolation=cv2.INTER_LINEAR,  # type: ignore
                )
            else:
                left_image_undistorted = left_image

            if self.scale != 1.0:
                left_image_undistorted = cv2.resize(  # type: ignore
                    left_image_undistorted,
                    (int(self.size[0] * self.scale), int(self.size[1] * self.scale)),
                )

            cv2.imwrite(  # type: ignore
                os.path.join(
                    self.left_img_folder, "{}.png".format(str(left_timestamp))
                ),
                left_image_undistorted,
            )

            if right_img_msg._type == "sensor_msgs/CompressedImage":
                right_image = cv_bridge.compressed_imgmsg_to_cv2(
                    right_img_msg, desired_encoding="bgr8"
                )
            elif right_img_msg._type == "sensor_msgs/Image":
                right_image = cv_bridge.imgmsg_to_cv2(
                    right_img_msg, desired_encoding="bgr8"
                )
            else:
                raise ValueError("Unknown right image type")

            if self.undistort:
                right_image_undistorted = cv2.remap(  # type: ignore
                    right_image,
                    self.right_map_x,
                    self.right_map_y,
                    interpolation=cv2.INTER_LINEAR,  # type: ignore
                )
            else:
                right_image_undistorted = right_image

            if self.scale != 1.0:
                right_image_undistorted = cv2.resize(  # type: ignore
                    right_image_undistorted,
                    (int(self.size[0] * self.scale), int(self.size[1] * self.scale)),
                )

            cv2.imwrite(  # type: ignore
                os.path.join(
                    self.right_img_folder, "{}.png".format(str(right_timestamp))
                ),
                right_image_undistorted,
            )

            self.last_timestamp = left_timestamp
            self.indx += 1


if __name__ == "__main__":
    rospy.init_node("bag_extract_stereo", anonymous=True)

    if not rospy.has_param("~input_bag"):
        rospy.logfatal("Require the input bag file")
        exit(1)

    if not rospy.has_param("~image_dir"):
        rospy.logfatal("Require the dataset path of asl directory")
        exit(1)

    if not rospy.has_param("~config_file"):
        rospy.logfatal("Require the config file path")
        exit(1)

    if not rospy.has_param("~left_topic"):
        rospy.logfatal("Require the left image topic")
        exit(1)

    if not rospy.has_param("~right_topic"):
        rospy.logfatal("Require the right image topic")
        exit(1)

    config_file = rospy.get_param("~config_file")
    image_folder = rospy.get_param("~image_dir")
    left_topic = rospy.get_param("~left_topic")
    right_topic = rospy.get_param("~right_topic")
    input_bag = rospy.get_param("~input_bag")

    if rospy.has_param("~scale"):
        scale = rospy.get_param("~scale")

    freq = 2.0
    if rospy.has_param("~freq"):
        freq = rospy.get_param("~freq")

    undistort = False
    if rospy.has_param("~undistort"):
        undistort = rospy.get_param("~undistort")

    delay = 1.0 / freq  # type: ignore
    rospy.loginfo("Delay : {}".format(delay))

    colmap_stereo = ColmapStereo(
        input_bag, image_folder, left_topic, right_topic, delay, scale=scale, config_file=config_file, undistort=undistort  # type: ignore
    )

    while not rospy.is_shutdown():
        rospy.spin()
