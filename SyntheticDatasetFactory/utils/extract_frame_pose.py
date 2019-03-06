#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Utility script to extract frames and drone poses from the given topics
(usually used with rosbag).
It generates a CSV file containing annotations for each extracted frame.
'''

import os
import sys
import cv2
import rospy
import cv_bridge
import message_filters

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CompressedImage, Image


class FramePoseExtractor():
    def __init__(self):
        self.output_path = rospy.get_param('~output', 'extracted_output/')
        self.img_name_fmt = "frame%d_%d.jpg"
        self.do_dynamic_scaling = rospy.get_param('~do_dynamic_scaling', False)
        self.raw = rospy.get_param('~raw', False)
        self.pose_topic = rospy.get_param('~poses', None)
        self.img_topic = rospy.get_param('~images', None)

        if self.pose_topic is None:
            rospy.logwarn("""FramePoseExtractor: rosparam '~poses' has not been specified!
Typical command-line usage:
      $ ./extract_frame_pose _poses:=<pose_topic> _images:=<image_topic> _output:=<output_path> _raw:=<1|0>'""")
            sys.exit(1)

        if self.img_topic is None:
            rospy.logwarn("""FramePoseExtractor: rosparam '~images' has not been specified!
Typical command-line usage:
      $ ./extract_frame_pose _poses:=<pose_topic> _images:=<image_topic> _output:=<output_path> _raw:=<1|0>'""")
            sys.exit(1)

    def extract(self):
        if not os.path.isdir(self.output_path):
            os.mkdir(self.output_path)
        self.csv = open(
            os.path.join(
                self.output_path, rospy.get_param('~filename', 'annotations.csv')
            ), 'w'
        )
        # Header
        self.csv.write("frame,translation_x,translation_y,translation_z,rotation_x,rotation_y,rotation_z,rotation_w\n")
        poses_sub = [message_filters.Subscriber(self.pose_topic, TransformStamped)]
        if self.raw:
            imgs_sub = [message_filters.Subscriber(self.img_topic, Image)]
        else:
            imgs_sub = [message_filters.Subscriber(self.img_topic, CompressedImage)]

        if rospy.get_param('~approximate_sync', False):
            print('[*] Using approximate sync')
            poses_sync = message_filters.ApproximateTimeSynchronizer(
                poses_sub, queue_size=100, slop=.1)
            imgs_sync = message_filters.ApproximateTimeSynchronizer(
                imgs_sub, queue_size=100, slop=.1)
        else:
            poses_sync = message_filters.TimeSynchronizer(
                poses_sub, queue_size=100)
            imgs_sync = message_filters.TimeSynchronizer(
                imgs_sub, queue_size=100)

        poses_sync.registerCallback(self._save_poses)
        if self.raw:
            imgs_sync.registerCallback(self._save_images)
        else:
            imgs_sync.registerCallback(self._save_compressed_images)
        print("[*] Extracting...")

    def _save_compressed_images(self, *img_messages):
        for i, img_msg in enumerate(img_messages):
            fname = self.img_name_fmt % (img_msg.header.stamp.secs, img_msg.header.stamp.nsecs)
            with open(os.path.join(self.output_path, fname), 'w') as img_file:
                img_file.write(img_msg.data)

    def _save_raw_images(self, *img_messages):
        bridge = cv_bridge.CvBridge()
        for i, img_msg in enumerate(img_messages):
            img = bridge.imgmsg_to_cv2(img_msg, desired_encoding="8UC3")
            channels = img.shape[2] if img.ndim == 3 else 1
            encoding_in = bridge.dtype_with_channels_to_cvtype2(img.dtype, channels)
            img = cv_bridge.cvtColorForDisplay(
                img, encoding_in="rgb8", encoding_out='',
                do_dynamic_scaling=self.do_dynamic_scaling)
            fname = self.img_name_fmt % (img_msg.header.stamp.secs, img_msg.header.stamp.nsecs)
            cv2.imwrite(fname, img)

    def _save_poses(self, *pose_msgs):
        for i, pose_msg in enumerate(pose_msgs):
            image_name = "frame{}_{}.jpg".format(pose_msg.header.stamp.secs,
                                              pose_msg.header.stamp.nsecs)
            translation = pose_msg.transform.translation
            rotation = pose_msg.transform.rotation
            self.csv.write("{},{},{},{},{},{},{},{}\n".format(
                image_name, translation.x, translation.y, translation.z,
                rotation.x, rotation.y, rotation.z, rotation.w
            ))


if __name__ == '__main__':
    rospy.init_node('extract_frame_pose')
    extractor = FramePoseExtractor()
    extractor.extract()
    rospy.spin()
