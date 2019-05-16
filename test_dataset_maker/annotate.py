#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2019 theo <theo@not-arch-linux>
#
# Distributed under terms of the MIT license.

"""
Annotate a base dataset of images with camera poses, by retro-projecting the
given gate position (in the world frame) on the image frame.
"""

from dataset import Dataset, TestAnnotations, AnnotatedImage
from pyrr import Matrix44, Vector3, Vector4, Quaternion
from tqdm import *

import multiprocessing.dummy as mp
import numpy as np
import argparse
import yaml
import os


class Annotator():
    def __init__(self, base_dataset: str, dest: str, config: str,
                 camera_parameters: str):
        with open(config, 'r') as conf:
            try:
                self.gates_config = yaml.safe_load(conf)
            except yaml.YAMLError as exc:
                raise Exception(exc)
        with open(camera_parameters, 'r') as cam_param:
            try:
                self.camera_parameters = yaml.safe_load(cam_param)
            except yaml.YAMLError as exc:
                raise Exception(exc)
        self.base_dataset = Dataset(os.path.join(base_dataset, "images"))
        self.base_dataset.load(annotations_path=os.path.join(base_dataset, "annotations.csv"),
                               randomize=False)
        self.width = self.base_dataset.width
        self.height = self.base_dataset.height
        self.annotated_dataset = Dataset(dest, max=150)
        self.__compute_camera_matrix()

    def __compute_camera_matrix(self):
        camera_intrinsics = [
            self.camera_parameters['camera_matrix']['data'][0:3],
            self.camera_parameters['camera_matrix']['data'][3:6],
            self.camera_parameters['camera_matrix']['data'][6::]
        ]
        fx, fy = camera_intrinsics[0][0], camera_intrinsics[1][1]
        cx, cy = camera_intrinsics[0][2], camera_intrinsics[1][2]
        x0, y0 = 0, 0 # Camera image origin
        zfar, znear = 100.0, 0.1 # distances to the clipping plane
        self.projection = Matrix44([
            [fx/cx, 0, 0, 0],
            [0, fy/cy, 0, 0],
            [0, 0, (-zfar - znear)/(zfar - znear), -1],
            [0, 0, (-2.0*zfar*znear)/(zfar - znear), 0]
        ])

    def run(self):
        print("[*] Generating dataset...")
        save_thread = mp.threading.Thread(target=self.annotated_dataset.save)
        save_thread.start()
        for i in tqdm(range(self.base_dataset.size()),
                      unit="img", bar_format="{l_bar}{bar}| {n_fmt}/{total_fmt}"):
            self.__generate(i)

        print("[*] Saving...")
        self.annotated_dataset.data.put(None)
        save_thread.join()
        print("[*] Saved to {}".format(self.annotated_dataset.path))

    def __generate(self, index):
        baseImage = self.base_dataset.get()
        annotations = []
        for gate in self.gates_config:
            pose = self.gates_config[gate]['translation']
            orientation = self.gates_config[gate]['rotation']
            print("[*] Pose: ", pose)
            print("[*] Orientation: ", orientation)
            input("")
            view = self.__compute_view_matrix(baseImage.annotations)
            gate_coord_img_frame = self.__back_project(pose, orientation, view)
            annotations.append(TestAnnotations(gate_coord_img_frame,
                                               Quaternion(),
                                              0.0, True))
        self.annotated_dataset.put(AnnotatedImage(baseImage.image_path(), index, annotations))

    def __compute_view_matrix(self, drone_pose):
        # Camera view matrix
        return Matrix44.look_at(
            # eye: position of the camera in world coordinates
            drone_pose.translation,
            # target: position in world coordinates that the camera is looking at
            drone_pose.translation + (drone_pose.orientation *
                                           Vector3([1.0, 0.0, 0.0])),
            # up: up vector of the camera.
            drone_pose.orientation * Vector3([0.0, 0.0, 1.0]),
        )

    def __back_project(self, position, orientation, view):
        # Return if the camera is within 50cm of the gate, because it's not
        # visible
        if np.linalg.norm(position - self.drone_pose.translation) <= 0.3:
            return [-1, -1]

        clip_space_gate_center = self.projection * (view *
                                                    Vector4.from_vector3(position,
                                                                         w=1.0))
        if clip_space_gate_center.w != 0:
            normalized_device_coordinate_space_gate_center\
                = Vector3(clip_space_gate_center.xyz) / clip_space_gate_center.w
        else: # Clipped
            normalized_device_coordinate_space_gate_center = clip_space_gate_center.xyz

        viewOffset = 0
        image_frame_gate_center =\
            ((np.array(normalized_device_coordinate_space_gate_center.xy) + 1.0) /
             2.0) * np.array([self.width, self.height]) + viewOffset

        # Translate from bottom-left to top-left
        image_frame_gate_center[1] = self.height - image_frame_gate_center[1]

        return image_frame_gate_center



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generate a test dataset from\
                                     a given base dataset with\
                                     back-projections of the physical drone\
                                     racing gates coordinates on the image\
                                     plane.')
    parser.add_argument('base_dataset', help='the path to the base dataset',
                        type=str)
    parser.add_argument('destination', help='the path to the exported\
                        datgaset', type=str)
    parser.add_argument('config', help='the path to the configuration YAML\
                        file containing the gate positions', type=str)
    parser.add_argument('--camera', dest='camera_parameters', type=str,
                        help='the path to the camera parameters YAML file\
                        (output of OpenCV\'s calibration)',
                        required=True)

    args = parser.parse_args()
    annotator = Annotator(args.base_dataset, args.destination, args.config,
                          args.camera_parameters)
    annotator.run()
