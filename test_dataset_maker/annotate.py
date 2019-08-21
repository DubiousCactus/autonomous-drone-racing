#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2019 theo <theo@not-arch-linux>
#
# Distributed under terms of the MIT license.

"""
Annotate a base dataset of images with camera poses, by projecting the
given gate position (in the world frame) on the image frame.
"""

from dataset import Dataset, AnnotatedImage
from pyrr import Matrix44, Vector3, Vector4, Quaternion
from math import atan2
from tqdm import *

import multiprocessing.dummy as mp
import numpy as np
import argparse
import yaml
import os


class Annotator():
    def __init__(self, base_dataset: str, dest: str, config: str,
                 camera_parameters: str, resolution: str):
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
        self.base_dataset.load(annotations_path=os.path.join(base_dataset,
                                                             "annotations.csv"),
                               randomize=False)
        if not resolution:
            self.width = self.base_dataset.width
            self.height = self.base_dataset.height
        else:
            self.width, self.height = int(resolution.split('x')[0]), int(resolution.split('x')[1])
        print("[*] Using {}x{} target resolution".format(self.width, self.height))
        self.annotated_dataset = Dataset(dest, max=300, verbose=True)
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
        zfar, znear = 100.0, 0.5 # distances to the clipping plane
        self.projection = Matrix44([
            [fx/cx, 0, 0, 0],
            [0, fy/cy, 0, 0],
            [0, 0, (-zfar - znear)/(zfar - znear), -1],
            [0, 0, (-2.0*zfar*znear)/(zfar - znear), 0]
        ])

    def run(self):
        print("[*] Generating dataset...")
        save_thread = mp.threading.Thread(
            target=self.annotated_dataset.save_json_live)
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
        for gate in self.gates_config['gates']:
            view = self.__compute_view_matrix(baseImage.annotations)
            coords = self.__compute_bbox_coords(gate, view)
            if coords != {}:
                distance = self.__compute_camera_proximity(gate,
                                                           baseImage.annotations)
                camera_yaw = self.__euler_yaw(baseImage.annotations.orientation)
                gate_yaw = self.__euler_yaw(Quaternion(gate['rotation']))
                rotation = camera_yaw - gate_yaw
                bbox = {
                    'class_id': 1,
                    'min': [coords['min'][0], coords['min'][1]],
                    'max': [coords['max'][0], coords['max'][1]],
                    'distance': distance,
                    'rotation': rotation
                }
                annotations.append(bbox)
        self.annotated_dataset.put(AnnotatedImage(baseImage.image_path(), index,
                                         annotations))

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

    def __euler_yaw(self, q):
        siny_cosp = 2.0 * ((q.w * q.z) + (q.x * q.y))
        cosy_cosp = 1.0 - (2.0 * ((q.y * q.y) + (q.z * q.z)))

        return atan2(siny_cosp, cosy_cosp) * (180.0/np.pi)

    '''
        Returns the Euclidean distance of the gate to the camera
    '''
    def __compute_camera_proximity(self, mesh, drone_pose):
        return np.linalg.norm((mesh['translation']) - drone_pose.translation)

    def __project_to_img_frame(self, vector, view):
        clip_space_vector = self.projection * (
            view * Vector4.from_vector3(vector, w=1.0))
        if clip_space_vector.w != 0:
            nds_vector = Vector3(clip_space_vector.xyz) / clip_space_vector.w
        else:  # Clipped
            nds_vector = clip_space_vector.xyz

        if nds_vector.z >= 1:
            return [-1, -1]

        viewOffset = 0
        image_frame_vector =\
            ((np.array(nds_vector.xy) + 1.0) /
             2.0) * np.array([self.width, self.height]) + viewOffset

        # Translate from bottom-left to top-left
        image_frame_vector[1] = self.height - image_frame_vector[1]

        return image_frame_vector


    '''
        Computes the bounding box min/max coordinates (diagonal corners) in the
        image frame, and clips them to the image borders if one of them is
        outside.  Otherwise, the gate is not visible.
    '''
    def __compute_bbox_coords(self, mesh, view):
        rotation = Quaternion(mesh['rotation'])
        center = mesh['translation']
        world_corners = {
            'top_left': Vector3([
                center[0],
                center[1],
                center[2]
            ]) + (rotation * Vector3([-mesh['width']/2, -mesh['width']/2,
                                      mesh['height']/2])),
            'top_right': Vector3([
                center[0],
                center[1],
                center[2]
            ]) + (rotation * Vector3([mesh['width']/2, mesh['width']/2,
                                      mesh['height']/2])),
            'bottom_right': Vector3([
                center[0],
                center[1],
                center[2]
            ]) + (rotation * Vector3([mesh['width']/2, mesh['width']/2,
                                      -mesh['height']/2])),
            'bottom_left': Vector3([
                center[0],
                center[1],
                center[2]
            ]) + (rotation * Vector3([-mesh['width']/2, -mesh['width']/2,
                                      -mesh['height']/2]))
        }

        hidden_corners = 0
        left = right = top = bottom = None

        for key, value in world_corners.items():
            img_coords = self.__project_to_img_frame(value, view)
            if (img_coords[0] < 10 or img_coords[0] > (self.width-10)
                    or img_coords[1] < 10 or img_coords[1] > (self.height-10)):
                hidden_corners += 1
            if left is None or (img_coords[0] < left['x']):
                left = {'x': img_coords[0], 'y': img_coords[1]}
            if top is None or (img_coords[1] < top['y']):
                top = {'x': img_coords[0], 'y': img_coords[1]}
            if bottom is None or (img_coords[1] > bottom['y']):
                bottom = {'x': img_coords[0], 'y': img_coords[1]}
            if right is None or (img_coords[0] > right['x']):
                right = {'x': img_coords[0], 'y': img_coords[1]}

        image_corners = {
            'min': [int(left['x']), int(top['y'])],
            'max': [int(right['x']), int(bottom['y'])]
        }

        if hidden_corners > 3:
            return {}
        elif hidden_corners > 0:
            for key, img_coords in image_corners.items():
                for i in range(0,2):
                    if img_coords[i] < 0:
                        img_coords[i] = 0
                    elif img_coords[i] > (self.width if i == 0 else self.height):
                        img_coords[i] = self.width if i == 0 else self.height
                image_corners[key] = img_coords

        return image_corners


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
    parser.add_argument('--res', dest='resolution', type=str, help='destination resolution')

    args = parser.parse_args()
    annotator = Annotator(args.base_dataset, args.destination, args.config,
                          args.camera_parameters, args.resolution)
    annotator.run()
