#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2019 transpalette <transpalette@arch-cactus>
#
# Distributed under terms of the MIT license.

"""
DatasetFactory

Generates a given number of images by projecting a given model in random
positions, onto randomly selected background images from the given dataset.
"""

import multiprocessing.dummy as mp
import numpy as np
import argparse
import cv2
import sys
import os

from tqdm import *
from pyrr import Vector3
from itertools import repeat
from PIL import Image, ImageDraw
from skimage.util import random_noise
from scene_renderer import SceneRenderer
from dataset import Dataset, BackgroundImage, AnnotatedImage, SyntheticAnnotations


'''
    ----- TODO -----

[x] Thread it!
[x] Random positioning of the gate
[x] Boundaries definition for the gate (relative to the mesh's size)
[x] Compute the center of the gate
[x] Compute the presence of the gate in the image frame
[x] Convert world coordinates to image coordinates
[?] Compute the distance to the gate
[x] Perspective projection for visualization
[x] Camera calibration (use the correct parameters)
[x] Project on transparent background
[x] Overlay with background image
[x] Model the camera distortion
[x] Save output every N samples to avoid crash
[ ] Add background gates
[ ] Compute gate orientation with respect to the camera
[x] Save annotations
[ ] Apply the distortion to the OpenGL projection
[ ] Histogram equalization of both images (hue, saturation, luminence ?...)
[x] Motion blur
[x] Anti alisasing
[ ] Ship it!

'''


class DatasetFactory:
    def __init__(self, args):
        self.mesh_path = args.mesh
        self.nb_threads = args.threads
        self.count = args.nb_images
        self.cam_param = args.camera_parameters
        self.verbose = args.verbose
        self.render_perspective = args.extra_verbose
        self.max_blur_amount = args.blur_threshold
        self.noise_amount = args.noise_amount
        self.no_blur = args.no_blur
        self.seed = args.seed
        if self.render_perspective:
            self.verbose = True
        self.background_dataset = Dataset(args.dataset, args.seed)
        if not self.background_dataset.load(self.count, args.annotations):
            print("[!] Could not load dataset!")
            sys.exit(1)
        self.generated_dataset = Dataset(args.destination, max=30)
        self.base_width, self.base_height = self.background_dataset.get_image_size()
        self.target_width, self.target_height = [int(x) for x in args.resolution.split('x')]
        self.sample_no = 0

    def set_mesh_parameters(self, boundaries, gate_center):
        self.world_boundaries = boundaries
        self.gate_center = gate_center

    def run(self):
        print("[*] Generating dataset...")
        generation_done_event = mp.Event()
        save_thread = mp.threading.Thread(target=self.generated_dataset.save,
                             args=(generation_done_event,))
        projector = SceneRenderer(self.mesh_path, self.base_width,
                                   self.base_height, self.world_boundaries,
                                   self.gate_center, self.cam_param,
                                  self.render_perspective, self.seed)
        save_thread.start()
        for i in tqdm(range(self.count)):
            self.generate(i, projector)

        # generation_done_event.set()
        self.generated_dataset.data.put(None)
        save_thread.join()
        print("[*] Saved to {}".format(self.generated_dataset.path))
#         with mp.Pool(self.nb_threads) as p:
            # max_ = self.count
            # with tqdm(total=max_) as pbar:
                # save_thread.start()
                # for i, _ in tqdm(enumerate(p.imap_unordered(self.generate, range(max_)))):
                    # pbar.update()
                # p.close()
                # p.join()
                # self.generated_dataset.data.join()
                # generation_done_event.set()
                # save_thread.join()
#                 print("[*] Saved to {}".format(self.generated_dataset.path))

    def generate(self, index, projector):
        background = self.background_dataset.get()
        projector.set_drone_pose(background.annotations)
        projection, annotations = projector.generate()
        projection_blurred = self.apply_motion_blur(projection,
                                                    amount=self.get_blur_amount(background.image()))
        projection_noised = self.add_noise(projection_blurred)
        output = self.combine(projection_noised, background.image())
        gate_center = self.scale_coordinates( annotations['gate_center_img_frame'], output.size)
        gate_visible = (gate_center[0] >=0 and gate_center[0] <=
                        output.size[0]) and (gate_center[1] >= 0 and
                                             gate_center[1] <= output.size[1])
        if self.verbose:
            self.draw_gate_center(output, gate_center)
            self.draw_image_annotations(output, annotations)

        self.generated_dataset.put(
            AnnotatedImage(output, index, SyntheticAnnotations(gate_center,
                                                               annotations['gate_rotation'],
                                                               gate_visible)))
        # projector.destroy()
        # del projector

    # Scale to target width/height
    def scale_coordinates(self, coordinates, target_coordinates):
        coordinates[0] = (coordinates[0] * target_coordinates[0]) / self.base_width
        coordinates[1] = (coordinates[1] * target_coordinates[1]) / self.base_height

        return coordinates

    # NB: Thumbnail() only scales down!!
    def combine(self, projection: Image, background: Image):
        background = background.convert('RGBA')
        if projection.size != (self.base_width, self.base_height):
            projection.thumbnail((self.base_width, self.base_height), Image.ANTIALIAS)
        output = Image.alpha_composite(background, projection)
        if output.size != (self.target_width, self.target_height):
            output.thumbnail((self.target_width, self.target_height), Image.ANTIALIAS)

        return output

    def get_blur_amount(self, img: Image):
        gray_scale = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2GRAY)
        variance_of_laplacian = cv2.Laplacian(gray_scale, cv2.CV_64F).var()
        blur_amount = variance_of_laplacian / self.max_blur_amount
        if blur_amount > 1:
            blur_amount = 0.9

        return 1 - blur_amount

    def equalize_histograms(self, img: Image, bg: Image):
        pass

    def add_noise(self, img):
        noisy_img = random_noise(img, mode='gaussian', var=self.noise_amount**2)
        noisy_img = (255*noisy_img).astype(np.uint8)

        return Image.fromarray(noisy_img)

    def apply_motion_blur(self, img: Image, amount=0.5):
        cv_img = np.array(img)

        if self.no_blur:
            return cv_img

        if amount <= 0.3:
            size = 3
        elif amount <= 0.5:
            size = 5
        else:
            size = 9
        kernel = np.identity(size)
        kernel /= size

        return cv2.filter2D(cv_img, -1, kernel)

    def draw_gate_center(self, img, coordinates, color=(0, 255, 0, 255)):
        gate_draw = ImageDraw.Draw(img)
        gate_draw.line((coordinates[0] - 10, coordinates[1], coordinates[0] + 10,
                   coordinates[1]), fill=color)
        gate_draw.line((coordinates[0], coordinates[1] - 10, coordinates[0],
                   coordinates[1] + 10), fill=color)

    def draw_image_annotations(self, img, annotations, color=(0, 255, 0, 255)):
        text = "gate_center_image_frame: {}\ngate_position: {}\ngate_rotation: {}\ndrone_pose: {}\ndrone_orientation:{}".format(
            annotations['gate_center_img_frame'], annotations['gate_position'],
                annotations['gate_rotation'], annotations['drone_pose'],
                annotations['drone_orientation'])
        text_draw = ImageDraw.Draw(img)
        text_draw.text((0, 0), text, color)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Generate a hybrid synthetic dataset of projections of a \
        given 3D model, in random positions and orientations, onto randomly \
        selected background images from a given dataset.')
    parser.add_argument('mesh', help='the 3D mesh to project', type=str)
    parser.add_argument('dataset', help='the path to the background images \
                        dataset, with height, roll, pitch and yaw annotations',
                       type=str)
    parser.add_argument('annotations', help='the path to the CSV annotations\
                        file', type=str)
    parser.add_argument('destination', metavar='dest', help='the path\
                        to the destination folder for the generated dataset',
                        type=str)
    parser.add_argument('--count', dest='nb_images', default=5, type=int,
                        help='the number of images to be generated')
    parser.add_argument('--res', dest='resolution', default='640x480',
                        type=str, help='the desired resolution')
    parser.add_argument('-t', dest='threads', default=4, type=int,
                        help='the number of threads to use')
    parser.add_argument('--camera', dest='camera_parameters', type=str,
                        help='the path to the camera parameters YAML file',
                        required=True)
    parser.add_argument('-v', dest='verbose', help='verbose output',
                        action='store_true', default=False)
    parser.add_argument('-vv', dest='extra_verbose', help='extra verbose\
                        output (render the perspective grid)',
                        action='store_true', default=False)
    parser.add_argument('--seed', dest='seed', default=None, help='use a fixed seed')
    parser.add_argument('--blur', dest='blur_threshold', default=200, type=int,
                        help='the blur threshold')
    parser.add_argument('--noise', dest='noise_amount', default=0.025,
                        type=float, help='the gaussian noise amount')
    parser.add_argument('--no-blur', dest='no_blur', action='store_true',
                        default=False, help='disable synthetic motion blur')

    datasetFactory = DatasetFactory(parser.parse_args())
    datasetFactory.set_mesh_parameters(
        {'x': 10, 'y': 10}, # Real world boundaries in meters (relative to the mesh's scale)
        Vector3([0.0, 0.0, 2.3]) # Figure this out in Blender
    )
    datasetFactory.run()
