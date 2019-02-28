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
import argparse
import os

from PIL import Image
from pyrr import Vector3
from scene_generator import SceneGenerator
from dataset import Dataset, AnnotatedImage, SyntheticAnnotations


'''
    ----- TODO -----

[x] Thread it!
[x] Random positioning of the gate
[x] Boundaries definition for the gate (relative to the mesh's size)
[x] Compute the center of the gate
[ ] Compute the presence of the gate in the image frame
[?] Compute the distance to the gate
[x] Perspective projection for visualization
[ ] Convert world coordinates to image coordinates
[ ] Camera calibration (use the correct parameters)
[x] Project on transparent background
[x] Overlay with background image
[ ] Model the camera distortion
[ ] Apply the distortion to the OpenGL projection
[ ] Histogram equalization of both images (hue, saturation, luminence ?...)
[ ] Motion blur (shader ?)
[x] Anti alisasing
[ ] Ship it!

'''


class DatasetFactory:
    def __init__(self, args):
        self.mesh_path = args.mesh
        self.nb_threads = args.threads
        self.count = args.nb_images
        self.blur_amount = args.blur_amount
        self.background_dataset = Dataset(args.dataset)
        self.background_dataset.load(self.count, args.annotations)
        self.generated_dataset = Dataset(args.destination)
        self.base_width, self.base_height = self.background_dataset.get_image_size()
        self.target_width, self.target_height = [int(x) for x in args.resolution.split('x')]
        self.world_boundaries = {'x': 3, 'y': 3, 'z': 0} # Real world boundaries in meters (relative to the mesh's scale)
        self.gate_center = Vector3([0.0, 0.0, 2.1]) # Figure this out in Blender

    def run(self):
        print("[*] Generating dataset...")
        p = mp.Pool(self.nb_threads)
        p.map(self.generate, range(self.count))
        p.close()
        p.join()

        print("[*] Scaling to {}x{} resolution".format(self.target_width,
                                                       self.target_height))
        print("[*] Saving to {}".format(self.generated_dataset.path))
        self.generated_dataset.save(self.nb_threads)

    def generate(self, index):
        projector = SceneGenerator(self.mesh_path, self.base_width,
                                        self.base_height,
                                        self.world_boundaries, self.gate_center)
        projection, gate_center, rotation, visible = projector.generate()
        background = self.background_dataset.get()
        output = self.combine(projection, background.image())
        self.generated_dataset.put(
            AnnotatedImage(output, index, SyntheticAnnotations(gate_center,
                                                               rotation, visible))
        )
        # self.background_dataset.task_done()

    def combine(self, projection: Image, background: Image):
        background = background.convert('RGBA')
        projection.thumbnail((self.base_width, self.base_height), Image.ANTIALIAS)
        output = Image.alpha_composite(background, projection)
        output.thumbnail((self.target_width, self.target_height), Image.ANTIALIAS)

        return output



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
    parser.add_argument('--blur-amount', dest='blur_amount', default=0.3,
                        type=float, help='the percentage of motion blur to be \
                        added')
    parser.add_argument('--res', dest='resolution', default='640x480',
                        type=str, help='the desired resolution')
    parser.add_argument('-t', dest='threads', default=4, type=int,
                        help='the number of threads to use')

    datasetFactory = DatasetFactory(parser.parse_args())
    datasetFactory.run()
