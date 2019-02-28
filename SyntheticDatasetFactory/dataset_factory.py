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

import argparse
import os

from tqdm import tqdm
from PIL import Image
from pyrr import Vector3
from dataset import Dataset
from scene_generator import SceneGenerator


'''
    ----- TODO -----

[ ] Match the perspective via camera height estimation (with camera
calibration)
[ ] WHY IS IT SO UGLY???!
[ ] Thread it!
[x] Random positioning of the gate
[x] Boundaries definition for the gate (relative to the mesh's size)
[x] Compute the center of the gate
[ ] Compute the presence of the gate in the image frame
[?] Compute the distance to the gate
[ ] Camera calibration (use the correct parameters)
[x] Project on transparent background
[x] Overlay with background image
[ ] Model the camera distortion
[ ] Apply the distortion to the OpenGL projection
[ ] Histogram equalization of both images (hue, saturation, luminence ?...)
[ ] Motion blur (shader ?)
[ ] Anti alisasing (shader ?)
[ ] Ship it!

'''


class DatasetFactory:
    def __init__(self, args):
        print(args)
        self.mesh_path = args.mesh
        self.count = args.nb_images
        self.blur_amount = args.blur_amount
        self.background_dataset = Dataset(args.dataset)
        self.background_dataset.load(args.annotations)
        self.generated_dataset = Dataset(args.destination)
        self.width, self.height = self.background_dataset.get_image_size()
        world_boundaries = {'x': 8, 'y': 10, 'z': 0} # Real world boundaries in meters (relative to the mesh's scale)
        gate_center = Vector3([0.0, 0.0, 2.1]) # Figure this out in Blender
        self.projector = SceneGenerator(self.mesh_path, self.width, self.height,
                                        world_boundaries, gate_center)

    def run(self):
        # TODO: Run in threads
        print("[*] Generating dataset...")
        for i in tqdm(range(self.count)):
            projection = self.projector.generate()
            background = self.background_dataset.get()
            output = self.combine(projection, background.image())
            output.show()

    def combine(self, projection, background):
        background = background.convert('RGBA')
        projection.thumbnail((self.width, self.height), Image.ANTIALIAS)
        output = Image.alpha_composite(background, projection)

        return output



if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Generate a hybrid synthetic dataset of projections of a \
        given 3D model, in random positions and orientations, onto randomly \
        selected background images from a given dataset.')
    parser.add_argument('mesh', help='the 3D mesh to project')
    parser.add_argument('dataset', help='the path to the background images \
                        dataset, with height, roll, pitch and yaw annotations')
    parser.add_argument('annotations', help='the path to the CSV annotations file')
    parser.add_argument('destination', metavar='dest', help='the path\
                        to the destination folder for the generated dataset')
    parser.add_argument('--count', dest='nb_images', default=10, type=int,
                        help='the number of images to be generated')
    parser.add_argument('--blur-amount', dest='blur_amount', default=0.3,
                        type=float, help='the percentage of motion blur to be \
                        added')

    datasetFactory = DatasetFactory(parser.parse_args())
    datasetFactory.run()
