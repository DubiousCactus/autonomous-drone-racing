#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2019 transpalette <transpalette@arch-cactus>
#
# Distributed under terms of the MIT license.

"""
Dataset class, holding background images along with their annotations
"""

import os
import random

from queue import Queue
from tqdm import tqdm
from PIL import Image


class BackgroundImage:
    def __init__(self, image, annotations):
        self.file = image
        self.camera_height = annotations['height'] # In centimeters
        self.roll = annotations['roll']
        self.pitch = annotations['pitch']
        self.yaw = annotations['yaw']

    def image(self):
        return Image.open(self.file)


class Dataset:
    def __init__(self, path):
        if not os.path.isdir(path):
            raise Exception("Dataset directory not found")
        self.path = path
        self.width = None
        self.height = None

    def parse_annotations(self, path):
        if not os.path.isfile(path):
            raise Exception("Annotations file not found")
        # Example:
        annotations = {
            '0001.jpg':
                {'height': 100, 'roll': 0, 'pitch': 0, 'yaw': 0}
        }
        with open(path) as file:
            # TODO
            pass

        return annotations

    def load(self, annotations_path=None):
        print("[*] Loading and randomizing base dataset...")
        files = os.listdir(self.path)
        random.shuffle(files)
        annotations = self.parse_annotations(annotations_path)
        self.data = Queue()
        for file in tqdm(files):
            full_path = os.path.join(self.path, file)
            if os.path.isfile(full_path) and full_path != annotations_path:
                self.data.put(BackgroundImage(full_path, annotations['0001.jpg']))
                if not self.width and not self.height:
                    with Image.open(full_path) as img:
                        self.width, self.height = img.size

    '''
    Returns the next BackgroundImage in the Queue
    '''
    def get(self):
        return self.data.get()

    def get_image_size(self):
        print("[*] Using {}x{} resolution".format(self.width, self.height))
        return (self.width, self.height)
