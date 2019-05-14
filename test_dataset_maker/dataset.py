#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2019 Theo Morales <theo.morales.fr@gmail.com>
#
# Distributed under terms of the GPLv3 license.

"""
Dataset class
"""

import random
import os

from PIL import Image
from tqdm import tqdm
from queue import Queue
from threading import Thread
from pyrr import Vector3, Quaternion


class BaseAnnotations:
    def __init__(self, translation: Vector3, orientation: Quaternion):
        self.translation = translation
        self.orientation = orientation


'''
Holds a base image along with its annotations
'''
class BaseImage:
    def __init__(self, image_path: str, annotations: BaseAnnotations):
        self.file = image_path
        self.annotations = annotations

    def image_path(self):
        return self.file


class TestAnnotations:
    def __init__(self, center, orientation: Quaternion, distance: float, on_screen: bool):
        self.center = [int(x) for x in center]
        self.orientation = orientation
        self.distance = distance
        self.on_screen = 1 if on_screen else 0


'''
Holds a test image along with its annotations
'''
class AnnotatedImage:
    def __init__(self, image_path: str, id, annotations: [TestAnnotations]):
        self.image_path = image_path
        self.id = id
        self.candidates = annotations


class Dataset:
    def __init__(self, path: str, seed=None, max=0):
        if not os.path.isdir(path):
            raise Exception("Dataset directory {} not found".format(path))
        if seed:
            random.seed(seed)
        else:
            random.seed()
        self.path = path
        self.width = None
        self.height = None
        self.data = Queue(maxsize=max)
        self.saving = False
        self.count = 0

    def parse_annotations(self, path: str):
        if not os.path.isfile(path):
            raise Exception("Annotations file not found")
        annotations = dict()
        with open(path) as file:
            file.readline() # Discard the header
            for line in file:
                items = line.split(',')
                annotations[items[0].strip()] = BaseAnnotations(
                    Vector3([float(x) for x in items[1:4]]),
                    Quaternion([float(x) for x in items[4:8]])
                )

        return annotations

    def load(self, count=None, annotations_path=None, randomize=True):
        print("[*] Loading and randomizing base dataset...")
        if randomize:
            files = os.listdir(self.path)
            random.shuffle(files)
        else:
            files = sorted(os.listdir(self.path))

        annotations = self.parse_annotations(annotations_path)
        # Remove files without annotations
        files = [file for file in files if file in annotations]
        if count is not None:
            while count > len(files):
                choice = random.choice(files)
                full_path = os.path.join(self.path, choice)
                if os.path.isfile(full_path) and full_path != annotations_path:
                    files += [choice]

        for file in files:
            full_path = os.path.join(self.path, file)
            if os.path.isfile(full_path) and full_path != annotations_path:
                self.count += 1
                self.data.put(BaseImage(full_path, annotations[file]))
                self.data.task_done()
                if not self.width and not self.height:
                    with Image.open(full_path) as img:
                        self.width, self.height = img.size

        self.data.join()
        return self.data.qsize() != 0

    '''
    Returns the next BaseImage in the Queue
    '''
    def get(self):
        return self.data.get()

    def task_done(self):
        self.data.task_done()

    def put(self, image: AnnotatedImage):
        self.data.put(image)

    # Runs in a thread
    def save(self):
        if not self.saving:
            self.output_csv = open(os.path.join(self.path,
                                                'annotations.csv'), 'w')
            self.output_csv.write(
                "frame,gate_center_x,gate_center_y,gate_rotation_x,gate_rotation_y,gate_rotation_z,gate_rotation_w,gate_distance,gate_visible\n")
            self.saving = True
            if not os.path.isdir(os.path.join(self.path, 'images')):
                os.mkdir(os.path.join(self.path, 'images'))

        for annotatedImage in iter(self.data.get, None):
            name = "%06d.png" % annotatedImage.id
            Image.open(annotatedImage.image_path).save(
                os.path.join(self.path, 'images', name)
            )
            for candidate in annotatedImage.candidates:
                self.output_csv.write("{},{},{},{},{},{},{},{},{}\n".format(
                    name,
                    candidate.center[0],
                    candidate.center[1],
                    candidate.orientation.x,
                    candidate.orientation.y,
                    candidate.orientation.z,
                    candidate.orientation.w,
                    candidate.distance,
                    candidate.on_screen
                ))
                self.output_csv.flush()
        self.output_csv.close()

    def get_image_size(self):
        print("[*] Using {}x{} base resolution".format(self.width, self.height))
        return (self.width, self.height)

    def size(self):
        return self.count
