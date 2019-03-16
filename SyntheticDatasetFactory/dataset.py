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

import random
import os

from PIL import Image
from tqdm import tqdm
from queue import Queue
from threading import Thread
from pyrr import Vector3, Quaternion


class BackgroundAnnotations:
    def __init__(self, translation: Vector3, orientation: Quaternion):
        self.translation = translation
        self.orientation = orientation


'''
Holds a background image along with its annotations
'''
class BackgroundImage:
    def __init__(self, image_path: str, annotations: BackgroundAnnotations):
        self.file = image_path
        self.annotations = annotations

    def image(self):
        return Image.open(self.file)


class SyntheticAnnotations:
    def __init__(self, center, orientation, on_screen: bool):
        self.center = [int(x) for x in center]
        self.orientation = int(orientation)
        self.on_screen = 1 if on_screen else 0


'''
Holds a generated image along with its annotations
'''
class AnnotatedImage:
    def __init__(self, image: Image, id, annotations: SyntheticAnnotations):
        self.image = image
        self.id = id
        self.annotations = annotations


class Dataset:
    def __init__(self, path: str, seed=None):
        if not os.path.isdir(path):
            raise Exception("Dataset directory {} not found".format(path))
        if seed:
            random.seed(seed)
        else:
            random.seed()
        self.path = path
        self.width = None
        self.height = None
        self.data = Queue()

    def parse_annotations(self, path: str):
        if not os.path.isfile(path):
            raise Exception("Annotations file not found")
        annotations = dict()
        with open(path) as file:
            file.readline() # Discard the header
            for line in file:
                items = line.split(',')
                annotations[items[0].strip()] = BackgroundAnnotations(
                    Vector3([float(x) for x in items[1:4]]),
                    Quaternion([float(x) for x in items[4:8]])
                )

        return annotations

    def load(self, count, annotations_path=None, randomize=True):
        print("[*] Loading and randomizing base dataset...")
        files = os.listdir(self.path)
        if randomize:
            random.shuffle(files)

        while count >= len(files):
            choice = random.choice(files)
            full_path = os.path.join(self.path, choice)
            if os.path.isfile(full_path) and full_path != annotations_path:
                files += [choice]

        not_found = 0
        annotations = self.parse_annotations(annotations_path)
        for file in files:
            full_path = os.path.join(self.path, file)
            if os.path.isfile(full_path) and full_path != annotations_path:
                if file not in annotations:
                    not_found += 1
                    continue
                self.data.put(BackgroundImage(full_path, annotations[file]))
                self.data.task_done()
                if not self.width and not self.height:
                    with Image.open(full_path) as img:
                        self.width, self.height = img.size

        self.data.join()
        print("[!] {} annotations could not be found!".format(not_found))
        return self.data.qsize() != 0

    '''
    Returns the next BackgroundImage in the Queue
    '''
    def get(self):
        return self.data.get()

    def task_done(self):
        self.data.task_done()

    def put(self, image: AnnotatedImage):
        self.data.put(image)

    def _save_one(self):
        while not self.data.empty():
            annotatedImage = self.data.get()
            name = "%06d.png" % annotatedImage.id
            annotatedImage.image.save(
                os.path.join(self.path, name)
            )
            self.output_csv.write("{},{},{},{},{}\n".format(
                name,
                annotatedImage.annotations.center[0],
                annotatedImage.annotations.center[1],
                annotatedImage.annotations.orientation,
                annotatedImage.annotations.on_screen
            ))
            self.data.task_done()

    def save(self, nb_threads):
        self.output_csv = open(os.path.join(self.path,
                                            'annotations.csv'), 'w')
        self.output_csv\
            .write("frame,gate_center_x,gate_center_y,gate_rotation,gate_visible\n")
        for i in range(nb_threads):
            t = Thread(target=self._save_one)
            t.daemon = True
            t.start()

        self.data.join()
        self.output_csv.close()

    def get_image_size(self):
        print("[*] Using {}x{} resolution".format(self.width, self.height))
        return (self.width, self.height)
