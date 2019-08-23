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
import json
import os

from tqdm import tqdm
from queue import Queue
from shutil import copyfile
from threading import Thread
from PIL import Image, ImageDraw
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


'''
Holds a test image along with its annotations
'''
class AnnotatedImage:
    def __init__(self, image_path: str, id, bboxes):
        self.image_path = image_path
        self.id = id
        self.bboxes = bboxes


class Dataset:
    def __init__(self, path: str, seed=None, max=0, verbose=False):
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
        self.annotations = dict()
        self.verbose = verbose

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

    def load(self, count=None, annotations_path=None, randomize=True, skip=None):
        print("[*] Loading and randomizing base dataset...")
        if randomize:
            files = os.listdir(self.path)
            random.shuffle(files)
        else:
            files = sorted(os.listdir(self.path))

        if skip:
            files = files[skip::]

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

    def draw_annotations(self, img, bboxes):
        gate_draw = ImageDraw.Draw(img)
        for i, bbox in enumerate(bboxes):
            color = 'green'
            xmin, ymin = bbox['min'][0], bbox['min'][1]
            xmax, ymax = bbox['max'][0], bbox['max'][1]
            gate_draw.rectangle([(xmin, ymin), (xmax, ymax)],
                                outline=color, width=2)
            label = "Candidate"
            textSize = gate_draw.textsize(label)
            gate_draw.rectangle((
                (xmin-2, ymin-2),
                (xmin+textSize[0]+2, ymin+textSize[1])),
                fill=color)
            gate_draw.text((xmin, ymin), label, fill='white')

    # Runs in a thread
    def save_json_live(self):
        if not self.saving:
            with open(os.path.join(self.path, 'annotations.json'),
                      'w', encoding='UTF-8') as f:
                annotations = {}
                annotations['classes'] = [
                    {'id': 0, 'label': 'Background'},
                    {'id': 1, 'label': 'Target 1'},
                    {'id': 2, 'label': 'Target 2'},
                    {'id': 3, 'label': 'Forward gate'},
                    {'id': 4, 'label': 'Backward gate'}
                ]
                annotations['annotations'] = []
                json.dump(annotations, f, ensure_ascii=False, indent=4)

            self.saving = True
            if not os.path.isdir(os.path.join(self.path, 'images')):
                os.mkdir(os.path.join(self.path, 'images'))

        for annotatedImage in iter(self.data.get, None):
            name = "%06d.png" % annotatedImage.id
            if self.verbose:
                img = Image.open(annotatedImage.image_path)
                self.draw_annotations(img, annotatedImage.bboxes)
                img.save(os.path.join(self.path, 'images', name))
            else:
                copyfile(annotatedImage.image_path,
                         os.path.join(self.path, 'images', name))
            bboxes = []
            for bbox in annotatedImage.bboxes:
                bboxes.append({
                    'class_id': bbox['class_id'],
                    'xmin': bbox['min'][0],
                    'ymin': bbox['min'][1],
                    'xmax': bbox['max'][0],
                    'ymax': bbox['max'][1],
                    'distance': bbox['distance'],
                    'rotation': bbox['rotation']
                })

            annotation = {
                'image': name,
                'annotations': bboxes
            }

            with open(os.path.join(self.path, 'annotations.json'),
                      'r', encoding='UTF-8') as f:
                data = json.load(f)

            data['annotations'].append(annotation)

            with open(os.path.join(self.path, 'annotations.json'),
                      'w', encoding='UTF-8') as f:
                json.dump(data, f, ensure_ascii=False, indent=4)

    def get_image_size(self):
        print("[*] Using {}x{} base resolution".format(self.width, self.height))
        return (self.width, self.height)

    def size(self):
        return self.count
