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

class BackgroundImage:
    def __init__(self, image, annotations)
        self.file = image,
        self.camera_height = annotations['height'] # In centimeters
        self.roll = annotations['roll']
        self.pitch = annotations['pitch']
        self.yaw = annotations['yaw']


class Dataset:
    def __init__(self, path, annotations_path):
        print("[*] Loading and randomizing dataset...")
        self.annotations = self.parse_annotations(annotations_path)
        self.data = self.load(path)

    def parse_annotations(self, path):
        # Example:
        annotations = [{
            'img0001.png':
                {'height': 100, roll: 0, pitch: 0, yaw: 0}
        }]
        with open(path) as file:
            # TODO

        return annotations

    def load(self, path):
        data = Queue()
        for file in tqdm(inos.listdir(dataset_path):
            full_path = os.path.join(dataset_path, file)
            if os.path.isfile(full_path) and full_path != self.annotations_path:
                data.put(BackgroundImage(file, self.annotations[file]))

        return data
