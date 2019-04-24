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

from dataset import Dataset
from tqdm import *

import argparse
import yaml


class Annotator():
    def __init__(self, base_dataset: str, dest: str, config: str):
        with open(config, 'r') as conf:
            try:
                self.gates_config = yaml.safe_load(conf)
            except yaml.YAMLError as exc:
                raise Exception(exc)
        self.base_dataset = Dataset(base_dataset)
        self.base_dataset.load(os.path.join(base_dataset, "annotations.csv"),
                               randomize=False)
        self.annotated_dataset = Dataset(dest)

    def generate(self):
        for i in tqdm(range(self.base_dataset.size())):
            baseImage = self.base_dataset.get()
            baseImage.image().show()

