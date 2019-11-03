#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2019 theomorales <theomorales@air-admin>
#
# Distributed under terms of the MIT license.

"""
Utilities to help working with the hybrid drone racing dataset.
"""

import os
import sys
import json


from tqdm import trange
from utils.ssd_output_decoder import decode_detections
from utils.object_detection_2d_geometric_ops import Resize
from utils.object_detection_2d_misc import apply_inverse_transforms
from utils.object_detection_2d_photometric_ops import ConvertTo3Channels
from utils.object_detection_2d_patch_sampling_ops import RandomPadFixedAR


def get_hybrid_dataset_classes_map(datasets_dir):
    classes = {}
    annotations_filenames = ""

    for root, subdirs, files in os.walk(datasets_dir):
        if 'images' in subdirs and 'annotations.json' in files:
            annotations_filenames = os.path.join(datasets_dir, root, 'annotations.json')
            break

    with open(annotations_filenames, 'r') as f:
        annotations = json.load(f)
        classes = annotations['classes']

    return classes

