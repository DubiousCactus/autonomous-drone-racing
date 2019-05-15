#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2019 theomorales <theomorales@air-admin>
#
# Distributed under terms of the MIT license.

"""
Compute T-SNE and plot the results.
"""


from MulticoreTSNE import MulticoreTSNE as TSNE
from matplotlib import pyplot as plt

import numpy as np
import argparse
import random
import os


def load_dataset(path, nsamples):
    files = []
    samples = np.array((nsamples, width, height))
    for file in os.listdir(path):
        if os.path.isfile(os.path.join(path, file)):
            files.append(file)
    for file in random.sample(files, nsamples)

def compute(args):
    synthetic_gates = load_dataset(args.synthetic, args.nsamples)
    real_gates = load_dataset(args.real, args.nsamples)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    K:wq

    parser.add_argument('synthetic', description="Synthetic gates dataset path")
    parser.add_argument('real', description="Real gates dataset path")
    parser.add_argument('nsamples', description="Number of samples to plot")
    compute(parser.parse_args())

