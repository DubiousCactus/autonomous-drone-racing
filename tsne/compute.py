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


# from MulticoreTSNE import MulticoreTSNE as TSNE
from mpl_toolkits.mplot3d import Axes3D
from sklearn.decomposition import PCA
from matplotlib import pyplot as plt
from sklearn.manifold import TSNE
from PIL import Image

import numpy as np
import argparse
import random
import os


def load_dataset(path, nsamples):
    files = []
    samples = []
    for file in os.listdir(path):
        if os.path.isfile(os.path.join(path, file)):
            files.append(os.path.join(path, file))
    for i, file in enumerate(random.sample(files, nsamples)):
        img = np.asarray(Image.open(file).convert("RGB"), dtype=np.uint8)
        samples.append(img.flatten())

    return np.array(samples)

def compute(args):
    synthetic_gates = load_dataset(args.synthetic, args.nsamples)
    real_gates = load_dataset(args.real, args.nsamples)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    synthetic_pca = PCA(n_components=50).fit_transform(synthetic_gates)
    synthetic_embeddings = TSNE(n_components=3).fit_transform(synthetic_pca)
    synthetic_vis_x = synthetic_embeddings[:, 0]
    synthetic_vis_y = synthetic_embeddings[:, 1]
    synthetic_vis_z = synthetic_embeddings[:, 2]
    ax.scatter(synthetic_vis_x, synthetic_vis_y, synthetic_vis_z, c="red",
                marker='o')

    real_pca = PCA(n_components=50).fit_transform(real_gates)
    real_embeddings = TSNE(n_components=3).fit_transform(real_pca)
    real_vis_x = real_embeddings[:, 0]
    real_vis_y = real_embeddings[:, 1]
    real_vis_z = real_embeddings[:, 2]
    ax.scatter(real_vis_x, real_vis_y, real_vis_z, c="blue", marker='+')

    # plt.colorbar(ticks=range(10))
    print("Showing the plot!")
    # plt.clim(-0.5, 9.5)
    # plt.show()
    plt.savefig('pca_t-sne.png')


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('synthetic', help="Synthetic gates dataset path")
    parser.add_argument('real', help="Real gates dataset path")
    parser.add_argument('nsamples', type=int, help="Number of samples to plot")
    compute(parser.parse_args())

