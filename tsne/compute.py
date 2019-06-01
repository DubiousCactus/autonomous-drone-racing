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
from sklearn.cluster import KMeans
from sklearn.manifold import TSNE
from shutil import copyfile
from PIL import Image

import numpy as np
import argparse
import random
import os


def load_dataset(path, nsamples):
    files = []
    samples = dict()
    for file in os.listdir(path):
        if os.path.isfile(os.path.join(path, file)):
            files.append(os.path.join(path, file))
    for i, file in enumerate(random.sample(files, nsamples)):
        img = np.asarray(Image.open(file).convert("RGB"), dtype=np.uint8)
        samples[file] = img.flatten()

    return samples.keys(), np.array(samples.values())

def compute(args):
    random.seed(args.seed)
    synth_keys, synthetic_gates = load_dataset(args.synthetic, args.nsamples)
    real_keys, real_gates = load_dataset(args.real, args.nsamples)
    dims = 3 if args.d != 2 else 2

    fig = plt.figure()
    if dims == 3:
        ax = fig.add_subplot(111, projection='3d')

    synthetic_pca = PCA(n_components=30).fit_transform(synthetic_gates)
    synthetic_embeddings = TSNE(n_components=dims).fit_transform(synthetic_pca)
    synthetic_vis_x = synthetic_embeddings[:, 0]
    synthetic_vis_y = synthetic_embeddings[:, 1]

    if dims == 3:
        synthetic_vis_z = synthetic_embeddings[:, 2]
        synth_dots = ax.scatter(synthetic_vis_x, synthetic_vis_y,
                                synthetic_vis_z, c="red", marker='o')
    else:
        synth_dots = plt.scatter(synthetic_vis_x, synthetic_vis_y, c="red", marker='o')

    real_pca = PCA(n_components=30).fit_transform(real_gates)
    real_embeddings = TSNE(n_components=dims).fit_transform(real_pca)
    real_vis_x = real_embeddings[:, 0]
    real_vis_y = real_embeddings[:, 1]

    if dims == 3:
        real_vis_z = real_embeddings[:, 2]
        real_dots = ax.scatter(real_vis_x, real_vis_y, real_vis_z, c="blue",
                               marker='o')
    else:
        real_dots = plt.scatter(real_vis_x, real_vis_y, c="blue", marker='o')

    # Clustering
    if args.c:
        if args.c > 8:
            args.c = 8
        kmeans = KMeans(n_clusters=args.c).fit(zip(synthetic_vis_x, synthetic_vis_y))

        labels = {"A": "blue", "B": "orange", "C": "red", "D": "green", "E":
                  "purple", "F": "black", "G": "pink", "H": "brown"}
        for label, cluster in zip(labels.keys()[:args.c], kmeans.cluster_centers_):
            plt.annotate(label,
                     xy=cluster,
                     horizontalalignment='center',
                     verticalalignment='center',
                     size=20, weight='bold',
                     color='white',
                     backgroundcolor=labels[label])
            cluster_files = []
            for i, synth_point in enumerate(zip(synthetic_vis_x, synthetic_vis_y)):
                if kmeans.predict([synth_point]) == labels.keys().index(label):
                    cluster_files.append(synth_keys[i])

            for i, real_point in enumerate(zip(real_vis_x, real_vis_y)):
                if kmeans.predict([real_point]) == labels.keys().index(label):
                    cluster_files.append(real_keys[i])

            if not os.path.exists("cluster_{}".format(label)):
                os.mkdir("cluster_{}".format(label))

            for i, file in enumerate(random.sample(cluster_files, min(10, len(cluster_files)))):
                copyfile(file, "cluster_{}/{}.{}".format(label, i,
                                                         file.split('.')[1]))


    if dims == 2:
        plt.legend([synth_dots, real_dots], ["Synthetic", "Real"])

    plt.rc('pgf', rcfonts=False)
    plt.rc('pgf', texsystem='pdflatex')

    if dims == 3:
        plt.savefig('pca_t-sne_3d.png')
        plt.savefig('pca_t-sne_3d.pgf')
    else:
        plt.savefig('pca_t-sne_2d.png')
        plt.savefig('pca_t-sne_2d.pgf')




if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('synthetic', help="Synthetic gates dataset path")
    parser.add_argument('real', help="Real gates dataset path")
    parser.add_argument('nsamples', type=int, help="Number of samples to plot")
    parser.add_argument('-d', type=int, help='number of dimensions')
    parser.add_argument('-c', type=int, help='number of clusters')
    parser.add_argument('--seed', type=int, help='random seed')

    compute(parser.parse_args())

