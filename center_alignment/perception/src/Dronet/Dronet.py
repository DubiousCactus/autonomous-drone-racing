#!/usr/bin/env python

from sensor_msgs.msg import CompressedImage, Image
from keras import backend as K
from cv_bridge import CvBridge

import tensorflow as tf
import numpy as np
import rospy
import utils


class Dronet(object):
    def __init__(self, json_model_path, weights_path, target_size=(340, 255),
                 filter_size=30):
        self.bridge = CvBridge()
        self.filter_size = filter_size
        self.original_size = (640, 480)
        self.visPublisher = rospy.Publisher("predictor/visualize", Image,
                                            queue_size=100)
        K.set_learning_phase(0)
        print("[*] Loading model from {}".format(json_model_path))
        self.model = utils.jsonToModel(json_model_path)
        self.model.load_weights(weights_path)
        print("Loaded model from {}".format(weights_path))
        self.model.compile(loss='mse', optimizer='sgd')
        self.graph = tf.get_default_graph()
        self.target_size = target_size
        self.previous_predictions = []

    def predict(self, image):
        np_image, cv_image = utils.callback_img(image, self.target_size)
        with self.graph.as_default():
            prediction = np.argmax(self.model.predict(np_image))

        pred_filtered = utils.median_filter(prediction,
                                            self.previous_predictions,
                                            self.filter_size)
        if len(self.previous_predictions) >= self.filter_size:
            del self.previous_predictions[0]
        self.previous_predictions.append(prediction)

        vis = utils.visualize(cv_image, prediction, pred_filtered,
                              self.original_size)
        self.visPublisher.publish(self.bridge.cv2_to_imgmsg(vis,
                                                            encoding="rgb8"))

        return prediction, pred_filtered
