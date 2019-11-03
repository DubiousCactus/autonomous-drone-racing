#!/usr/bin/env python3

from perception.msg import GatePrediction
from sensor_msgs.msg import CompressedImage
from keras.models import model_from_json
from Detector import Detector
from PIL import Image

import tensorflow as tf
import os, datetime
import rospy
import json


class Node():
    def __init__(self):
        rospy.init_node('detector', anonymous=True)
        config_path = rospy.get_param("~config_path")
        gate_pose_model_path = rospy.get_param("~gp_model_path")
        detector_weights_path = rospy.get_param("~d_weights_path")
        gate_pose_weights_path = rospy.get_param("~gp_weights_path")
        target_size = rospy.get_param("~target_size", '300,225').split(',')
        target_size = tuple([int(t) for t in target_size])
        with open(gate_pose_model_path, 'r') as jsonfile:
            # with tf.device("/cpu:0"):
            self.pose_estimator = model_from_json(json.load(jsonfile))
        print("[*] Loading weights from '{}'".format(gate_pose_weights_path))
        self.pose_estimator.load_weights(gate_pose_weights_path, by_name=True)
        global pose_graph
        pose_graph = tf.get_default_graph()
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        tf.keras.backend.set_session(tf.Session(config=config))
        self.network = Detector.Detector(config_path, detector_weights_path,
                                         target_size)
        rospy.Subscriber("camera", CompressedImage, self.image_callback,
                         queue_size=1, buff_size=52428800)
        self.predictorPublisher = rospy.Publisher("predictor/raw",
                                                  GatePrediction,
                                                  queue_size=20)
        self.filteredPredictorPublisher = rospy.Publisher("predictor/filtered",
                                                  GatePrediction,
                                                  queue_size=20)

    def image_callback(self, image):
        cropped, prediction, filtered_prediction = self.network.predict(image)
        msg = GatePrediction()
        msg.header.stamp = rospy.Time.now()
        if prediction is not None:
            with pose_graph.as_default():
                cropped /= 255.0
                distance_pred = self.pose_estimator.predict(cropped)
            msg.locked = True
            msg.bbox.minX = int(prediction[2])
            msg.bbox.minY = int(prediction[3])
            msg.bbox.maxX = int(prediction[4])
            msg.bbox.maxY = int(prediction[5])
            msg.distance = distance_pred
        else:
            msg.locked = False
        self.predictorPublisher.publish(msg)
        msg.bbox = filtered_prediction
        self.filteredPredictorPublisher.publish(msg)

    def run(self):
        print("[*] Running !")
        rospy.spin()


if __name__ == "__main__":
    node = Node()
    node.run()
