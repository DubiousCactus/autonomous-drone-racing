#!/usr/bin/env python

from perception.msg import GatePrediction
from sensor_msgs.msg import CompressedImage
from Dronet import Dronet

import os, datetime
import rospy


class Node():
    def __init__(self):
        rospy.init_node('dronet', anonymous=True)
        json_model_path = rospy.get_param("~json_model_path")
        weights_model_path = rospy.get_param("~weights_path")
        target_size = rospy.get_param("~target_size", '340,255').split(',')
        target_size = tuple([int(t) for t in target_size])
        self.network = Dronet.Dronet(json_model_path, weights_model_path, target_size)
        rospy.Subscriber("camera", CompressedImage, self.image_callback)
        self.predictorPublisher = rospy.Publisher("predictor/raw",
                                                  GatePrediction,
                                                  queue_size=100)
        self.filteredPredictorPublisher = rospy.Publisher("predictor/filtered",
                                                  GatePrediction,
                                                  queue_size=100)

    def image_callback(self, image):
        prediction, filtered_prediction = self.network.predict(image)
        msg = GatePrediction()
        msg.header.stamp = rospy.Time.now()
        msg.window = prediction
        self.predictorPublisher.publish(msg)
        msg.window = filtered_prediction
        self.filteredPredictorPublisher.publish(msg)

    def run(self):
        print("[*] Running !")
        rospy.spin()


if __name__ == "__main__":
    node = Node()
    node.run()
