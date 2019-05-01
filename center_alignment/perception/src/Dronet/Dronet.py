#!/usr/bin/env python

from perception.msg import GatePredictionMessage
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Bool, Empty
from keras import backend as K
from cv_bridge import CvBridge

import numpy as np
import rospy
import utils


TEST_PHASE=0



class Dronet(object):
    def __init__(self,
                 json_model_path,
                 weights_path, target_size=(340, 255), filter_size=30):
        self.bridge = CvBridge()
        self.filter_size = filter_size
        self.predictorPublisher = rospy.Publisher("predictor/raw",
                                                  GatePredictionMessage,
                                                  queue_size=5)
        self.filteredPredictorPublisher = rospy.Publisher("predictor/filtered",
                                                  GatePredictionMessage,
                                                  queue_size=5)
        self.visPublisher = rospy.Publisher("predictor/visualize", Image,
                                            queue_size=5)
        K.set_learning_phase(TEST_PHASE)
        print("[*] Loading model from {}".format(json_model_path))
        model = utils.jsonToModel(json_model_path)
        model.load_weights(weights_path)
        print("Loaded model from {}".format(weights_path))
        model.compile(loss='mse', optimizer='sgd')
        self.model = model
        self.target_size = target_size
        self.previous_predictions = []

    def run(self):
        print("[*] Running !")
        while not rospy.is_shutdown():
            msg = GatePredictionMessage()
            msg.header.stamp = rospy.Time.now()
            data = None
            while data is None:
                try:
                    data = rospy.wait_for_message("camera", CompressedImage, timeout=10)
                except:
                    pass

            np_image, cv_image = utils.callback_img(data, self.target_size)
            prediction = np.argmax(self.model.predict(np_image))
            msg.window = prediction
            self.predictorPublisher.publish(msg)

            pred_filtered = utils.median_filter(prediction,
                                                self.previous_predictions,
                                                self.filter_size)
            if len(self.previous_predictions) >= self.filter_size:
                del self.previous_predictions[0]
            self.previous_predictions.append(prediction)
            msg.window = pred_filtered
            self.filteredPredictorPublisher.publish(msg)

            vis = utils.visualize(cv_image, prediction, pred_filtered, self.target_size)
            self.visPublisher.publish(self.bridge.cv2_to_imgmsg(vis,
                                                                encoding="rgb8"))
