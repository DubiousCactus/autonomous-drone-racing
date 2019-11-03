#!/usr/bin/env python3

from .models.keras_mobilenet_v2_ssdlite import mobilenet_v2_ssd
from sensor_msgs.msg import CompressedImage, Image
from .models.losses.keras_ssd_loss import SSDLoss
from keras.models import model_from_json
from keras import backend as K
from cv_bridge import CvBridge

import tensorflow as tf
import sensor_msgs
import numpy as np
import rospy
import yaml
from .img_utils import *


class Detector(object):
    def __init__(self, config_path, weights_path, prediction_size=(300, 225), filter_size=30):
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        self.bridge = CvBridge()
        self.filter_size = filter_size
        self.original_size = (640, 480)
        # self.visPublisher = rospy.Publisher("predictor/visualize",
                                            # sensor_msgs.msg.Image,
                                            # queue_size=100)
        self.model = mobilenet_v2_ssd(config, mode='inference')
        global graph
        graph = tf.get_default_graph()
        self.model.load_weights(weights_path, by_name=True)
        print("Loaded weights from {}".format(weights_path))
        ssdloss = SSDLoss(neg_pos_ratio=3, alpha=1.0)
        self.model.compile(loss=ssdloss.compute_loss, optimizer='adam')
        self.prediction_size = prediction_size
        self.previous_predictions = []

    def predict(self, image):
        colors = ['black', 'blue', 'purple', 'gren', 'red']
        classes = ['Background', 'Target 1', 'Target 2', 'Candidate', 'Backward gate']
        np_image, cv_image = callback_img(image, self.prediction_size)

        # TODO: Refactor this
        with graph.as_default():
            y_pred = self.model.predict(np_image)
            confidence_threshold = 0.5
            y_pred_thresh = [y_pred[k][y_pred[k,:,1] > confidence_threshold]
                             for k in range(y_pred.shape[0])]
            prediction = None
            for box in y_pred_thresh[0]:
                if int(box[0]) == 1:
                    prediction = box
                    break

        pred_filtered = prediction
        if prediction is not None:
            min_corner = (int(prediction[2]), int(prediction[3]))
            max_corner = (int(prediction[4]), int(prediction[5]))
            cropped = crop_and_pad(np_image, min_corner, max_corner,
                                   centered=False)
        else:
            cropped = None
#         if prediction is not None:
            # pred_filtered = median_filter(prediction, self.previous_predictions,
                                          # self.filter_size)
            # if len(self.previous_predictions) >= self.filter_size:
                # del self.previous_predictions[0]
#             self.previous_predictions.append(prediction)

        # vis = visualize(cv_image, y_pred_thresh, pred_filtered,
                              # self.original_size)
        # self.visPublisher.publish(self.bridge.cv2_to_imgmsg(vis,
                                                            # encoding="rgb8"))

        return cropped, prediction, pred_filtered
