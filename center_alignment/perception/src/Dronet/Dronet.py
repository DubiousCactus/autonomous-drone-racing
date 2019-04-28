#!/usr/bin/env python

from img_center_alignment.msg import GatePredictionMessage
from std_msgs.msg import Bool, Empty
from sensor_msgs.msg import Image, CompressedImage
from keras import backend as K

import rospy
import utils
import numpy as np


TEST_PHASE=0



class Dronet(object):
    def __init__(self,
                 json_model_path,
                 weights_path, target_size=(340, 255)):
        self.predictorPublisher = rospy.Publisher("predictor",
                                                  GatePredictionMessage,
                                                  queue_size=5)
#         self.feedthrough_sub = rospy.Subscriber("state_change", Bool,
                                                # self.callback_feedthrough,
#                                                 queue_size=1)
  #       self.land_sub = rospy.Subscriber("land", Empty, self.callback_land,
  #                                        queue_size=1)
        # self.imgs_rootpath = imgs_rootpath
        # Set keras utils
        K.set_learning_phase(TEST_PHASE)
        # Load json and create model
        print("[*] Loading model from {}".format(json_model_path))
        model = utils.jsonToModel(json_model_path)
        # Load weights
        model.load_weights(weights_path)
        print("Loaded model from {}".format(weights_path))
        model.compile(loss='mse', optimizer='sgd')
        self.model = model
        self.target_size = target_size

    # def callback_feedthrough(self, data):
        # self.use_network_out = data.data

    # def callback_land(self, data):
    #     self.use_network_out = False

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

            cv_image = utils.callback_img(data, self.target_size)
                                          # self.imgs_rootpath)
            outs = self.model.predict(cv_image)
            msg.window = np.argmax(outs[0])
            self.predictorPublisher.publish(msg)
