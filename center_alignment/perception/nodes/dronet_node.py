#!/usr/bin/env python

import rospy
from Dronet import Dronet
import os, datetime

def run_network():

    rospy.init_node('dronet', anonymous=True)

    json_model_path = rospy.get_param("~json_model_path")
    weights_model_path = rospy.get_param("~weights_path")
    target_size = rospy.get_param("~target_size", '340,255').split(',')
    target_size = tuple([int(t) for t in target_size])

    network = Dronet.Dronet(json_model_path, weights_model_path, target_size)
    network.run()

if __name__ == "__main__":
    run_network()
