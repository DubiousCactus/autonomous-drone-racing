#!/usr/bin/env python

import rospy
from Dronet import Dronet
import os, datetime

def run_network():

    rospy.init_node('dronet', anonymous=True)

    # LOAD ROS PARAMETERS
    json_model_path = rospy.get_param("~json_model_path")
    weights_model_path = rospy.get_param("~weights_path")
    # onboard_images_folder = rospy.get_param("~onboard_images_folder")
#     if onboard_images_folder:
        # imgs_rootpath = os.path.join(onboard_images_folder,
                 # datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))
        # os.makedirs(imgs_rootpath)
    # else:
#         imgs_rootpath = None
    target_size = rospy.get_param("~target_size", '340, 255').split(',')
    target_size = tuple([int(t) for t in target_size])

    # BUILD NETWORK CLASS
    network = Dronet.Dronet(json_model_path, weights_model_path, target_size)

    # RUN NETWORK
    network.run()

if __name__ == "__main__":
    run_network()