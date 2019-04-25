from cv_bridge import CvBridge, CvBridgeError
from keras.models import model_from_json
import cv2
import numpy as np
import rospy

bridge = CvBridge()

def callback_img(data, target_size, rootpath):
    try:
        image_type = data.encoding
        img = bridge.imgmsg_to_cv2(data, image_type)
    except CvBridgeError, e:
        print e

    img = cv2.resize(img, target_size)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # TODO: RGB2GRAY ?

    return np.asarray(img, dtype=np.float32) * np.float32(1.0/255)

def jsonToModel(json_model_path):
    with open(json_model_path, 'r') as json_file:
        loaded_model_json = json_file.read()

    model = model_from_json(loaded_model_json)

    return model
