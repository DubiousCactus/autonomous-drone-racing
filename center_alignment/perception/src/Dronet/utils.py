from cv_bridge import CvBridge, CvBridgeError
from keras.models import model_from_json
import cv2
import numpy as np
import rospy

bridge = CvBridge()

def callback_img(data, target_size):
    # try:
        # image_type = data.encoding
        # img = bridge.imgmsg_to_cv2(data, image_type)
        # img = data
    # except CvBridgeError, e:
        # print e

    img = cv2.imdecode(np.fromstring(data.data, dtype=np.uint8),
                       cv2.IMREAD_COLOR)
    img = cv2.resize(img, target_size)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY) # TODO: RGB2GRAY ?
    img_array = np.asarray(img, dtype=np.float32) #* np.float32(1.0/255)
    img_array *= (1.0/255)
    img_array = np.expand_dims(img_array, axis=2)

    return np.expand_dims(img_array, axis=0)

def jsonToModel(json_model_path):
    with open(json_model_path, 'r') as json_file:
        loaded_model_json = json_file.read()

    model = model_from_json(loaded_model_json)

    return model
