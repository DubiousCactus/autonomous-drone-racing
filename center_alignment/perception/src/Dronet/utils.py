from cv_bridge import CvBridge, CvBridgeError
from keras.models import model_from_json
from PIL import Image, ImageDraw
from math import sqrt

import cv2
import rospy
import numpy as np

bridge = CvBridge()

def callback_img(data, target_size):
    img = cv2.imdecode(np.fromstring(data.data, dtype=np.uint8),
                       cv2.IMREAD_COLOR)
    original_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, target_size)
    img_array = np.asarray(img, dtype=np.float32)
    img_array *= (1.0/255)
    # img_array = np.expand_dims(img_array, axis=2)

    return np.expand_dims(img_array, axis=0), original_img


def median_filter(prediction, previous_predictions, nb_frames):
    if len(previous_predictions) < nb_frames:
        return prediction
    window = previous_predictions + [prediction]
    window.sort()
    return window[int(len(window)/2)]


def visualize(img, prediction, filtered_prediction, img_size):
    # img *= 255.0/img.max()
    np_array = np.uint8(img)
    img = Image.fromarray(np_array.reshape((np_array.shape[0],
                                           np_array.shape[1], np_array.shape[2])), mode="RGB")
    # img = img.convert("RGB")
    draw = ImageDraw.Draw(img)

    nb_windows = 25
    sqrt_win = int(sqrt(nb_windows))
    window_width = img_size[0] / sqrt_win
    window_height = img_size[1] / sqrt_win

    if filtered_prediction == 0:
        draw.text(((img.width / 2)-30, (img.height/2)-5), "NO GATE", "red")
    else:
        window_idx = filtered_prediction % sqrt_win
        if window_idx == 0:
            window_idx = sqrt_win
        window_x = (window_idx - 1) * window_width
        window_y = window_height * int(filtered_prediction/sqrt_win)
        draw.rectangle([(window_x, window_y),
                       (window_x + window_width, window_y + window_height)],
                       outline="green")

    if prediction == 0:
        draw.text(((img.width / 2)-30, (img.height/2)-5), "NO GATE", "red")
    else:
        # Draw a red square at the estimated region
        window_idx = prediction % sqrt_win
        if window_idx == 0:
            window_idx = sqrt_win
        window_x = (window_idx - 1) * window_width
        window_y = window_height * int(prediction/sqrt_win)
        draw.rectangle([(window_x, window_y),
                       (window_x + window_width, window_y + window_height)],
                       outline="red")


    return np.asarray(img, dtype=np.uint8)

def jsonToModel(json_model_path):
    with open(json_model_path, 'r') as json_file:
        loaded_model_json = json_file.read()

    model = model_from_json(loaded_model_json)

    return model
