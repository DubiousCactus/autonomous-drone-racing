from keras.models import model_from_json
from keras.preprocessing import image
from PIL import Image, ImageDraw
from math import sqrt

''' Fuck off ROS ! '''
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

import cv2
import rospy
import numpy as np

def callback_img(data, target_size):
    img = cv2.imdecode(np.fromstring(data.data, dtype=np.uint8),
                       cv2.IMREAD_COLOR)
    original_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, target_size)
    img_array = np.asarray(img, dtype=np.float32)

    return np.expand_dims(img_array, axis=0), original_img


'''
    Crops the given image over the given bounding box coordinates, and applies
    zero-padding to the rest of the image. The returned image in fact has the
    same dimensions as the given image
'''
def crop_and_pad(img, corner_min, corner_max, centered=False):
    img = img[0,:,:,:]
    if img.shape[2] > 1:
        img = np.dot(img[...,:3], [0.2989, 0.5870, 0.1140])
        img = np.expand_dims(img, axis=-1)
    cropped = np.zeros(img.shape, dtype=img.dtype)
    crop = img[corner_min[1]:corner_max[1], corner_min[0]:corner_max[0],:]
    if centered:
        startW = int((img.shape[1] - crop.shape[1]) / 2)
        startH = int((img.shape[0] - crop.shape[0]) / 2)
        cropped[startH:startH+crop.shape[0], startW:startW+crop.shape[1],:] = crop
    else:
        cropped[corner_min[1]:corner_max[1], corner_min[0]:corner_max[0],:] = crop
    cropped = np.expand_dims(cropped, axis=0)
    return cropped


def median_filter(prediction, previous_predictions, nb_frames):
    if len(previous_predictions) < nb_frames:
        return prediction
    coords = [prev_pred[i] + pred[i] for i in range(2, len(prediction)) for prev_pred in previous_predictions]
    [x.sort() for x in coords]
    filtered = [coord[int(len(coord)/2)] for coord in coords]

    return filtered


def visualize(img, prediction, filtered_prediction, img_size):
    colors = ['black', 'blue', 'purple', 'green', 'red']
    classes = ['Background', 'Target 1', 'Target 2',  'Candidate', 'Forward gate']
    np_array = np.uint8(img)
    img = Image.fromarray(np_array.reshape((np_array.shape[0],
                                            np_array.shape[1],
                                            np_array.shape[2])), mode="RGB")
    draw = ImageDraw.Draw(img)

    # TODO: Draw the filtered prediction
    for box in prediction[0]:
        xmin, ymin, xmax, ymax = box[2], box[3], box[4], box[5]
        if int(box[0]) is not 0:
            color = colors[int(box[0])]
            label = '{}: {:.2f}'.format(classes[int(box[0])], box[1])
            draw.rectangle(((xmin, ymin), (xmax, ymax)), outline=color, width=2)
            textSize = draw.textsize(label)
            draw.rectangle((
                (xmin-2, ymin-2),
                (xmin+textSize[0]+2, ymin+textSize[1])),
                fill=color)
            draw.text((xmin, ymin), label, fill='white')

    return np.asarray(img, dtype=np.uint8)


def jsonToModel(json_model_path):
    with open(json_model_path, 'r') as json_file:
        loaded_model_json = json_file.read()

    model = model_from_json(loaded_model_json)

    return model
