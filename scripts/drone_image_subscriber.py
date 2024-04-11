#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import time


from typing import Iterable, Dict
import tensorflow as tf
import kerasncp as kncp
from kerasncp.tf import LTCCell, WiredCfcCell
from tensorflow import keras
import numpy as np
from matplotlib.image import imread
import pandas as pd

IMAGE_SHAPE = (144, 256, 3)
IMAGE_SHAPE_CV = (IMAGE_SHAPE[1], IMAGE_SHAPE[0])

tf.config.set_visible_devices([], 'GPU')

print(os.getcwd())
with tf.device('/cpu:0'):
    model = tf.keras.models.load_model('model_ssfalse_b64_lr0.0001wscheduler_seqlen64_new_dataset.h5')

predictions = []

image_sequences = []
index = 0

def image_callback(msg):
    print("entering_function")
    bridge = CvBridge()
    try:
        # Convert to OpenCV image format
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv_image = cv2.resize(cv_image, (IMAGE_SHAPE[1], IMAGE_SHAPE[0])) 
        print("converted and resized to cv img")
    except CvBridgeError as e:
        print(e)
    
    if index == 0:
        image_sequences = np.stack([cv_image] * 64)
        
        print(image_sequences.shape)
        
    else:
        
        image_sequences = image_sequences[0][1:] + [cv_image]
        
    image_sequences = np.expand_dims(image_sequences, axis=0)

    with tf.device('/cpu:0'):
        print(model.predict(image_sequences))
        

    # save_dir = "./image_feed"
    # if not os.path.exists(save_dir):
    #     os.makedirs(save_dir)

    # # Define the filename
    # filename = os.path.join(save_dir, "image_{}.png".format(time.time()))
    
    # # Save the image
    # cv2.imwrite(filename, cv_image)
    # rospy.loginfo("Image saved as: {}".format(filename))

def main():
    rospy.init_node('drone_raw_image_viewer', anonymous=True)
    # Subscribe to the raw image topic
    rospy.Subscriber("/cgo3_camera/image_raw", Image, image_callback)
    print("Subscribed")
    
    rospy.spin()
    

if __name__ == '__main__':
    main()
