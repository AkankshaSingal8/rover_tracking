#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped


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
velocity_publisher = None

def image_callback(msg):
    global velocity_publisher
    global index
    global image_sequences
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
        
        image_sequences = np.vstack((image_sequences[0][1:], [cv_image]))
        print(image_sequences.shape)


    image_sequences = np.expand_dims(image_sequences, axis=0)
    index += 1

    with tf.device('/cpu:0'):
        preds = model.predict(image_sequences)
        print(preds[0][63])
        
        vx, vy, vz, omega_z = preds[0][63]
        vel_msg = Twist()

        # Set the velocities in the message
        vel_msg.linear.x = 2 * vx
        vel_msg.linear.y = 2 * vy
        vel_msg.linear.z = 2 * vz
        vel_msg.angular.z = 2 * omega_z

        velocity_publisher.publish(vel_msg)
        print("Velocity published:", vx, vy, vz, omega_z)


    # save_dir = "./image_feed"
    # if not os.path.exists(save_dir):
    #     os.makedirs(save_dir)

    # # Define the filename
    # filename = os.path.join(save_dir, "image_{}.png".format(time.time()))
    
    # # Save the image
    # cv2.imwrite(filename, cv_image)
    # rospy.loginfo("Image saved as: {}".format(filename))

def main():
    global velocity_publisher
    global index
    global image_sequences
    rospy.init_node('drone_raw_image_viewer', anonymous=True)
    # Subscribe to the raw image topic
    rospy.Subscriber("/cgo3_camera/image_raw", Image, image_callback)
    print("Subscribed")
    velocity_publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
    
    
    rospy.spin()
    

if __name__ == '__main__':
    main()
