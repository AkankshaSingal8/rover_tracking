#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import time
from mavros_msgs.srv import CommandBool, SetMode

from typing import Iterable, Dict
import tensorflow as tf
import kerasncp as kncp
from kerasncp.tf import LTCCell, WiredCfcCell
from tensorflow import keras
import numpy as np
from matplotlib.image import imread
import pandas as pd


current_state = State()
current_pose = PoseStamped()

MODEL_MODE = False
VEL = False
CV_IMAGE = None
INDEX = 0
image_sequences = []
vel_msg = Twist()
COUNT = 0

IMAGE_SHAPE = (144, 256, 3)
IMAGE_SHAPE_CV = (IMAGE_SHAPE[1], IMAGE_SHAPE[0])


tf.config.set_visible_devices([], 'GPU')

rospy.loginfo("current working dir")
rospy.loginfo(os.getcwd())
root = '/home/akanksha/catkin_ws/src/rover_tracking/scripts/model_ssfalse_b64_lr0.0001wscheduler_seqlen64_new_dataset.h5'
with tf.device('/cpu:0'):
    model = tf.keras.models.load_model(root)

def state_cb(msg):
    global current_state
    current_state = msg

def position_cb(pose):
    global current_pose
    current_pose = pose

def image_callback(msg):
    global CV_IMAGE
    bridge = CvBridge()
    try:
        # Convert to OpenCV image format
        cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv_img = cv2.resize(cv_img, (IMAGE_SHAPE[1], IMAGE_SHAPE[0])) 
        
    except CvBridgeError as e:
        print(e)

    CV_IMAGE = cv_img
    # rospy.loginfo("got image")
    save_dir = "./image_feed"
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    # Define the filename
    filename = os.path.join(save_dir, "image_{}.png".format(time.time()))
    
    # Save the image
    cv2.imwrite(filename, cv_img)
    # rospy.loginfo("Image saved as: {}".format(filename))


def model_vel():
    global CV_IMAGE
    global MODEL_MODE
    global VEL
    global INDEX
    global image_sequences
    global vel_msg
    global COUNT

    rospy.loginfo("Entering function to get pred")
    img = CV_IMAGE
    save_dir = "./vel_input"
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    # Define the filename
    filename = os.path.join(save_dir, "image_{}.png".format(time.time()))
    
    # Save the image
    cv2.imwrite(filename, img)

    print(INDEX)
    if INDEX == 0:
        image_sequences = np.stack([img] * 64)
        
        # print(image_sequences.shape)
        
    else:
        
        image_sequences = np.vstack((image_sequences[0][1:], [img]))
        # print(image_sequences.shape)

    image_sequences = np.expand_dims(image_sequences, axis=0)
    INDEX += 1

    with tf.device('/cpu:0'):
        preds = model.predict(image_sequences)
        print(preds[0][63])
        
        vx, vy, vz, omega_z = preds[0][63]

        # Set the velocities in the message
        vel_msg.linear.x = 0.1 * vx
        vel_msg.linear.y = 0.1 * vy
        vel_msg.linear.z = 0.1 * vz
        vel_msg.angular.z = 0.1 * omega_z

    rospy.loginfo("got velocity")
    VEL = True
    COUNT += 1

    
if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    local_position_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, position_cb)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    
    velocity_publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)

    img_subscriber = rospy.Subscriber("/cgo3_camera/image_raw", Image, image_callback)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 1
    pose.pose.position.y = 1
    pose.pose.position.z = 6

    # vel_msg = Twist()
    # vel_msg.linear.x = 0.5 
    # vel_msg.linear.y = 0
    # vel_msg.linear.z = 0
    # vel_msg.angular.z = 0

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()
        
        
        

        if MODEL_MODE == False or VEL == False:
            # rospy.loginfo("Publishing pose")
            local_pos_pub.publish(pose)
            # print("pose", current_pose.pose.position.z)
            if current_pose.pose.position.z > 5.5:
                rospy.loginfo("Target pos reached")
                MODEL_MODE = True
        else:
            
            velocity_publisher.publish(vel_msg)
            rospy.loginfo("Vel published")
                
        
        if MODEL_MODE:
            rospy.loginfo("Calculating vel")
            model_vel()  
        

        rate.sleep()
