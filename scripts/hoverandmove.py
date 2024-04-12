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

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

IMAGE_SHAPE = (144, 256, 3)
IMAGE_SHAPE_CV = (IMAGE_SHAPE[1], IMAGE_SHAPE[0])
TAKEOFF_HEIGHT = 6 

tf.config.set_visible_devices([], 'GPU')

# print(os.getcwd())
with tf.device('/cpu:0'):
    model = tf.keras.models.load_model('model_ssfalse_b64_lr0.0001wscheduler_seqlen64_new_dataset.h5')

predictions = []

image_sequences = []
index = 0
velocity_publisher = None

class DroneController:
    def __init__(self):
        # Set up ROS node
        rospy.init_node('drone_controller', anonymous=True)
        
        # Set up publishers and subscribers
        self.pose_publisher = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.velocity_publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
        self.state_subscriber = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.image_subscriber = rospy.Subscriber("/cgo3_camera/image_raw", Image, self.image_callback)
        self.local_position_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_position_cb)
        self.current_altitude = 0.0
        self.bridge = CvBridge()
        
        # Service clients
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        # Drone state
        self.rate = rospy.Rate(20)  # Hz
        self.state = State()
        self.reached_takeoff_height = False

    def local_position_cb(self, data):
        self.current_altitude = data.pose.position.z

    def state_cb(self, state):
        self.state = state

    def takeoff_and_hover(self, target_position):
        pose = PoseStamped()
        pose.header.frame_id = "base_footprint"
        pose.pose.position.x = target_position['x']
        pose.pose.position.y = target_position['y']
        pose.pose.position.z = target_position['z']

        # Set to offboard mode and arm the drone
        while not self.state.connected:
            self.rate.sleep()

        for _ in range(100):
            self.pose_publisher.publish(pose)
            self.rate.sleep()

        rospy.wait_for_service('/mavros/set_mode')
        try:
            if self.set_mode_client(custom_mode="OFFBOARD").mode_sent:
                rospy.loginfo("Offboard mode set")
            if self.arming_client(True).success:
                rospy.loginfo("Drone armed")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

        # Take off and reach the desired altitude
        while not rospy.is_shutdown():
            pose.header.stamp = rospy.Time.now()
            self.pose_publisher.publish(pose)
            self.rate.sleep()
            if abs(self.current_altitude - target_position['z']) < 0.1:  # Check if the drone is close to the target height
                self.reached_takeoff_height = True
                rospy.loginfo("Reached target altitude.")
                break

    def image_callback(self, msg):
        global image_sequences, index
        if not self.reached_takeoff_height:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image = cv2.resize(cv_image, (IMAGE_SHAPE[1], IMAGE_SHAPE[0])) 
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        
        if index == 0:
            image_sequences = np.stack([cv_image] * 64)
            print("index", index, image_sequences.shape)
        else:
            image_sequences = np.vstack((image_sequences[0][1:], [cv_image]))
            print("index", index, image_sequences.shape)

        image_sequences = np.expand_dims(image_sequences, axis=0)
        index += 1

        with tf.device('/cpu:0'):
            preds = model.predict(image_sequences)
            vx, vy, vz, omega_z = preds[0][63]
            self.publish_velocity(vx, vy, vz, omega_z)

    def publish_velocity(self, vx, vy, vz, omega_z):
        vel_msg = Twist()
        vel_msg.linear.x = 4 * vx
        vel_msg.linear.y = 4 * vy
        vel_msg.linear.z = 4 * vz
        vel_msg.angular.z = 4 * omega_z
        self.velocity_publisher.publish(vel_msg)
        print("velocity published", vx, vy, vz, omega_z)

if __name__ == '__main__':
    drone_controller = DroneController()
    target_position = {'x': 1, 'y': 1, 'z': TAKEOFF_HEIGHT}
    
    # Perform takeoff
    drone_controller.takeoff_and_hover(target_position)
    
    # If takeoff is successful, start image processing and velocity publishing
    if drone_controller.reached_takeoff_height:
        rospy.loginfo("Drone reached takeoff height, starting image processing.")
        rospy.spin()
    else:
        rospy.loginfo("Failed to reach takeoff height.")