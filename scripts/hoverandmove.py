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
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

IMAGE_SHAPE = (144, 256, 3)
IMAGE_SHAPE_CV = (IMAGE_SHAPE[1], IMAGE_SHAPE[0])
TAKEOFF_HEIGHT = 6 

tf.config.set_visible_devices([], 'GPU')

# print(os.getcwd())
with tf.device('/cpu:0'):
    model = tf.keras.models.load_model('model_ssfalse_b64_lr0.0001wscheduler_seqlen64_new_dataset.h5')


velocity_publisher = None


class DroneTakeoff:
    def __init__(self, target_position):
        rospy.init_node('drone_hover_node', anonymous=True)

        # Unpack the target position into altitude and coordinates
        self.target_altitude = target_position['z']
        self.target_x = target_position['x']
        self.target_y = target_position['y']
        
        # Initialize the ROS publisher and subscribers
        self.pose_publisher = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.state_subscriber = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.local_position_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.position_cb)
        self.img_subscriber = rospy.Subscriber("/cgo3_camera/image_raw", Image, self.image_callback)
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.velocity_publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
        
        # rospy.wait_for_service("/mavros/cmd/arming")
        # self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

        # rospy.wait_for_service("/mavros/set_mode")
        # self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        self.rate = rospy.Rate(20)  # Hz
        self.state = State()
        self.current_pose = PoseStamped()

        self.MODEL_MODE = False
        self.VEL = False


        self.predictions = []
        self.cv_image = None
        self.image_sequences = []
        self.index = 0

        self.vel_msg = Twist()

    def state_cb(self, state):
        self.state = state
    
    def position_cb(self, pose):
        self.current_pose = pose
    
    def image_callback(self, msg):
    
        bridge = CvBridge()
        try:
            # Convert to OpenCV image format
            cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_img = cv2.resize(cv_img, (IMAGE_SHAPE[1], IMAGE_SHAPE[0])) 
            
        except CvBridgeError as e:
            print(e)
        self.cv_image = cv_img
        # rospy.loginfo("got image")
        
    
    def model_vel(self):
        rospy.loginfo("Entering function to get pred")
        img = self.cv_image
        if self.index == 0:
            self.image_sequences = np.stack([img] * 64)
            
            print(self.image_sequences.shape)
            
        else:
            
            self.image_sequences = np.vstack((self.image_sequences[0][1:], [img]))
            print(self.image_sequences.shape)

        self.image_sequences = np.expand_dims(self.image_sequences, axis=0)
        self.index += 1

        with tf.device('/cpu:0'):
            preds = model.predict(self.image_sequences)
            print(preds[0][63])
            
            vx, vy, vz, omega_z = preds[0][63]

            # Set the velocities in the message
            self.vel_msg.linear.x = 5 * vx
            self.vel_msg.linear.y = 5 * vy
            self.vel_msg.linear.z = 5 * vz
            self.vel_msg.angular.z = 5 * omega_z

        rospy.loginfo("got velocity")
        self.VEL = True

        # save_dir = "./image_feed"
        # if not os.path.exists(save_dir):
        #     os.makedirs(save_dir)

        # # Define the filename
        # filename = os.path.join(save_dir, "image_{}.png".format(time.time()))
        
        # # Save the image
        # cv2.imwrite(filename, cv_image)
        # rospy.loginfo("Image saved as: {}".format(filename))
    
    def is_within_tolerance(self, current_position, tolerance):
        return (current_position.z > 5.5)

    def takeoff_and_hover(self):
        # Create a PoseStamped message with the desired pose.
        pose = PoseStamped()
        pose.header.frame_id = "base_footprint"  # Set the correct frame id
        pose.pose.position.x = self.target_x
        pose.pose.position.y = self.target_y
        pose.pose.position.z = self.target_altitude

        # Wait for FCU connection.
        # while(not rospy.is_shutdown() and not self.state.connected):
        #     self.rate.sleep()

        while not self.state.connected:
            self.rate.sleep()

        # Publish the pose a few times before changing the mode to ensure it's received.
        for _ in range(100):
            pose.header.stamp = rospy.Time.now()
            self.pose_publisher.publish(pose)
            self.rate.sleep()

        # Set to offboard mode
        if self.set_mode_client(custom_mode="OFFBOARD").mode_sent:
            rospy.loginfo("Offboard mode set")
        else:
            rospy.loginfo("Failed to set Offboard mode")

        # Arm the drone
        if self.arming_client(True).success:
            rospy.loginfo("Drone armed")
        else:
            rospy.loginfo("Failed to arm drone")

        # offb_set_mode = SetModeRequest()
        # offb_set_mode.custom_mode = 'OFFBOARD'

        # arm_cmd = CommandBoolRequest()
        # arm_cmd.value = True

        # last_req = rospy.Time.now()

        # Keep publishing the pose to maintain altitude and position.
        rospy.loginfo("Maintaining position...")
        while not rospy.is_shutdown():
            # if(self.state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            #     if(self.set_mode_client.call(offb_set_mode).mode_sent == True):
            #         rospy.loginfo("OFFBOARD enabled")

            #     last_req = rospy.Time.now()
            # else:
            #     if(not self.state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            #         if(self.arming_client.call(arm_cmd).success == True):
            #             rospy.loginfo("Vehicle armed")

            #         last_req = rospy.Time.now()

            if self.MODEL_MODE == False or self.VEL == False:
                pose.header.stamp = rospy.Time.now()
                self.pose_publisher.publish(pose)
                
                if self.is_within_tolerance(self.current_pose.pose.position, tolerance=0.1):
                    rospy.loginfo("Target position reached")
                    self.MODEL_MODE = True
                else:
                    self.MODEL_MODE = False
        
            if self.MODEL_MODE:
                self.model_vel()
                rospy.loginfo("calculated vel")
            
            if self.MODEL_MODE == True and self.VEL == True:
                self.velocity_publisher.publish(self.vel_msg)
                rospy.loginfo("Vel published")
            

                    



if __name__ == '__main__':
    try:
        # The desired target position is extracted from the provided image
        target_position = {
            'x': 0,  
            'y': 0,  
            'z': 6  
        }
        takeoff = DroneTakeoff(target_position)
        takeoff.takeoff_and_hover()
    except rospy.ROSInterruptException:
        pass
