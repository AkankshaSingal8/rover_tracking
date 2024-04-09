#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import time

def image_callback(msg):
    print("entering_function")
    bridge = CvBridge()
    try:
        # Convert to OpenCV image format
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        print("converted to cv img")
    except CvBridgeError as e:
        print(e)

    save_dir = "./image_feed"
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    # Define the filename
    filename = os.path.join(save_dir, "image_{}.png".format(time.time()))
    
    # Save the image
    cv2.imwrite(filename, cv_image)
    rospy.loginfo("Image saved as: {}".format(filename))

def main():
    rospy.init_node('drone_raw_image_viewer', anonymous=True)
    # Subscribe to the raw image topic
    rospy.Subscriber("/cgo3_camera/image_raw", Image, image_callback)
    print("Subscribed")
    
    rospy.spin()
    

if __name__ == '__main__':
    main()
