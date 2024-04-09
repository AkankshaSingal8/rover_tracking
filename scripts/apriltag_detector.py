#!/usr/bin/env python
import rospy
from apriltag_ros.msg import AprilTagDetectionArray

def tag_callback(data):
    # Process detection data
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.detections)

def listener():
    rospy.init_node('tag_listener', anonymous=True)
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, tag_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
