#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class DroneTakeoff:
    def __init__(self, target_altitude):
        rospy.init_node('drone_takeoff_node', anonymous=True)

        self.target_altitude = target_altitude
        self.pose_publisher = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.state_subscriber = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        self.rate = rospy.Rate(20)  # Hz
        self.state = State()

    def state_cb(self, state):
        self.state = state

    def takeoff_and_hover(self):
        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = self.target_altitude

        # Wait for FCU connection
        while not self.state.connected:
            self.rate.sleep()

        # Publish the pose a few times before changing the mode to ensure it's received.
        for _ in range(100):
            self.pose_publisher.publish(pose)
            self.rate.sleep()

        # Set to offboard mode
        if self.set_mode_client(custom_mode="OFFBOARD").mode_sent:
            rospy.loginfo("Offboard mode set")

        # Arm the drone
        if self.arming_client(True).success:
            rospy.loginfo("Drone armed")

        # Keep publishing the pose to maintain altitude and position.
        rospy.loginfo("Maintaining altitude...")
        while not rospy.is_shutdown():
            self.pose_publisher.publish(pose)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        takeoff = DroneTakeoff(target_altitude=5)  # Target altitude in meters
        takeoff.takeoff_and_hover()
    except rospy.ROSInterruptException:
        pass
