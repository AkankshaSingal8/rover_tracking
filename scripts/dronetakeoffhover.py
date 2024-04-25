#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class DroneTakeoff:
    def __init__(self, target_position):
        rospy.init_node('drone_takeoff_node', anonymous=True)

        # Unpack the target position into altitude and coordinates
        self.target_altitude = target_position['z']
        self.target_x = target_position['x']
        self.target_y = target_position['y']
        
        # Initialize the ROS publisher and subscribers
        self.pose_publisher = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.state_subscriber = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.local_position_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.position_cb)
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        self.rate = rospy.Rate(20)  # Hz
        self.state = State()
        self.current_pose = PoseStamped()

    def state_cb(self, state):
        self.state = state

    def takeoff_and_hover(self):
        # Create a PoseStamped message with the desired pose.
        pose = PoseStamped()
        pose.header.frame_id = "base_footprint"  # Set the correct frame id
        pose.pose.position.x = self.target_x
        pose.pose.position.y = self.target_y
        pose.pose.position.z = self.target_altitude

        # Wait for FCU connection.
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

        # Keep publishing the pose to maintain altitude and position.
        rospy.loginfo("Maintaining position...")
        while not rospy.is_shutdown():
            pose.header.stamp = rospy.Time.now()
            self.pose_publisher.publish(pose)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        # The desired target position is extracted from the provided image
        target_position = {
            'x': 1,  
            'y': 1,  
            'z': 6  
        }
        takeoff = DroneTakeoff(target_position)
        takeoff.takeoff_and_hover()
    except rospy.ROSInterruptException:
        pass
