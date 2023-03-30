#!/usr/bin/env python3

import rospy
# import the Odometry message from the nav_msgs package:
from nav_msgs.msg import Odometry
# additionally, import the "euler_from_quaternion" function from the tf library
# for converting the raw orientation values from the odometry message into euler angles:
from tf.transformations import euler_from_quaternion
from math import pi

class OdomSubscriber():

    def callback(self, topic_data: Odometry):
        # extracting the pose part of the Odometry message
        pose = topic_data.pose.pose
        # extracting information about the "position" and "orientation" of the robot
        position = pose.position
        orientation = pose.orientation 
        # extracting "position" data provided in meters
        pos_x = position.x
        pos_y = position.y
        pos_z = position.z
        # "orientation" data is in quaternions, converting this using the 
        # "euler_from_quaternion" function 
        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, 
                     orientation.y, orientation.z, orientation.w], 
                     'sxyz')

        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        

        # Here we print out the live position values of the robot
        
    def __init__(self):
        node_name = "odom_subscriber" # a name for our node
        rospy.init_node(node_name, anonymous=True)
        # When setting up the subscriber, the "odom" topic needs to be specified
        # and the message type (Odometry) needs to be provided
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)
        # an optional status message:
        rospy.loginfo(f"The '{node_name}' node is active...")

        self.print_initial_pos = True
        self.counter = 0      

    def main_loop(self):
        # set the node to remain active until closed manually:
        rospy.spin()

if __name__ == '__main__':
    subscriber_instance = OdomSubscriber()
    subscriber_instance.main_loop()