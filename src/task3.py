#!/usr/bin/env python3

from turtle import left
import rospy

import numpy as np

# import the Twist message for publishing velocity commands:
from geometry_msgs.msg import Twist

# import the Odometry message for subscribing to the odom topic:
from nav_msgs.msg import Odometry

# import the function to convert orientation from quaternions to angles:
from tf.transformations import euler_from_quaternion

from sensor_msgs.msg import LaserScan

# import math:
from math import pi

class Task3:
    
    def odometry_callback(self, odom_data):
        # obtain relevant topic data: pose (position and orientation):
        pose = odom_data.pose.pose
        position = pose.position
        orientation = pose.orientation

        # obtain the robot's position co-ords:
        pos_x = position.x
        pos_y = position.y

        # convert orientation co-ords to roll, pitch & yaw (theta_x, theta_y, theta_z)
        # using the "euler_from_quaternion" function:
        (roll, pitch, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w], "sxyz"
        )

        # We're only interested in x, y and theta_z
        # so assign these to class variables:
        self.x = pos_x 
        self.y = pos_y 
        self.theta_z = yaw

        if self.startup:
            # don't initialise again:
            self.startup = False
            # set the reference position:
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z

        

    def scan_callback(self, scan_data):
        left_arc = scan_data.ranges[0:21]
        right_arc = scan_data.ranges[-20:]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        self.min_dis_front = front_arc.min()
        self.max_dis_front = front_arc.max()
        self.min_dis_left = np.array(left_arc).min()
        self.min_dis_right = np.array(right_arc).min()

        # if self.min_dis_front <= 0.5:
        #     if self.min_dis_right > 0.3:
        #         self.turn_direction = "right"
        #     elif self.max_dis_front < 0.6:
        #         self.turn_direction = "uTurn"
        #     else:
        #         self.turn_direction = "left"
        # else:
        #     # if self.min_dis_right < 0.25:
        #     #     self.turn_direction = "right"
        #     # else:
        #     self.turn_direction = "forward"

        # if self.min_dis_front > 0.5:
        #     self.turn_direction = "forward"
        if self.min_dis_left <= 0.5:
            if self.min_dis_front <=0.5:
                self.turn_direction = "right"
            else:
                self.turn_direction = "forward"
        else:
            self.turn_direction = "left"
            

    # def scan_callback(self, scan_data):
    #     left_arc = scan_data.ranges[220:400]
    #     right_arc = scan_data.ranges[0:140]
    #     front_arc = np.array(left_arc[::-1] + right_arc[::-1])
    #     self.min_dis_front = front_arc.min()
    #     self.min_dis_left = np.array(left_arc).min()
    #     self.min_dis_right = np.array(right_arc).min()
    #     self.max_dis_front = np.array(front_arc[0:50]).max()
        
    #     if self.min_dis_left > 0.5:
    #         # Too far away from the wall on the left, need to turn left
    #         self.turn_direction = "left"
    #     elif self.min_dis_left < 0.3:
    #         # Too close to the wall on the left, need to turn right
    #         self.turn_direction = 'right'
    #     elif self.min_dis_front < 0.25:
    #         # Obstacle in front, need to turn right
    #         self.turn_direction = 'right

    
    def __init__(self):
        node_name = "task3"
        self.startup = True
        self.turn_direction = "None"

        self.sub_scan = rospy.Subscriber("scan", LaserScan, self.scan_callback)
        rospy.Subscriber("odom", Odometry, self.odometry_callback)
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        rospy.init_node(node_name, anonymous=True)
        # self.rate = rospy.Rate(10)  # hz
        
        # define the robot pose variables and initialise them to zero:
        # variables for the robot's "current position":
        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0

        self.min_dis_front = 0
        self.min_dis_right = 0
        self.min_dis_left = 0
        self.max_dis_front = 0
        self.turn_direction = "forward"
        self.turn = "no"
        
        # define a Twist message instance, to set robot velocities
        self.vel = Twist()
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"the {node_name} node has been initialised...")

    def shutdownhook(self):
        self.pub.publish(Twist())
        self.ctrl_c = True
            
    def main_loop(self):
        current_yaw = 0
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            current_yaw = current_yaw + abs(self.theta_z - self.theta_z0 )
            self.theta_z0 = self.theta_z
            if self.turn_direction == "left":
                if current_yaw <= pi/2 :
                    self.vel.angular.z = 0.5
                else:
                    self.vel = Twist()
                    # current_yaw = 0
                    # self.vel.linear.x = 0.2
                    self.turn_direction = "forward"
            
            if self.turn_direction == "forward":
                self.vel.linear.x = 0.3
                self.vel.angular.z = 0
            elif self.turn_direction == "right":
                self.vel.linear.x = 0
                self.vel.angular.z = -0.5
            else:
                self.vel.linear.x = 0
                self.vel.angular.z = 0
            self.pub.publish(self.vel)
            r.sleep()
    
        
if __name__ == "__main__":
    node = Task3()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass


