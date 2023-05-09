#!/usr/bin/env python3

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
import math 

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
        left = scan_data.ranges[0:20]
        right = scan_data.ranges[-21:]
        front_arc = np.array(left[::-1] + right[::-1])
        self.min_dis_front = front_arc.min()


        left_arc = scan_data.ranges[30:50]
        right_arc = scan_data.ranges[-50:-30]
        self.left_arc = np.array(left_arc).min()
        self.right_arc = np.array(right_arc).min()

        self.absolute_left = scan_data.ranges[90]
        self.absolute_right = scan_data.ranges[-90]
        self.absolute_front = scan_data.ranges[0]

        # print("l",self.left_arc)
        # print("f",self.absolute_front)
        # print("r",self.right_arc)

            
    
    def __init__(self):
        node_name = "task3"
        self.startup = True

        self.sub_scan = rospy.Subscriber("scan", LaserScan, self.scan_callback)
        rospy.Subscriber("odom", Odometry, self.odometry_callback)
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        rospy.init_node(node_name, anonymous=True)
        # self.rate = rospy.Rate(10)  # hz
        
        # define the robot pose variables and initialise them to zero:
        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        # variables for a "reference position":
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0

        self.absolute_right = 0
        self.absolute_left = 0
        self.absolute_front = 0
        self.right_arc = 0
        self.left_arc = 0
        self.wall_count = 0
        self.current_yaw = 0

        
        # define a Twist message instance, to set robot velocities
        self.vel = Twist()
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"the {node_name} node has been initialised...")

    def shutdownhook(self):
        self.pub.publish(Twist())
        self.ctrl_c = True

    def start(self):
        rospy.sleep(1)
        self.vel.linear.x = 0.25
        self.vel.angular.z = 0.0
        self.pub.publish(self.vel)
        rospy.sleep(3)

    def turn_right(self):
        self.current_yaw = self.current_yaw + abs(self.theta_z - self.theta_z0 )
        self.theta_z0 = self.theta_z
        self.vel.linear.x = 0
        self.vel.angular.z = -0.6
        self.pub.publish(self.vel)
        if self.absolute_left > 0.35:
            self.vel.linear.x = 0
            self.vel.angular.z = -0.3
            self.pub.publish(self.vel)
        if self.current_yaw >= math.pi/2:
            self.vel = Twist()
            self.current_yaw = 0
        rospy.sleep(2.5)
        self.vel = Twist()
        rospy.sleep(0.5)

    def turn_left(self):
        self.current_yaw = self.current_yaw + abs(self.theta_z - self.theta_z0 )
        self.theta_z0 = self.theta_z
        self.vel.linear.x = 0
        self.vel.angular.z = 0.6
        self.pub.publish(self.vel)
        if self.absolute_right > 0.35:
            self.vel.linear.x = 0
            self.vel.angular.z = 0.3
            self.pub.publish(self.vel)
        if self.current_yaw >= math.pi/2:
            self.vel = Twist()
            self.current_yaw = 0
        rospy.sleep(2.5)
        self.vel = Twist()
        rospy.sleep(0.5)
        
    def fix_position(self):
        if self.left_arc < 0.3:
            self.vel.linear.x = 0
            self.vel.angular.z = -0.3
            self.pub.publish(self.vel)
        elif self.right_arc < 0.3:
            self.vel.linear.x = 0
            self.vel.angular.z = 0.3
            self.pub.publish(self.vel)
        else:
            if self.min_dis_front<0.7:
                self.vel.linear.x = 0.16
                self.vel.angular.z = 0.0
                self.pub.publish(self.vel)
            else :
                self.vel.linear.x = 0.26
                self.vel.angular.z = 0.0
                self.pub.publish(self.vel)

            
    def main_loop(self):
        self.start()
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.absolute_front<0.45:
                if self.absolute_left < 0.5 :
                    self.vel= Twist()
                    rospy.sleep(0.5)
                    self.turn_right()
                elif self.absolute_right < 0.5 :
                    self.vel= Twist()
                    rospy.sleep(0.5) 
                    self.turn_left()
                elif self.absolute_left >= 0.65 and self.absolute_right >= 0.65:
                # else:
                    if self.wall_count == 0:
                        self.vel= Twist()
                        rospy.sleep(0.5) 
                        self.turn_left()
                        self.wall_count+=1
                    elif self.wall_count == 1:
                        self.vel= Twist()
                        rospy.sleep(0.5) 
                        self.turn_right()
                        self.wall_count+=1
                    else:
                        self.vel= Twist()
                        rospy.sleep(0.5) 
                        self.turn_left()
                else:
                    self.fix_position()
            else :
                self.fix_position()
            self.pub.publish(self.vel)
            r.sleep()
    
        
if __name__ == "__main__":
    node = Task3()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass