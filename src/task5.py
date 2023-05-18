#!/usr/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
import argparse
from math import pi
from pathlib import Path
import string
import numpy as np
import rospy
import actionlib

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan

class Task5:
    def __init__(self):
        self.node_name ="task5"
        self.map_path = "/home/student/catkin_ws/src/com2009_team46/maps/task5_map"
        cli = argparse.ArgumentParser(description=f"Command-line interface for the '{self.node_name}' node.")
        cli.add_argument("colour", metavar="COL", default="Black",help="The name of a colour(Blue/Red/Yellow?Green)")

        args, self.colour = cli.parse_known_args(rospy.myargv()[1:])
        rospy.init_node('task5',anonymous=True)
        
        self.camera_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        self.cv_bridge = CvBridge()
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odometry_callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.vel = Twist()
        
        self.lower_colour = (100, 200, 100)
        self.upper_colour = (100, 255, 255)
        
        if self.colour[0] == "red":
            self.lower_colour = np.array([0, 100, 100])
            self.upper_colour = np.array([10, 255, 255])
        elif self.colour[0] == "green":
            self.lower_colour = np.array([50, 100, 100])
            self.upper_colour = np.array([70, 255, 255])
        elif self.colour[0] == "blue":
            self.lower_colour = np.array([110, 100, 100])
            self.upper_colour = np.array([130, 255, 255])
        elif self.colour[0] == "yellow":
            self.lower_colour = np.array([20, 100, 100])
            self.upper_colour = np.array([40, 255, 255])
        else:
            rospy.loginfo("Invalid colour")
        
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
        self.right_arc_dis = 0
        self.left_arc_dis = 0
        self.current_yaw = 0
        self.turn_direction = "None"
        self.startup = True
        self.pic = False
        self.m00 = 0
        self.m00_min = 10000
        self.base_image_path = Path.home().joinpath("/home/student/catkin_ws/src/com2009_team46/pictures/")
        self.base_image_path.mkdir(parents=True, exist_ok=True) 
        
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"the {self.node_name} node has been initialised...")
        rospy.loginfo(f"TASK 5 BEACON: The target is {self.colour[0]}.")
        

    def shutdownhook(self):
        self.pub.publish(Twist())
        
        cv2.destroyAllWindows()
        self.ctrl_c = True
        
    def camera_callback(self, msg):
        try:
            cv_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        height, width, _ = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

            
        mask = cv2.inRange(hsv_img, self.lower_colour, self.upper_colour)
        res = cv2.bitwise_and(crop_img, crop_img, mask = mask)

        m = cv2.moments(mask)
        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
        
        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)
            
    
    def scan_callback(self, scan_data):
        left = scan_data.ranges[0:20]
        right = scan_data.ranges[-21:]
        front_arc = np.array(left[::-1] + right[::-1])
        self.min_dis_front = front_arc.min()


        left_arc = scan_data.ranges[30:50]
        right_arc = scan_data.ranges[-50:-30]
        self.left_arc_dis = np.array(left_arc).min()
        self.right_arc_dis = np.array(right_arc).min()

        self.absolute_left = scan_data.ranges[90]
        self.absolute_right = scan_data.ranges[-90]
        self.absolute_front = scan_data.ranges[0]
        
    def odometry_callback(self, topic_data):
        
        pose = topic_data.pose.pose
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
        self.theta_z = abs(yaw)

        # If this is the first time that the callback_function has run
        # (e.g. the first time a message has been received), then
        # obtain a "reference position" (used to determine how far
        # the robot has moved during its current operation)
        if self.startup:
            # don't initialise again:
            self.startup = False
            # set the reference position:
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z
    
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
        if self.current_yaw >= pi/2:
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
        if self.current_yaw >= pi/2:
            self.vel = Twist()
            self.current_yaw = 0
        rospy.sleep(2.5)
        self.vel = Twist()
        rospy.sleep(0.5)
        
    def fix_position(self):
        if self.left_arc_dis < 0.3:
            self.vel.linear.x = 0
            self.vel.angular.z = -0.3
            self.pub.publish(self.vel)
        elif self.right_arc_dis < 0.3:
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

    def save_image(self, cv_img, base_image_path):
        # save the image:
        image_path = base_image_path.joinpath("/beacon.jpg")
        cv2.imshow('beacon', cv_img)
        cv2.imwrite(str(image_path), cv_img)
        # increment the image counter:
        self.image_counter += 1
        # print a message to the console:
        print("Saved image " + image_path)
        
            
    def main_loop(self):
        self.start()
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            
            if self.m00 > self.m00_min:
                if (self.pic == False):
                    self.pic = True
                    self.save_image(self.cv_img, self.base_image_path)
            
            if self.absolute_front<0.45:
                if self.absolute_left < 0.5 :
                    self.vel= Twist()
                    rospy.sleep(0.5)
                    self.turn_right()
                elif self.absolute_right < 0.5 :
                    self.vel= Twist()
                    rospy.sleep(0.5) 
                    self.turn_left()
                else:
                        self.vel= Twist()
                        rospy.sleep(0.5) 
                        self.turn_left()
            else :
                self.fix_position()
            self.pub.publish(self.vel)
            r.sleep()
        

if __name__ == "__main__":
    node = Task5()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass