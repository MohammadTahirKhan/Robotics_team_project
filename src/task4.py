#!/usr/bin/env python3

import argparse
from tkinter.messagebox import YES
import rospy
import numpy as np
from math import pi
from pathlib import Path 
import roslaunch

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan

class Task4:
    def __init__(self):
        self.node_name ="task4"
        rospy.init_node('task4',anonymous=True)

        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        self.cvbridge = CvBridge()
        
        self.subScan = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber('/odom', Odometry, self.odometry_callback)

        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.vel = Twist()

        self.lower_colour = (0,0,0)
        self.upper_colour = (255,255,255)
        self.blue_lower = (115, 50, 100)
        self.blue_upper = (130, 255, 255)
        self.red_lower = (0, 50, 100)
        self.red_upper = (10, 255, 255)
        self.yellow_lower = (25,50,100)
        self.yellow_upper = (40, 255, 255)
        self.green_lower = (50, 70, 100)
        self.green_upper = (70,255,255)
        self.tur_lower = (85, 200, 100)
        self.tur_upper = (100, 255, 255) 
        self.purple_lower = (145, 200, 100)
        self.purple_upper = (165, 255, 255)
        self.start_zone_colours = ["Blue", "Red", "Yellow", "Green", "Turquoise", "Purple"]
        self.start_zone = ""
        rospy.on_shutdown(self.shutdownhook)

         # define the robot pose variables and initialise them to zero:
        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        # variables for a "reference position":
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0
        self.colour_found = False
        self.colour = "no"
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
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
        self.base_image_path = Path.home().joinpath("/home/student/catkin_ws/src/com2009_team46/snaps/")
        self.base_image_path.mkdir(parents=True, exist_ok=True) 
        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()

        rospy.loginfo(f"the {self.node_name} node has been initialised...")
        

    def shutdownhook(self):
        self.pub.publish(Twist())
        cv2.destroyAllWindows()
        self.ctrl_c = True
        
    def detect_colour(self, cv_img, lower, upper):
        height, width, _ = cv_img.shape
        crop_width = width - 800
        crop_height = 100
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y+20:crop_y+crop_height+20, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv_img, lower, upper)
        res = cv2.bitwise_and(crop_img, crop_img, mask = mask)
        m00_min = 10000
        m = cv2.moments(mask)
        m00 = m['m00']
        cy = m['m10'] / (m['m00'] + 1e-5)

        if m00 > m00_min:
            return True
        else:
            return False

    def camera_callback(self, img_data):
        try:
            self.cv_img = self.cvbridge.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        height, width, _ = self.cv_img.shape
        crop_width = width - 800
        crop_height = 100
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = self.cv_img[crop_y+20:crop_y+crop_height+20, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv_img,self.lower_colour, self.upper_colour)
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
    
    # def start(self):
    #     rospy.sleep(1)
    #     self.vel.linear.x = 0.25
    #     self.vel.angular.z = 0.0
    #     self.pub.publish(self.vel)
    #     rospy.sleep(3)

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

    def main_loop(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.colour == "no":
                rospy.sleep(1)
                self.vel.linear.x = 0.0
                self.vel.angular.z = 2.0
                self.pub.publish(self.vel)
                rospy.sleep(2)
                while(self.colour == "no"):
                    if self.detect_colour(self.cv_img, self.blue_lower, self.blue_upper) == True:
                        self.colour = self.start_zone_colours[0]
                        self.lower_colour = self.blue_lower
                        self.upper_colour = self.blue_upper
                    if self.detect_colour(self.cv_img, self.red_lower, self.red_upper) == True:
                        self.colour = self.start_zone_colours[1]
                        self.lower_colour = self.red_lower
                        self.upper_colour = self.red_upper
                    if self.detect_colour(self.cv_img, self.yellow_lower, self.yellow_upper) == True:
                        self.colour = self.start_zone_colours[2]
                        self.lower_colour = self.yellow_lower
                        self.upper_colour = self.yellow_upper
                    if self.detect_colour(self.cv_img, self.green_lower, self.green_upper) == True:
                        self.colour = self.start_zone_colours[3]
                        self.lower_colour = self.green_lower
                        self.upper_colour = self.green_upper
                    if self.detect_colour(self.cv_img, self.tur_lower, self.tur_upper) == True:
                        self.colour = self.start_zone_colours[4]
                        self.lower_colour = self.tur_lower
                        self.upper_colour = self.tur_upper
                    if self.detect_colour(self.cv_img, self.purple_lower, self.purple_upper) == True:
                        self.colour = self.start_zone_colours[5]
                        self.lower_colour = self.purple_lower
                        self.upper_colour = self.purple_upper
                    # print(self.colour)
                print(f"SEARCH INITIATED: The target beacon colour is {self.colour}.")
                self.vel.linear.x = 0
                self.vel.angular.z = -2.0
                self.pub.publish(self.vel)
                rospy.sleep(2.4)
                self.vel = Twist()
                self.pub.publish(self.vel)
                rospy.sleep(2)
                    
            if self.colour != "no" and self.m00 > self.m00_min:
                self.vel = Twist()
                self.pub.publish(self.vel)
                if self.cy <=550:
                    self.vel.angular.z = 0.3
                elif self.cy >=570:
                    self.vel.angular.z = -0.3
                else:
                    self.vel.angular.z = 0.0
                self.vel.linear.x = 0.2
                self.pub.publish(self.vel)
                rospy.sleep(0.1)
                print("TARGET DETECTED: Beaconing initiated.")
                
                if self.absolute_front <= 0.59:
                    print("BEACONING COMPLETE: The robot has now stopped.")
                    self.vel = Twist()
                    self.pub.publish(self.vel)
                    self.shutdownhook()
                    break
                    
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
    node = Task4()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass