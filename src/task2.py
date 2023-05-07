#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import random
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

LINEAR_SPEED = 0.6
ANGULAR_SPEED = 1.0
RAND=random.random()

RANGE_THRESHOLD = 0.85
x,y,z=0,0,0
plot=5
twist_msg = Twist()
def odom_callback(msg):
        global x,y,z,plot
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.orientation.z
        if y>1:
            if x>1:
                plot=9
            elif  -1<x<1: 
                 plot=6
            elif  x<-1: 
                 plot=3
        elif -1<y<1:
            if x>1:
                plot=8
            elif -1<x<1: 
                 plot=5
            elif  x<-1: 
                 plot=2
        elif  y<-1:
            if x>1:
                plot=7
            elif  -1<x<1: 
                 plot=4
            elif x<-1: 
                 plot=1
        #rospy.loginfo("My variable is %d", plot)
def scan_callback(data):
    global time
    ranges = data.ranges
    tui=0
    qian=0
    obstacle_flag = False
    global RAND

    for i in [0,15]:
      if ranges[i] < RANGE_THRESHOLD:
        obstacle_flag = True
        if ranges[i] < RANGE_THRESHOLD/2:
            tui=1
  
    for l in [-15,0]:
       if ranges[l] < RANGE_THRESHOLD:
        obstacle_flag = True
        if ranges[i] < RANGE_THRESHOLD/2:
            tui=1
    for m in [180,165]:
      if ranges[m] < 0.3:
        qian = 1
  
    for n in [-165,-180]:
      if ranges[m] < 0.3:
        qian = 1

    if obstacle_flag:
        twist_msg.linear.x = LINEAR_SPEED/5
        twist_msg.angular.z = ANGULAR_SPEED
        if rospy.get_time()%10>=5:
         twist_msg.angular.z = -ANGULAR_SPEED
        if ranges[90]<0.6==ranges[-90]<0.6:
         if ranges[90]> ranges[-90]:
                twist_msg.angular.z = ANGULAR_SPEED
         elif ranges[90]<ranges[-90]:
             twist_msg.angular.z = -ANGULAR_SPEED



    else:
        #RAND = random.random()
        twist_msg.angular.z = 0.0
        twist_msg.linear.x = LINEAR_SPEED
    if tui==1:
        twist_msg.linear.x = -2*LINEAR_SPEED
        twist_msg.angular.z = 0.0
    if qian==1:
        twist_msg.linear.x = 1
        twist_msg.angular.z = 0.0
    cmd_vel_pub.publish(twist_msg)

if __name__ == '__main__':
    try:
    
        rospy.init_node('explorer', anonymous=True)

        time = rospy.get_rostime()
        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
        odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass

