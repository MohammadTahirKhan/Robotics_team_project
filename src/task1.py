#!/usr/bin/env python3

import rospy

# import the Twist message for publishing velocity commands:
from geometry_msgs.msg import Twist

# import the Odometry message for subscribing to the odom topic:
from nav_msgs.msg import Odometry

# import the function to convert orientation from quaternions to angles:
from tf.transformations import euler_from_quaternion

# import some useful mathematical operations (and pi):
from math import sqrt, pow, pi

class Task1():
    def callback_function(self, topic_data: Odometry):
        # obtain relevant topic data: pose (position and orientation):
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
        self.theta_z = yaw

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



    def __init__(self):
        node_name = "task1"
        # a flag if this node has just been launched
        self.startup = True
        # setup a '/cmd_vel' publisher and an '/odom' subscriber:
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback_function)
        
        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(10)  # hz
        
        # define the robot pose variables and initialise them to zero:
        # variables for the robot's "current position":
        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        # variables for a "reference position":
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0
        # define a Twist message instance, to set robot velocities
        self.vel = Twist()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"the {node_name} node has been initialised...")

    def shutdownhook(self):
        # publish an empty twist message to stop the robot
        # (by default all velocities will be zero):
        self.pub.publish(Twist())
        self.ctrl_c = True

    def main_loop(self):
        current_movement = 0
        loop_num = 1
        while not self.ctrl_c:
            
            dist_moved = sqrt(pow(self.x-self.x0,2)+pow(self.y-self.y0,2))
            current_movement = current_movement + abs(dist_moved)
            self.x0 = self.x
            self.y0 = self.y

            if current_movement >= 2*pi*0.5 and loop_num==1:
                # stop after first loop completes, getting started for the next loop
                self.vel = Twist()
                loop_num +=1

            if current_movement >= 4*pi*0.5:
                # 2 loops completed, stop the robot
                self.shutdownhook()

            if loop_num==2 :
                # second loop,
                # angular velocity is negative now to change the direction to clockwise
                self.vel.angular.z = -2*pi/30
                self.vel.linear.x = pi/30
            else:
                # first loop, 
                # linear vel is pi/30 because vel = distance/time = 2*pi*0.5 (m) /30 (sec)
                # angular vel is twice the linear velocity because,
                # angular vel = linear vel / radius = (pi/30)/0.5 = 2*pi/30
                self.vel.angular.z = 2*pi/30
                self.vel.linear.x = pi/30
            
            print(f"x = {self.x:.2f} [m], y = {self.y:.2f} [m], theta_z = {self.theta_z*(180/pi):.1f} [degrees]")

            # publish velocity command that has been set in your code above:
            self.pub.publish(self.vel)
            # maintain the loop rate @ 1 hz
            self.rate.sleep()

if __name__ == "__main__":
    node = Task1()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass
