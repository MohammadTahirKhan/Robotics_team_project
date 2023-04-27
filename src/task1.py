#!/usr/bin/env python3

import rospy

# import the Twist message for publishing velocity commands:
from geometry_msgs.msg import Twist

# import the Odometry message for subscribing to the odom topic:
from nav_msgs.msg import Odometry

# import the function to convert orientation from quaternions to angles:
from tf.transformations import euler_from_quaternion

# import some useful mathematical operations (and pi):
from math import pi

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
        self.theta_z = abs(yaw)
        # make the initial yaw and the later yaw positions relative to zero
        self.theta_z = self.theta_z - self.initial_yaw

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
            self.initial_yaw = self.theta_z

        

    def __init__(self):
        node_name = "task1"
        # a flag if this node has just been launched
        self.startup = True
        # initial yaw of the robot, later used to change the starting yaw to zero if not already
        self.initial_yaw = 0.0
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
        current_yaw = 0
        loop_num = 1
        print_count = 0
        print_yaw = 0.0
        while not self.ctrl_c:
            current_yaw = current_yaw + abs(self.theta_z - self.theta_z0 )
            self.theta_z0 = self.theta_z

            if current_yaw >= 2*pi and loop_num==1:
                # stop after first loop completes, getting started for the next loop
                self.vel = Twist()
                loop_num +=1

            if current_yaw >= 4*pi:
                # 2 loops completed, stop the robot
                self.shutdownhook()

            if loop_num==2 :
                # second loop,
                # angular velocity is negative now, to change the direction to clockwise
                self.vel.angular.z = -2*pi/30
                self.vel.linear.x = pi/30
            else:
                # first loop, 
                # linear vel is pi/30 because vel = distance/time = 2*pi*0.5 (m) /30 (sec)
                # angular vel is twice the linear velocity because,
                # angular vel = linear vel / radius = (pi/30)/0.5 = 2*pi/30
                self.vel.angular.z = 2*pi/30
                self.vel.linear.x = pi/30
            
            # print at the speed of 1 hz
            if print_count%10==0:
                # current yaw to print
                if current_yaw > 3*pi:
                    # +180 to 0 degrees
                    print_yaw = 4*pi - current_yaw
                elif current_yaw > 2*pi:
                    # 0 to -180 degrees
                    print_yaw = 2*pi - current_yaw
                elif current_yaw > pi:
                    # -180 to 0 degrees
                    print_yaw = current_yaw - 2*pi
                else:
                    # 0 to 180 degrees
                    print_yaw = current_yaw
                print(f"x={self.x-self.x0:.2f} [m], y={self.y-self.y0:.2f} [m], yaw={print_yaw*180/pi:.1f} [degrees].")

            print_count+=1
            # publish velocity command that has been set in code above:
            self.pub.publish(self.vel)
            # maintain the loop rate @ 10 hz
            self.rate.sleep()

if __name__ == "__main__":
    node = Task1()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass
