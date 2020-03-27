#!/usr/bin/env python

import math
import rospy
import sys
import time
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import Rectangle
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class SquareMoveVel():
  
    def __init__(self):

        self.dataOdomX = []
        self.dataOdomY = []

        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=2)
        rospy.Subscriber("odom", Odometry, self.odom_callback)

        self.pub_rate = 0.1

    def odom_callback(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.dataOdomX.append(x)
        self.dataOdomY.append(y)

    def stop_robot(self):

        # Get the initial time
        self.t_init = time.time()

        # We publish for a second to be sure the robot receive the message
        while time.time() - self.t_init < 1 and not rospy.is_shutdown():
            
            self.vel_pub.publish(Twist())
            time.sleep(self.pub_rate)
        self.viewGraph()
        self.save()
        sys.exit("The process has been interrupted by the user!")

    def go_forward(self, duration, speed):

        # Get the initial time
        self.t_init = time.time()

        # Set the velocity forward and wait (do it in a while loop to keep publishing the velocity)
        while time.time() - self.t_init < duration and not rospy.is_shutdown():

            msg = Twist()
            msg.linear.x = speed
            msg.angular.z = 0
            self.vel_pub.publish(msg)
            time.sleep(self.pub_rate)
    def save(self):
        with open("odomentry_x.txt", "w") as odom_x:
            for line in self.dataOdomX:
                odom_x.write(str(line)+ "\n")
            odom_x.close()
        with open("odomentry_y.txt", "w") as odom_y:
            for line in self.dataOdomY:
                odom_y.write(str(line)+ "\n")
            odom_y.close()
        with open("odomentry_x.txt", "w") as odom_x:
            for line in self.dataOdomX:
                odom_x.write(str(line)+ "\n")
            odom_x.close()
    

    def turn(self, duration, ang_speed):

         # Get the initial time
        self.t_init = time.time()

        # Set the velocity forward and wait 2 sec (do it in a while loop to keep publishing the velocity)
        while time.time() - self.t_init < duration and not rospy.is_shutdown():

            msg = Twist()
            msg.linear.x = 0
            msg.angular.z = ang_speed
            self.vel_pub.publish(msg)
            time.sleep(self.pub_rate)

    def move(self):

        self.go_forward(6, 0.5) # let's go forward at 1.0 m/s

        self.turn(3.5, 0.5)

        self.go_forward(6, 0.5)

        self.turn(3.5, 0.5)

        self.go_forward(6, 0.5)

        self.turn(3.5, 0.5)

        self.go_forward(6, 0.5)

        self.stop_robot()

    def viewGraph(self):

        plt.figure()
        currentAxis = plt.gca()
        currentAxis.add_patch(Rectangle((0.0, 0.0), 3.0, 3.0, fill=None, alpha=1, edgecolor="green", label="Origin"))

        plt.plot(self.dataOdomX, self.dataOdomY, 'r', label="Odometry")
        plt.ylabel('axis[y]')
        plt.xlabel('axis[x]')

        plt.xlim((-1, 4))   # set the xlim to left, right
        plt.ylim((-1, 4))     # set the xlim to left, right

        plt.legend()


        plt.show()

if __name__ == '__main__':
    
    rospy.init_node("square_move")
    r = SquareMoveVel()
    r.move()
