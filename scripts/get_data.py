#! /usr/bin/env python

import rospy
import math
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

odomDataX = []
odomDataY = []
odomDataYaw = []
twistDataX = []
twistDataY = []
twistDataYaw = []

class find():

    def __init__(self):
        
        sub_odom = rospy.Subscriber('/odom', Odometry, self.OdomCallback)
        sub_tele = rospy.Subscriber('/mobile_base/commands/velocity', Twist, self.TwistCallback )

        rospy.loginfo("publishing twist.")

        self.rate = rospy.Rate(10)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

    def TwistCallback(self, msg):

        x = msg.linear.x
        y = msg.linear.y
        yaw = msg.angular.z

        twistDataX.append(x)
        twistDataY.append(y)
        twistDataYaw.append(yaw)

    def OdomCallback(self, msg):

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion (orientation_list)

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        odomDataX.append(x)
        odomDataY.append(y)
        odomDataYaw.append(yaw)

if __name__ == '__main__':
    try:
        rospy.init_node('get_data' , anonymous=True)
        f = find()
        f.run()
    except rospy.ROSInterruptException: pass
    '''
    with open("odomentry.txt", "w") as txt_file:
        for line in odomData:
            txt_file.write(', '.join(map(str, line)) + "\n")
    '''
    plt.plot(odomDataX, odomDataY, 'r', twistDataX, twistDataY, 'b')
    plt.ylabel('axis[y]')
    plt.xlabel('axis[x]')

    #plt.xlim((-1, 4))   # set the xlim to left, right
    #plt.ylim((-1, 4))     # set the xlim to left, right
    plt.show()