#!/usr/bin/env python
import rospy
from explorer_turtle.msg import Marker, MarkerArray #pylint:disable=import-error
import numpy as np
from statistics import stdev, mean

NUMBER_SAMPLES = 1000000
NUMBER_MARKERS = 8
KEY_NUMBER = 1024

def callback(msg):

    sample = []

    for i in range(NUMBER_SAMPLES):
        d = msg.angle
        sample.append(d)

    #sigma = stdev(sample)
    arr = np.array(sample)
    sigma = np.std(arr)

    print('{0:.16f}'.format(sigma))


def marker_callback(msg):
    
    for i in range(NUMBER_MARKERS):
        try:
            dist = msg.markers[i].distance
            th = msg.markers[i].angle
            index = msg.markers[i].id
        except:
            break
        list_ids[i] = index
        markers_info[i] = (dist, th)
    
    print(list_ids, markers_info)
    
def statistic():

    rospy.init_node('statistic', anonymous=True)

    rospy.Subscriber("/explorer/marker", Marker, callback)
    #rospy.Subscriber("/explorer/markers", MarkerArray, marker_callback)

    rospy.spin()

if __name__ == '__main__':

    markers_info = [None]*NUMBER_MARKERS
    list_ids = np.ones(NUMBER_MARKERS, dtype='int32')*KEY_NUMBER

    statistic()
