#!/usr/bin/env python

import rospy
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point
from tf.transformations import quaternion_from_euler

class MarkerRviz():

	def __init__(self, name, id, position, angles, scale, color):

		self.marker = Marker()
		self.marker.header.frame_id = name
		self.marker.header.stamp = rospy.get_rostime()
		self.marker.ns = 'door_'+ name
		self.marker.id = id
		self.marker.action =  Marker.ADD
		self.marker.pose.position.x = position[0]
		self.marker.pose.position.y = position[1]
		self.marker.pose.position.z = position[2]
		q = quaternion_from_euler(angles[0],angles[1],angles[2])
		self.marker.pose.orientation.x = q[0]
		self.marker.pose.orientation.y = q[1]
		self.marker.pose.orientation.z = q[2]
		self.marker.pose.orientation.w = q[3]
		self.marker.scale.x = scale[0]
		self.marker.scale.y = scale[1]
		self.marker.scale.z = scale[2]
		self.marker.color.r = color[0]
		self.marker.color.g = color[1]
		self.marker.color.b = color[2]
		self.marker.color.a = 1.0

		self.marker.type = Marker.CUBE

		self.pub_send_marker = rospy.Publisher('/explorer/totens_'+name, Marker, queue_size=10)

	def publish(self):
		self.marker.header.stamp = rospy.get_rostime()
		self.pub_send_marker.publish(self.marker)
