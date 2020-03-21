#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, Point

class Room():

    def __init__(self, ident):

        self.ident = ident
        self.name = ""
        self.description = ""
        self.position = Point()
        self.orientation = Point()

        self.nameGenerator(ident)

    def nameGenerator(self, indent):
        newName = "room_" + str(indent)
        self.setName(newName)

    def getName(self):
        return self.name
    
    def getDescription(self):
        return self.description
    
    def getPosition(self):
        return self.position
    
    def getOrientaion(self):
        return self.orientation

    def setName(self, name):
        self.name = name
    
    def setDescription(self, d):
        self.description = d
    
    def setPosition(self, p):
        self.position = p
    
    def setOrientaion(self, o):
        self.orientation = o

