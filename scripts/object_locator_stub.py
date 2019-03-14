#!/usr/bin/env python

import rospy
import numpy as np
from turtlebot_team5.msg import ObjectLocation, ObjectLocationList
from turtlebot_team5.msg import DetectedObject, DetectedObjectList
from geometry_msgs.msg import Pose2D
from ekf import MeasurementObjectEKF
from utils import wrapToPi, log, debug, error
from tf import TransformListener
import tf


# # if sim is True/using gazebo, therefore want to subscribe to /gazebo/model_states\
# # otherwise, they will use a TF lookup
# use_gazebo = rospy.get_param("sim")

# # if using gmapping, you will have a map frame. otherwise it will be odom frame
# mapping = rospy.get_param("map")

# OBJECT_STATE_COV = 0.01

# print("object_locator settings:")
# print("use_gazebo:", use_gazebo)
# print("mapping:", mapping)

class ObjectLocatorStub:
    def __init__(self, use_ekf=False):
        rospy.init_node('object_locator_stub', log_level=rospy.INFO, anonymous=True)
        self.use_ekf = use_ekf
        # object dict name:(location, count)
        self.objects = {}
        # name:object_ekf
        self.object_ekfs = {}
        # robot location
        self.x = 0
        self.y = 0
        self.trans_listener = TransformListener()
        self.pub = rospy.Publisher('/object_location', ObjectLocationList, queue_size=10)

    def publish_object_location(self):
        obj_list = ObjectLocationList()
        obj_loc = ObjectLocation()
        obj_loc.name = "FAKE_BOTTLE"
        obj_loc.x = self.x
        obj_loc.y = self.y
        obj_loc.count = 3
        obj_list.locations.append(obj_loc)
        self.pub.publish(obj_list)

    def run(self):
        rate = rospy.Rate(1)
        counter = 0
        while not rospy.is_shutdown():
            self.publish_object_location()
            exit(1)

if __name__ == '__main__':
    loc = ObjectLocatorStub()
    loc.run()
