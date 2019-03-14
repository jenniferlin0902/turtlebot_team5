#!/usr/bin/env python
import rospy, sys, os, time
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped, Vector3, Point
from utils import log, error, debug
import numpy as np
import tf
import math
from enum import Enum
# how is nav_cmd being decided -- human manually setting it, or rsviz
rviz = rospy.get_param("rviz")
# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")
class TurtleBotMaker:
    def __init__(self):
        rospy.init_node('model_marker')
        self.x = 0
        self.y = 0
        self.trans_listener = tf.TransformListener()
    def printout(self)
        print('''Commands:
            -x
            ''', self.x, 
            '''-y
            ''', self.y,
            '''-z
            ''', 0)

    def loop(self):
        # if not use_gazebo:
            try:
                origin_frame = "/map" if mapping else "/odom"
                (translation,rotation) = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
                self.x = translation[0]
                self.y = translation[1]
                sys.exit(1)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                if rospy.is_shutdown():
                  sys.exit(0)
                pass

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    marker.run()
    printout()

# <node name="model_marker" type="model_marker" args="-urdf -model turtlebot3_burger -x $(arg x_pos) -y $(arg y_pos) -z $(arg z_pos) -param robot_description" />



