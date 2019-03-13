#!/usr/bin/env python

import rospy
import numpy as np
from turtlebot_team5.msg import ObjectLocation
from geometry_msgs.msg import Pose2D
from utils import wrapToPi


# if sim is True/using gazebo, therefore want to subscribe to /gazebo/model_states\
# otherwise, they will use a TF lookup
use_gazebo = rospy.get_param("sim")

# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")

log("object_locator settings:")
log("use_gazebo:", use_gazebo)
log("mapping:", mapping)

class ObjectLocator:
    def __init__(self):
        rospy.init_node('object_locator', log_level=rospy.DEBUG, anonymous=True)
        # object lists
        self.objects = []
        # robot location
 		self.x
 		self.y
        self.trans_listener = tf.TransformListener()

        rospy.Subscriber('/detector/objects', DetectedObjectList, self.dectected_object_callback)
        self.pub = rospy.Publisher('/object_location', ObjectLocation, queue_size=10)

    def update_current_pose(self):
    	# TODO check if we should use /base_footprint or use the camera location (raspicam or camera?)
        if not use_gazebo:
            try:
                origin_frame = "/map" if mapping else "/odom"
                (translation,rotation) = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
                self.x = translation[0]
                self.y = translation[1]
                euler = tf.transformations.euler_from_quaternion(rotation)
                self.theta = euler[2]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                log("got tf exception")
                pass

    def detected_object_callback(self, msg):
    	self.update_current_pose()
    	obj_name = msg.name
    	# use naive method for now, will change to filtering later
    	obj_r = msg.distance
    	obj_alpha = wrapToPi((msg.thetaleft + msg.thetaright)/2)
    	x = obj_r * np.sin(obj_alpha + self.theta)
    	y = obj_r * np.cos(obj_alpha + self.theta)

    	obj_loc_msg = ObjectLocation()
    	loc = Pose2D()
    	obj_loc_msg.name = obj_name
    	loc.x = x
    	loc.y = y
    	loc.theta = 0 # orientataion doesn't matter here
    	obj_loc_msg.location = loc

    	self.objects.append(obj_loc_msg)

    def run(self):
    	rospy.loginfo("object locator started")
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
    		while len(self.objects.append(obj_loc_msg)):
    			self.pub.publish(self.objects.pop())
    		rate.sleep()

if __name__ == '__main__':
    loc = ObjectLocator()
    loc.run()






