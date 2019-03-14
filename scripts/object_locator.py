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


# if sim is True/using gazebo, therefore want to subscribe to /gazebo/model_states\
# otherwise, they will use a TF lookup
use_gazebo = rospy.get_param("sim")

# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")

OBJECT_STATE_COV = 0.01

print("object_locator settings:")
print("use_gazebo:", use_gazebo)
print("mapping:", mapping)

class ObjectLocator:
    def __init__(self, use_ekf=False):
        rospy.init_node('object_locator', log_level=rospy.INFO, anonymous=True)
        self.use_ekf = use_ekf
        # object dict name:(location, count)
        self.objects = {}
        # name:object_ekf
        self.object_ekfs = {}
        # robot location
        self.x = None
        self.y = None
        self.theta = None
        self.trans_listener = TransformListener()
        rospy.Subscriber('/detector/objects', DetectedObjectList, self.detected_object_callback)
        self.pub = rospy.Publisher('/object_location', ObjectLocationList, queue_size=10)

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
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                debug("object locator: got tf exception",e)
                pass

    def ekf_update_location(self, name, obj):
        if name not in self.object_ekfs:
            r_obj = msg.distance
            alpha_obj = wrapToPi((msg.thetaleft + msg.thetaright)/2)
            x = r_obj * np.sin(alpha_obj + self.theta) + self.x
            y = r_obj * np.cos(alpha_obj + self.theta) + self.y
            self.object_ekfs[name] = MeasurementObjectEKF(
                np.array([x,y]), OBJECT_STATE_COV*np.eye(2), 
                np.zeros((2,2))) # no contorl noise
        else:
            # doesn't matter since object doesn't move
            self.object_ekfs[name].transition_update(None, 0)
            self.object_ekfs[name].measurement_update(np.zeros[r_obj, alpha_obj], np.array([msg.confidence]))



    def naive_update_location(self, name, obj):
        r_obj = obj.distance
        alpha_obj = wrapToPi((obj.thetaleft + obj.thetaright)/2.0)
        x = r_obj * np.cos(alpha_obj + self.theta) + self.x
        y = r_obj * np.sin(alpha_obj + self.theta) + self.y
        debug("r_obj {}, alpha_obj {}, x {}, y {}, self x {}, self y {}".format(r_obj, alpha_obj, x, y, self.x, self.y))
        if name not in self.objects:
            self.objects[name] = ((x,y), 1)
        else:
            x_old, y_old = self.objects[name][0]
            c = self.objects[name][1]
            x_new = (c/float(c+1))*x_old + 1/float(c+1)*x
            y_new = (c/float(c+1))*y_old + 1/float(c+1)*y
            self.objects[name] = ((x_new, y_new), c+1) 
            debug("Saw {} {} times".format(name, c+1))


    def detected_object_callback(self, msg):
        self.update_current_pose()
        rospy.logdebug("Received detected object callback:")
        names = msg.objects
        obj_msgs = msg.ob_msgs
        for i in range(len(names)):
            name = names[i]
            obj = obj_msgs[i]
            rospy.logdebug("{}: {}".format(name, obj))
            if not self.use_ekf:
                self.naive_update_location(name, obj)
            else:
                self.ekf_update_location(name, obj)

    def publish_object_location(self):
        if self.objects:
            obj_list = ObjectLocationList()
            for name, obj in self.objects.iteritems():
                obj_loc = ObjectLocation()
                obj_loc.name = name
                obj_loc.x, obj_loc.y = obj[0]
                obj_loc.count = obj[1]
                obj_list.locations.append(obj_loc)
                rospy.logdebug("Publishing object {} at location {}".format(name, obj_loc))
            self.pub.publish(obj_list)

    def run(self):
        rate = rospy.Rate(10)  # Update detections 10 Hz
        counter = 0
        while not rospy.is_shutdown():
            if counter == 0: self.publish_object_location()
            counter = (counter + 1) % 10  # Publish 1 Hz
            rate.sleep()

if __name__ == '__main__':
    loc = ObjectLocator()
    loc.run()






