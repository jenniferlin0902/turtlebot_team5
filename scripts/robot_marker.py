#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray, String
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped, Vector3, Point
from turtlebot_team5.msg import DetectedObjectList, DetectedObject, ObjectLocationList, ObjectLocation #
from utils import log, error, debug
import numpy as np
import tf
import math
from enum import Enum
use_gazebo = rospy.get_param("sim")
# how is nav_cmd being decided -- human manually setting it, or rsviz
rviz = rospy.get_param("rviz")
# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")
class TurtleBotMaker:
    def __init__(self):
        rospy.init_node('turtlbot_marker')
        self.x = 0
        self.y = 0
        self.theta = 0
        self.trans_listener = tf.TransformListener()
        self.x_g=0
        self.y_g=0
        self.theta_g=0
        self.home_check = True
        robot_topic = 'robot_marker'
        robot_des_topic = 'robot_des_marker'
        goal_topic = 'goal_marker'
        location_topic = 'location_topic'
        self.robot_marker_publisher = rospy.Publisher(robot_topic, Marker, queue_size=15)
        self.robot_des_publisher = rospy.Publisher(robot_des_topic, Marker, queue_size=15)
        self.goal_marker_publisher = rospy.Publisher(goal_topic, Marker, queue_size=15)
        self.location_marker_publisher = rospy.Publisher(location_topic, Marker, queue_size=15)
        
        # how is nav_cmd being decided -- human manually setting it, or rviz
    
        self.objectList = None
        self.objectLocatorList = None
        self.doneList = []
        self.name2id = {} 
        self.delivery_requests = []
        
        self.num_obj = 0
        rospy.Subscriber('/cmd_pose', Pose2D, self.cmd_pose_callback)
        rospy.Subscriber('/detector/objects', DetectedObjectList, self.object_callback, queue_size=10)
        rospy.Subscriber('/object_location', ObjectLocationList, self.objectLocator_callback, queue_size=10)
        rospy.Subscriber('/delivery_request', String, self.delivery_request_callback)

    def cmd_pose_callback(self, data):
        ######### YOUR CODE HERE ############
        # fill out cmd_pose_callback
        self.x_g = data.x
        self.y_g = data.y
        self.theta_g = data.theta
        ######### END OF YOUR CODE ##########

    def gazebo_callback(self, msg):
        pose = msg.pose[msg.name.index("turtlebot3_burger")]
        twist = msg.twist[msg.name.index("turtlebot3_burger")]
        self.x = pose.position.x
        self.y = pose.position.y
        quaternion = (
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.theta = euler[2]

    def publish_marker(self, origin_frame):
            if self.home_check:
                marker = Marker()
                marker.header.stamp = rospy.Time(0)
                marker.header.frame_id = origin_frame
                marker.text = "Home"
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.id = 999 # newly added
                marker.scale.x = 0.3
                marker.scale.y = 0.3 # 15cm
                marker.scale.z = 0.3 # 15cm
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.pose.orientation.w = self.theta
                marker.pose.position.x = self.x
                marker.pose.position.y = self.y
                self.robot_marker_publisher.publish(marker)
                self.home_check = False
                print("Home marker ====== ")

            if self.objectLocatorList:
                for obj in self.objectLocatorList:
                    if obj.name not in self.name2id:  # name2id[name] = id
                        obj_id = self.num_obj
                        self.name2id[obj.name] = obj_id
                        self.num_obj += 1
                    else:
                        obj_id = self.name2id[obj.name]
                    marker = Marker()
                    marker.header.stamp = rospy.Time(0)
                    marker.header.frame_id = origin_frame
                    marker.type = marker.CUBE
                    marker.text = "Object"
                    marker.id = obj_id
                    marker.action = marker.ADD
                    marker.scale.x = 0.15
                    marker.scale.y = 0.15  # 30cm
                    marker.scale.z = 0.15  # 30cm
                    marker.color.a = 1.0
                    if obj.name == "banana":
                        marker.color.r = 255.0/255.0
                        marker.color.g = 255.0/255.0
                        marker.color.b = 0.0
                    elif obj.name == "donut":
                        marker.color.r = 255.0/255.0
                        marker.color.g = 153.0/255.0
                        marker.color.b = 204.0/255.0
                    elif obj.name == "broccoli":
                        marker.color.r = 0.0/255.0
                        marker.color.g = 153.0/255.0
                        marker.color.b = 51.0/255.0
                    elif obj.name == "apple":
                        marker.color.r = 255.0/255.0
                        marker.color.g = 51.0/255.0
                        marker.color.b = 0.0
                    elif obj.name == "stop_sign":
                        marker.color.r = 255.0/255.0
                        marker.color.g = 102.0/255.0
                        marker.color.b = 0.0/255.0
                    else: # blue
                        marker.color.r = 0.0/255.0
                        marker.color.g = 102.0/255.0
                        marker.color.b = 255.0/255.0
                    marker.pose.position.x = obj.x
                    marker.pose.position.y = obj.y
                    self.location_marker_publisher.publish(marker)

    
    def publish_des_marker(self, origin_frame): # robot_locator      
                marker = Marker()
                marker.header.stamp = rospy.Time(0)
                marker.header.frame_id = origin_frame
                marker.type = marker.SPHERE
                marker.text = "Robot"
                marker.action = marker.ADD
                marker.scale.x = 0.15
                marker.scale.y = 0.15 # 15cm
                marker.scale.z = 0.15 # 15cm
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 51.0/255.0
                marker.color.b = 153.0/255.0
                marker.pose.orientation.w = self.theta
                marker.pose.position.x = self.x
                marker.pose.position.y = self.y
                self.robot_des_publisher.publish(marker)
                print("Robot marker ====== ")
                    
    def publish_goal_marker(self, origin_frame): # robot_locator
        for request in self.delivery_requests:
            if self.objectLocatorList:  
                for obj in self.objectLocatorList:
                    if obj.name == request:
                        marker = Marker()
                        marker.header.stamp = rospy.Time(0)
                        marker.header.frame_id = origin_frame
                        marker.type = marker.CYLINDER
                        marker.text = "Goal"
                        marker.action = marker.ADD
                        marker.scale.x = 0.15
                        marker.scale.y = 0.15 # 15cm
                        marker.scale.z = 0.3 # 15cm
                        marker.color.a = 0.5
                        marker.pose.position.x = obj.x
                        marker.pose.position.y = obj.y
                        if request == "banana":
                            marker.color.r = 255.0/255.0
                            marker.color.g = 255.0/255.0
                            marker.color.b = 0.0
                        elif request == "donut":
                            marker.color.r = 255.0/255.0
                            marker.color.g = 153.0/255.0
                            marker.color.b = 204.0/255.0
                        elif request == "broccoli":
                            marker.color.r = 0.0/255.0
                            marker.color.g = 153.0/255.0
                            marker.color.b = 51.0/255.0
                        elif request == "apple":
                            marker.color.r = 255.0/255.0
                            marker.color.g = 51.0/255.0
                            marker.color.b = 0.0
                        elif request == "stop_sign":
                            marker.color.r = 255.0/255.0
                            marker.color.g = 102.0/255.0
                            marker.color.b = 0.0/255.0
                        else: # blue
                            marker.color.r = 0.0/255.0
                            marker.color.g = 102.0/255.0
                            marker.color.b = 255.0/255.0
                        self.goal_marker_publisher.publish(marker)



            
    def object_callback(self, msg):
        self.objectList = msg.ob_msgs

    def objectLocator_callback(self, msg):
        self.objectLocatorList = msg.locations #changed from msg

    def delivery_request_callback(self, msg):
        items = msg.data.lower().strip().split(",")
        items = [i.strip() for i in items]
        debug("Supervisor: delivery_request_callback: Got {} items:".format(len(items)), items)
        if len(items) > 0:
            self.delivery_requests.extend(items)
            self.delivery_requests.append("home")
    


    def loop(self):
            try:
                origin_frame = "/map" if mapping else "/odom"
                (translation,rotation) = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
                self.x = translation[0]
                self.y = translation[1]
                euler = tf.transformations.euler_from_quaternion(rotation)
                self.theta = euler[2]
                self.publish_marker(origin_frame)
                self.publish_des_marker(origin_frame)
                self.publish_goal_marker(origin_frame)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass


    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep

if __name__ == '__main__':
    marker = TurtleBotMaker()
    marker.run()
