#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray, String
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped, Vector3, Point
from asl_turtlebot.msg import DetectedObjectList #
from asl_turtlebot.msg import DetectedObject #
import numpy as np
import tf
import math
from enum import Enum


# use_gazebo = rospy.get_param("sim")


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
        robot_topic = 'robot_marker'
        self.robot_marker_publisher = rospy.Publisher(robot_topic, Marker, queue_size=10)

        goal_topic = 'goal_marker'
        self.goal_marker_publisher = rospy.Publisher(goal_topic, Marker, queue_size=10)
        self.x_g=0
        self.y_g=0
        self.theta_g=0

        
        angle_topic = 'angle_topic'
        marker_pub = rospy.Publisher('marker_test', Marker, queue_size = 10)
        self.marker_line = rospy.Publisher(angle_topic, Marker, queue_size=10)
        # how is nav_cmd being decided -- human manually setting it, or rviz
        

        rospy.Subscriber('/cmd_pose', Pose2D, self.cmd_pose_callback)
        # # if using gazebo, we have access to perfect state
        # if use_gazebo:
        #     rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)

        self.objectList = None
        rospy.Subscriber('/detector/objects', DetectedObjectList, self.object_callback, queue_size=10)
        

    def cmd_pose_callback(self, data):
        ######### YOUR CODE HERE ############
        # fill out cmd_pose_callback
        self.x_g = data.x
        self.y_g = data.y
        self.theta_g = data.theta
        print("GOT cmd pose {}", (data.x, data.y))
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
            marker = Marker()
            marker.header.stamp = rospy.Time(0)
            marker.header.frame_id = origin_frame
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.15
            marker.scale.y = 0.15 # 15cm
            marker.color.a = 1.0
            marker.color.r = 197/256.0
            marker.color.g = 244/256.0
            marker.color.b = 66/256.0
            marker.pose.orientation.w = self.theta
            marker.pose.position.x = self.x
            marker.pose.position.y = self.y
            self.robot_marker_publisher.publish(marker)

            marker = Marker()
            marker.header.stamp = rospy.Time(0)
            marker.header.frame_id = origin_frame
            marker.type = marker.CYLINDER
            marker.action = marker.ADD
            marker.scale.x = 0.15
            marker.scale.y = 0.15 # 15cm
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.pose.orientation.w = self.theta_g
            marker.pose.position.x = self.x_g
            marker.pose.position.y = self.y_g
            self.goal_marker_publisher.publish(marker)

            
    def object_callback(self, msg):
        self.objectList = msg

    def publish_line(self, origin_frame):
        if self.objectList:
            for obj in self.objectList.ob_msgs:
            # if obj.ob_msgs.name == "stop sign": # to be modified based on the object classes
                                                  # using different marker 
                theta_l = obj.thetaleft
                theta_r = obj.thetaright
                dist = obj.distance
                # scale = Vector3(2,4,0.69)
                scale = Vector3(0.1,0.1,0.1)
                self.marker_line.publish(self.make_arrow_points_marker(scale, Point(self.x, self.y, 0), 
                            Point(self.x + dist * np.cos(self.theta + theta_l) , 
                                  self.y + dist * np.sin(self.theta + theta_l) ,0), 3, origin_frame))
                
                self.marker_line.publish(self.make_arrow_points_marker(scale, Point(self.x, self.y, 0), 
                            Point(self.x + dist * np.cos(self.theta - theta_r) , 
                                  self.y + dist * np.sin(self.theta - theta_r) ,0), 3, origin_frame))
                # print('****** Done ******')
            self.objectList = []



    def make_arrow_points_marker(self, scale, tail, tip, idnum, origin_frame):
        # make a visualization marker array for the occupancy grid
        m = Marker()
        m.action = Marker.ADD
        t = rospy.Duration(0.2)
        # t = rospy.Duration()
        m.lifetime = t
        # m.header.frame_id = '/base_link'
        m.header.frame_id = origin_frame
        m.header.stamp = rospy.Time.now()
        m.ns = 'points_arrows'
        m.id = idnum    # Arrow (ARROW=0), Cube (CUBE=1), Sphere (SPHERE=2), Cylinder (CYLINDER=3)
                        # Line Strip (LINE_STRIP=4), Line List (LINE_LIST=5), Cube List (CUBE_LIST=6)
                        # Sphere List (SPHERE_LIST=7)
        # m.type = Marker.LINE_STRIP
        m.type = Marker.ARROW
        m.pose.orientation.y = 0
        m.pose.orientation.w = 1
        m.scale = scale
        m.color.r = 100/256.0
        m.color.g = 100/256.0
        m.color.b = 100/256.0
        m.color.a = 0.8
        m.points = [ tail, tip ]
        # m.action = Marker.DELETE
        return m

    def loop(self):
        # if not use_gazebo:
            try:
                origin_frame = "/map" if mapping else "/odom"
                (translation,rotation) = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
                self.x = translation[0]
                self.y = translation[1]
                euler = tf.transformations.euler_from_quaternion(rotation)
                self.theta = euler[2]
                self.publish_marker(origin_frame)
                self.publish_line(origin_frame)
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
