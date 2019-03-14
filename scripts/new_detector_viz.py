#!/usr/bin/env python
import rospy
import os
# watch out on the order for the next two imports lol
from tf import TransformListener
import tensorflow as tf
import numpy as np
from sensor_msgs.msg import CompressedImage, Image, CameraInfo, LaserScan
from turtlebot_team5.msg import DetectedObject, DetectedObjectList
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import cv2
import math
import array 


CV2_FONT = cv2.FONT_HERSHEY_SIMPLEX
BOX_TIMEOUT = 1.0
mapping = rospy.get_param("map")
#mapping = 1
class DetectorViz:

	def __init__(self):
		rospy.init_node('turtlebot_detector', anonymous=True)
		self.bridge = CvBridge()
		self.x = 0
		self.y = 0
		self.theta = 0
		self.tf_listener = TransformListener()
		self.last_box_time = rospy.get_rostime()
		rospy.Subscriber('/detector/objects', DetectedObjectList, self.detected_objects_name_callback, queue_size=10)
		self.detected_objects = None
		rospy.Subscriber('/camera_relay/image_raw', Image, self.camera_callback, queue_size=1, buff_size=2**24)
		rospy.Subscriber('/camera_relay/image/compressed', CompressedImage, self.compressed_camera_callback, queue_size=1, buff_size=2**24)
		box_topic = 'box_topic'
		self.box_marker = rospy.Publisher(box_topic, Marker, queue_size=10)

	def load_image_into_numpy_array(self, img):
		""" converts opencv image into a numpy array """

		(im_height, im_width, im_chan) = img.shape
		return np.array(img.data).reshape((im_height, im_width, 3)).astype(np.uint8)

	def detected_objects_name_callback(self, msg):
		# rospy.loginfo("There are %i detected objects" % len(msg.objects))
		rospy.loginfo("Detected %i objects: " % len(msg.objects))
		self.detected_objects = msg
		if self.detected_objects:
			for obj in self.detected_objects.ob_msgs:
				rospy.loginfo(obj.name + " ")
		self.last_box_time = rospy.get_rostime()

	def camera_callback(self, msg):
		""" callback for camera images """

		try:
			img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
			img_bgr8 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError as e:
			print(e)
		if (msg.header.stamp.to_sec() - self.last_box_time.to_sec()) > BOX_TIMEOUT:
			self.detected_objects = None

		if np.abs(rospy.get_rostime().to_sec() - self.last_box_time.to_sec()) > BOX_TIMEOUT:
			self.detected_objects = None

		self.camera_common(img, img_bgr8)

	def compressed_camera_callback(self, msg):
		""" callback for camera images """

		# save the corresponding laser scan
		try:
			img = self.bridge.compressed_imgmsg_to_cv2(msg, "passthrough")
			img_bgr8 = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError as e:
			print(e)

		if np.abs(rospy.get_rostime().to_sec() - self.last_box_time.to_sec()) > BOX_TIMEOUT:
			self.detected_objects = None
		self.camera_common(img, img_bgr8)
		

	def camera_common(self, img, img_bgr8):
		(img_h,img_w,img_c) = img.shape
		draw_color = (0,255,0)
		if self.detected_objects is not None:
			for ob_msg in self.detected_objects.ob_msgs:
				ymin, xmin, ymax, xmax = [int(x) for x in ob_msg.corners]
				cv2.rectangle(img_bgr8, (xmin,ymin), (xmax,ymax), draw_color, 2)
				# cool add-on by student in 2018 class
				cv2.putText(img_bgr8, ob_msg.name + ":" + str(round(ob_msg.confidence, 2)), (xmin, ymin+13), CV2_FONT, .5, draw_color)
		cv2.imshow("Camera", img_bgr8)
		cv2.waitKey(1)

	def box_drawing(self, origin_frame):
		if self.detected_objects is not None:
			for ob_msg in self.detected_objects.ob_msgs:
				ymin, xmin, ymax, xmax = [int(x) for x in ob_msg.corners]
				theta_l = ob_msg.thetaleft
				theta_r = ob_msg.thetaright
				dist = ob_msg.distance
				xl = self.x + dist * np.cos(self.theta + theta_l)
				yl = self.y + dist * np.sin(self.theta + theta_l)
				xr = self.x + dist * np.cos(self.theta - theta_r)
				yr = self.y + dist * np.sin(self.theta - theta_r)
				z = 1 	# 1 meter height 
				X = array.array('f', [xl, xl, xr, xr, xl])  
				Y = array.array('f', [yl, yl, yr, yr, yl])  
				Z = array.array('f', [0, z, z, 0, 0])  

				for i in range(4):
					# start to draw line
					marker = Marker()
					marker.action = Marker.ADD
					t = rospy.Duration(0.2)
					marker.lifetime = t
					marker.header.frame_id = origin_frame
					marker.header.stamp = rospy.Time.now()
					marker.ns = 'bounding box'
									# Arrow (ARROW=0), Cube (CUBE=1), Sphere (SPHERE=2), Cylinder (CYLINDER=3)
									# Line Strip (LINE_STRIP=4), Line List (LINE_LIST=5), Cube List (CUBE_LIST=6)
									# Sphere List (SPHERE_LIST=7)
					# m.type = Marker.LINE_STRIP
					marker.type = Marker.LINE_STRIP
					# marker scale
					marker.scale.x = 0.03
					marker.scale.y = 0.03
					marker.scale.z = 0.03

					# marker color
					marker.color.a = 1.0
					marker.color.r = 0.0
					marker.color.g = 1.0
					marker.color.b = 0.0

					# marker orientaiton
					marker.pose.orientation.x = 0.0
					marker.pose.orientation.y = 0.0
					marker.pose.orientation.z = 0.0
					marker.pose.orientation.w = 1.0

					# marker position
					marker.pose.position.x = 0.0
					marker.pose.position.y = 0.0
					marker.pose.position.z = 0.0
					marker.points = []
					# first point
					first_line_point = Point()
					first_line_point.x = X[i]
					first_line_point.y = Y[i]
					first_line_point.z = Z[i]
					marker.points.append(first_line_point)
					# second point
					second_line_point = Point()
					second_line_point.x = X[i + 1]
					second_line_point.y = Y[i + 1]
					second_line_point.z = Z[i + 1]
					marker.points.append(second_line_point)
					# Publish the Marker
					self.box_marker.publish(marker)
	def loop(self):
		try:
			origin_frame = "/map" if mapping else "/odom"
			(translation,rotation) = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
			self.x = translation[0]
			self.y = translation[1]
			euler = tf.transformations.euler_from_quaternion(rotation)
			self.theta = euler[2]
			self.box_drawing(origin_frame)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			pass

	def run(self):
		while not rospy.is_shutdown():
			self.loop()
			rate.sleep()

if __name__=='__main__':
	d = DetectorViz()
	d.run()
