#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, PoseArray, Pose2D
from std_msgs.msg import Float32MultiArray, String
import tf
import numpy as np
from numpy import linalg
from utils import wrapToPi, log, debug, error
from enum import Enum

# control gains
K1 = 0.4 #originally 0.4
K2 = 0.8 #originally 0.8
K3 = 0.8 #originally 0.4

# tells the robot to stay still
# if it doesn't get messages within that time period
TIMEOUT = np.inf

# maximum velocity
V_MAX = 0.1

# maximim angular velocity
W_MAX = 0.5

# whether goal is from rviz
# need to find a better way to get this info
RVIZ = 1

DIST_PREC = 0.06
YAW_PREC = np.pi*(10/180.0)
YAW_STEP_SMALL = 0.1
YAW_STEP_LARGE = 0.5

# Robot will fix its direction first if it is off by more than FIX_YAW_THRESHOLD degree

# if sim is True/using gazebo, therefore want to subscribe to /gazebo/model_states\
# otherwise, they will use a TF lookup (hw2+)
use_gazebo = rospy.get_param("sim")

# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")

log("pose_controller settings:")
log("use_gazebo:", use_gazebo)
log("mapping:", mapping)

class PCState(Enum):
    IDLE = 1
    FIX_YAW_INIT = 2
    MOVE_FWD = 3
    FIX_YAW_FINAL = 4


class PoseController:
    def __init__(self):
        rospy.init_node('turtlebot_pose_controller_nav', log_level=rospy.INFO, anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # current state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # goal state
        self.x_g = None
        self.y_g = None
        self.theta_g = None  

        # FSM 
        self.state = PCState.IDLE

        # time last pose command was received
        self.cmd_pose_time = rospy.get_rostime()
        # if using gazebo, then subscribe to model states
        if use_gazebo:
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
 
        self.trans_listener = tf.TransformListener()

        rospy.Subscriber('/step_goal_pose', Pose2D, self.cmd_pose_callback)
        

    def gazebo_callback(self, data):
        if "turtlebot3_burger" in data.name:
            pose = data.pose[data.name.index("turtlebot3_burger")]
            twist = data.twist[data.name.index("turtlebot3_burger")]
            self.x = pose.position.x
            self.y = pose.position.y
            quaternion = (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.theta = euler[2]

    def change_state(self, state):
        if state != self.state:
            log("PoseController: State changed to {}".format(state))
        self.state = state

    def cmd_pose_callback(self, data):
        debug("in cmd pose callback")
        if data.x == self.x_g and data.y == self.y_g:
            return

        self.x_g = data.x
        self.y_g = data.y
        self.theta_g = data.theta

        self.cmd_pose_time = rospy.get_rostime()
        log("PoseController: Got new goal x:{} y:{}, theta:{}".format(self.x_g, self.y_g, self.theta_g))
        # start moving, always turn first
        self.change_state(PCState.MOVE_FWD)

    def update_current_pose(self):
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

    def get_ctrl_output_fwd(self):
        """ runs a simple feedback pose controller """
        debug("Getting ctrl output fwd - goal x:{} y:{} theta:{}".format(self.x_g, self.y_g, self.theta_g))
        if (rospy.get_rostime().to_sec()-self.cmd_pose_time.to_sec()) < TIMEOUT:
            rel_coords = np.array([self.x-self.x_g, self.y-self.y_g])
            R = np.array([[np.cos(self.theta_g), np.sin(self.theta_g)], [-np.sin(self.theta_g), np.cos(self.theta_g)]])
            rel_coords_rot = np.dot(R,rel_coords)

            rho = linalg.norm(rel_coords) 
            ang = np.arctan2(rel_coords_rot[1],rel_coords_rot[0])+np.pi 
            th_rot = self.theta-self.get_direction(self.x_g,self.y_g)
            angs = wrapToPi(np.array([ang-th_rot, ang])) 
            # alpha = angs[0]
            # delta = angs[1]

            if th_rot < YAW_PREC:
                debug("Facing correct direction, moving forward")
                V = K1*rho
                om = 0
            else:
                debug("Deviated from direction, fixing yaw")
                V = 0
                om = th_rot
            cmd_x_dot = np.sign(V)*min(V_MAX, np.abs(V))
            cmd_theta_dot = np.sign(om)*min(W_MAX, np.abs(om))
            debug("ctrl x dot {}, theta dot {}".format(cmd_x_dot, cmd_theta_dot))

        else:
            # haven't received a command in a while so stop
            debug("PoseController: No command in a while, commanding zero controls")
            cmd_x_dot = 0
            cmd_theta_dot = 0


        cmd = Twist()
        cmd.linear.x = cmd_x_dot
        cmd.angular.z = cmd_theta_dot
        return cmd

    def get_direction(self,x, y):
        return np.arctan2(y-self.y, x-self.x)

    def get_ctrl_output_fix_yaw(self, theta_target):
        cmd_x_dot = 0
        cmd_theta_dot = 0

        if (rospy.get_rostime().to_sec()-self.cmd_pose_time.to_sec()) < TIMEOUT:
            err_yaw = wrapToPi(theta_target - self.theta)
            if np.fabs(err_yaw) > YAW_PREC:
                debug("yaw error = %f", err_yaw)
                if np.fabs(err_yaw) > 0.5:
                    cmd_theta_dot = YAW_STEP_LARGE if err_yaw > 0 else -YAW_STEP_LARGE
                else:
                    cmd_theta_dot = YAW_STEP_SMALL if err_yaw > 0 else -YAW_STEP_SMALL

        else:
            # haven't received a command in a while so stop
            debug("PoseController: No command in a while, commanding zero controls")
            cmd_x_dot = 0
            cmd_theta_dot = 0
            err_yaw = 0

        cmd = Twist()
        cmd.linear.x = cmd_x_dot
        cmd.angular.z = cmd_theta_dot
        return cmd, err_yaw

    def close_to_goal(self):
        self.update_current_pose()            
        rel_coords = np.array([self.x-self.x_g, self.y-self.y_g])
        rho = linalg.norm(rel_coords) 
        if (rho < DIST_PREC):
            return True 
        else:
            return False

    def get_ctrl_output_idle(self):
        cmd = Twist()
        cmd.linear.x = 0
        cmd.angular.z = 0
        return cmd

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # don't start until we received the first goal 
            if self.x_g == None:
               self.change_state(PCState.IDLE)
               # For some reason, if we try to update_current_goal()
               # before a nav_goal is given we will get an tf error

            # State machine for pose controller
            if self.state == PCState.IDLE:
                ctrl_output = self.get_ctrl_output_idle()

            # elif self.state == PCState.FIX_YAW_INIT:
            #     self.update_current_pose()
            #     ctrl_output, err_yaw = self.get_ctrl_output_fix_yaw(self.get_direction(self.x_g, self.y_g))
            #     if err_yaw < YAW_PREC:
            #         self.change_state(PCState.MOVE_FWD)

            elif self.state == PCState.MOVE_FWD:
                self.update_current_pose()            
                ctrl_output = self.get_ctrl_output_fwd()
                if self.close_to_goal():
                    self.change_state(PCState.FIX_YAW_FINAL)

            elif self.state == PCState.FIX_YAW_FINAL:
                self.update_current_pose()
                ctrl_output, err_yaw = self.get_ctrl_output_fix_yaw(self.theta_g)
                if err_yaw < YAW_PREC:
                    self.change_state(PCState.IDLE)

            self.pub.publish(ctrl_output)
                
            rate.sleep()


if __name__ == '__main__':
    pctrl = PoseController()
    pctrl.run()
