#!/usr/bin/env python

'''
    Team 5 Supervisor Node
    Runs State Machine

    README:
    - Please do not change state in any helper functions. Only change state inside
      of the state machine (the loop function). Each helper function does one thing only
    - Please put most helper functions in team5_util.py. That is any function
      that is not part of Supervisor, i.e. doesn't require 'self'.
    - Please comment new functions

    - See team5_timers.py for timers.
    - See team5_util.py for misc functions.
'''

'''
    TODO:
    - Implement manual control. Need something like:
        from asl_turtlebot.msg import ManualInput
        assign callback
'''

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from asl_turtlebot.msg import DetectedObject
import tf
import math
from enum import Enum

from team5_util import log, valdiate_value
from team5_timers import Timers


####################### Constants #######################

# Threshold at which we consider the robot at a location
POS_EPS = .1
THETA_EPS = .3

# Minimum distance from a stop sign to obey it
STOP_MIN_DIST = 0.5


####################### Modes #######################

class Mode(Enum):
    '''
        All possible states for state machine.
    '''
    START = 0
    IDLE = 1
    POSE = 2
    STOP = 3
    CROSS = 4
    NAV = 5
    MANUAL = 6


####################### Supervisor #######################

class Supervisor:
    '''
        Runs finite state machine.
        See loop() for implementation.
    '''

    def __init__(self):
        '''
            Set initial state. In Loop(), START state will call self.init()  
        '''
        self.mode = Mode.START



    ####################### Init #######################

    def init(self):
        self.init_flags()
        self.init_positions_variables()
        self.init_timers()
        self.init_global_ros_settings()
        self.init_ros_node()
        self.init_ros_publishers()
        self.init_ros_subscribers()


    def init_flags(self):
        self._last_mode_printed = None
        self._stop_sign_detected = False
        self._is_currently_stopping = False
        self._is_currently_crossing = False

    def init_positions_variables(self):
        self.x = 0
        self.y = 0
        self.theta = 0

    def init_timers(self):
        '''
            See team5_timers.py for more details.
        '''
        self.timers = Timers()

    def init_global_ros_settings():
        # if sim is True/using gazebo, therefore want to subscribe to /gazebo/model_states\
        # otherwise, they will use a TF lookup (hw2+)
        self.use_gazebo = rospy.get_param("sim")

        # how is nav_cmd being decided -- human manually setting it, or rviz
        self.rviz = rospy.get_param("rviz")

        # if using gmapping, you will have a map frame. otherwise it will be odom frame
        self.mapping = rospy.get_param("map")

        log("supervisor settings:\n")
        log("use_gazebo = %s\n" % self.use_gazebo)
        log("rviz = %s\n" % self.rviz)
        log("mapping = %s\n" % self.mapping)

    def init_ros_node(self):
        rospy.init_node('turtlebot_supervisor', anonymous=True)

    def init_ros_publishers(self):
        self.trans_listener = tf.TransformListener()
        # command pose for controller
        self.pose_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
        # command vel (used for idling)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def init_ros_subscribers(self):
        # stop sign detector
        rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_detected_callback)
        # high-level navigation pose
        rospy.Subscriber('/nav_pose', Pose2D, self.nav_pose_callback)
        # if using gazebo, we have access to perfect state
        if self.use_gazebo:
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        # if using rviz, we can subscribe to nav goal click
        if self.rviz:
            rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)
        
        '''
            TODO:
                - Possibly add callback for manual control?
        '''



    ####################### Callbacks #######################

    # TODO: Implement this
    def manual_control_callback(self, msg):
        pass


    def nav_pose_callback(self, msg):
        log("IN NAV_POSE_CALLBACK")
        self.x_g = msg.x
        self.y_g = msg.y
        self.theta_g = msg.theta

        ######### Start My changes #########

        # This was present in the original TA code. I removed this.
        # self.mode = Mode.NAV

        ######### End My Changes #########


    def stop_sign_detected_callback(self, msg):
        '''
            Callback for when the detector has found a stop sign. Note that
            A distance of 0 can mean that the lidar did not pickup the stop sign at all
        '''
        # distance of the stop sign
        dist = msg.distance

        log("IN STOP_SIGN_DETECTED_CALLBACK.")
        log("distance: ", dist)

        # if close enough and in nav mode, stop
        if dist > 0 and dist < STOP_MIN_DIST:
            self.init_stop_sign()

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

    def rviz_goal_callback(self, msg):
        """ callback for a pose goal sent through rviz """
        log("In RVIZ_GOAL_CALLBACK.")
        origin_frame = "/map" if self.mapping else "/odom"
        log("rviz command received!")
        try:
            nav_pose_origin = self.trans_listener.transformPose(origin_frame, msg)
            self.x_g = nav_pose_origin.pose.position.x
            self.y_g = nav_pose_origin.pose.position.y
            quaternion = (
                    nav_pose_origin.pose.orientation.x,
                    nav_pose_origin.pose.orientation.y,
                    nav_pose_origin.pose.orientation.z,
                    nav_pose_origin.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.theta_g = euler[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        ######### Start My changes #########

        # This was present in the original TA code. I removed this.
        # self.mode = Mode.NAV

        ######### End My Changes #########





    ####################### Movement #######################

    def go_to_pose(self):
        '''
            Send the current desired pose to the pose controller.
        '''
        pose_g_msg = Pose2D()
        try:
            pose_g_msg.x = self.x_g
            pose_g_msg.y = self.y_g
            pose_g_msg.theta = self.theta_g
        except:
            log("WARNING: x_g, y_g, theta_g not set.")

        self.pose_goal_publisher.publish(pose_g_msg)

    def idle(self):
        '''
            Send zero velocity to stay put.
        '''
        vel_g_msg = Twist()
        self.cmd_vel_publisher.publish(vel_g_msg)



    ####################### Final Goal Detection #######################

    def close_to(self, x, y, theta):
        '''
            Check if the robot is at a pose within some threshold
        '''
        return (abs(x - self.x) < POS_EPS and
                abs(y - self.y) < POS_EPS and
                abs(theta - self.theta) < THETA_EPS)

    def goal_reached(self):
        '''
            If x_g, y_g, theta_g not set, returns False
        '''
        try:
            return self.close_to(self.x_g, self.y_g, self.theta_g)
        except:
            return False



    ####################### Stop Sign Detection #######################

    def stop_sign_detected(self):
        '''
            If stop sign was detected, reset value
        '''
        if self._stop_sign_detected:
            print("FSM DETECTED STOP SIGN.")
            self.clear_stop_sign_detected()
            return True
        else:
            return False

    def init_stop_sign(self):
        '''
            Callback function. Do not use in State Machine.
        '''
        print("STOP SIGN DETECTED.")
        self._stop_sign_detected = True

    def clear_stop_sign_detected(self):
        '''
            Setter for _stop_sign_detected. DO use in State Machine.
        '''
        self._stop_sign_detected = False



    ####################### Stopping #######################

    def is_currently_stopping(self):
        '''
            Check if currently stopping.
            If value has not been set yet, set it.
        '''
        return self._is_currently_stopping

    def start_stopping(self):
        '''
            Set flag. Begin timer.
        '''
        if self.is_currently_stopping():
            raise Exception("Called start_stopping() while already stopping.")
        self._is_currently_stopping = True
        self.timers.start_timer(Timers.STOP_TIMER)

    def end_stopping(self):
        '''
            Set _is_currently_stopping flag to False.
            Clear stop_sign_detected flag.
        '''
        if not self.is_currently_stopping():
            raise Exception("Called end_stopping() while not stopping.")
        self._is_currently_stopping = False
        self.clear_stop_sign_detected()

    def is_done_stopping(self):
        '''
            Check if stopping timer is finished.
        '''
        return self.timers.timer_finished(Timers.STOP_TIMER)



    ####################### Crossing #######################

    def is_currently_crossing(self):
        '''
            Check if currently crossing.
            If value has not been set yet, set it.
        '''
        return self._is_currently_crossing

    def start_crossing(self):
        '''
            Set flag. Begin timer.
        '''
        if self.is_currently_crossing():
            raise Exception("Called start_crossing() while already crossing.")
        self._is_currently_crossing = True
        self.timers.start_timer(Timers.CROSS_TIMER)

    def end_crossing(self):
        '''
            Set _is_currently_crossing flag to False.
        '''
        if not self.is_currently_crossing():
            raise Exception("Called end_crossing() while not crossing.")
        self._is_currently_crossing = False

    def is_done_crossing(self):
        '''
            Check if crossing timer is finished.
        '''
        return self.timers.timer_finished(Timers.CROSS_TIMER)


    ####################### Logging #######################

    def log_state(state):
        '''
            Log state if it has changed.
            log() is in team5_util.py.
        '''
        if not(self._last_mode_printed == state):
            msg = "Current Mode: %s" % (state)
            log(msg)
            self._last_mode_printed = state






    ####################### State Machine #######################


    def loop(self):
        '''
            - PLEASE DO NOT CHANGE STATE OUTSIDE OF THIS FUNCTION. Don't change state in helper functions. 
              Each helper function does one thing only

            - There should be no "dead zones" for any state. Each state MUST handle every case. This means
              there should be no situation where you do nothing. Either update the state, or call
              an action (helper function).
              Said another way, if you ever write an if-statement to check a condition, there should be a
              corresponding else statement

            The main loop of the robot. At each iteration, depending on its
            mode (i.e. the finite state machine's state), if takes appropriate
            actions. This function shouldn't return anything
        '''

        #################################################################################
        # Do not change this for hw2 -- this won't affect your FSM since you are using gazebo
        if self.mode != Mode.START and not self.use_gazebo:
            try:
                origin_frame = "/map" if self.mapping else "/odom"
                (translation,rotation) = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
                self.x = translation[0]
                self.y = translation[1]
                euler = tf.transformations.euler_from_quaternion(rotation)
                self.theta = euler[2]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
        #################################################################################



        # '''''''''''''''''''''''''''''''''''''''''''''''''
        #                    STATE MACHINE
        # '''''''''''''''''''''''''''''''''''''''''''''''''

        state = self.mode
        log_state(state)


        # '''
        #     START
        # '''
        if state == Mode.START:                 # Go to NAV
            self.init()
            state = Mode.NAV
        
        # '''
        #     IDLE
        # '''
        elif state == Mode.IDLE:                # Wait forever
            state = self.idle()

        # '''
        #    NAV
        # '''
        elif state == Mode.NAV:                 
            if self.goal_reached():             # If goal reached, idle
                state = Mode.IDLE
            elif self.stop_sign_detected():     # If stop sign detected, go to mode.STOP
                state = Mode.STOP
            else:                               # If normal, continue driving
                self.go_to_pose()

        # '''
        #    STOP
        # '''
        elif state == Mode.STOP:
            if self.is_currently_stopping():    # If currently stopping
                if self.is_done_stopping():     # If done, end then go to CROSS
                    self.end_stopping()
                    state = Mode.CROSS
                else:                           # If not done, continue idling
                    self.idle()
            else:                               # If not currently stopping, start
                self.start_stopping()

        # '''
        #    CROSS
        # '''
        elif state == Mode.CROSS:
            if self.is_currently_crossing():    # If currently crossing
                if self.is_done_crossing():     # If done, end then go to POSE
                    self.end_crossing()
                    state = Mode.NAV
                elif self.goal_reached():       # If goal reached, idle
                    self.end_crossing()
                    state = Mode.IDLE
                else:                           # If not done, continue driving
                    self.go_to_pose()

            else:                               # If not currently crossing, start
                self.start_crossing()


        # '''
        #    MANUAL
        # '''
        elif state == Mode.MANUAL:
            raise Exception("In state Mode.MANUAL. Currenlty not implemented.")


        # '''
        #    ERROR
        # '''
        elif state == Mode.POSE:
            raise Exception("In state Mode.POSE. Shouldn't be here.")
        else:
            raise Exception("This mode not supported. Mode: " + str(state))


        # Assign new state
        self.mode = state



    ####################### Run #######################

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()




####################### Main #######################     

if __name__ == '__main__':
    sup = Supervisor()
    sup.run()