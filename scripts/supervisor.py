#!/usr/bin/env python
# This ROS node runs the state machine for the robot and stores the global state needed to
# coordinate the different nodes.

import rospy
import tf
import threading
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from asl_turtlebot.msg import DetectedObject
from timer import Timer


# ==================================================================================================
# Constants.

# Threshold (m) for robot being "close enough" to a given position/theta for it to be considered
# there.
POS_EPS = .1
THETA_EPS = .3

# Minimum distance (m) from a stop sign at which to obey it by initiating stopping behavior.
STOP_MIN_DIST = 0.5

# Time (s) taken to cross an intersection.
CROSSING_TIME = 3

# Time (s) to stop at a stop sign.
STOP_TIME = 3

# ==================================================================================================
# Helper functions.

def log(*args):
    rospy.loginfo(" ".join([str(arg) for arg in args]))

def error(*args):
    rospy.logerror(" ".join([str(arg) for arg in args]))

# ==================================================================================================
# Parameters.

# Whether running in simulation gazebo, therefore want to subscribe to /gazebo/model_states\
# otherwise, they will use a TF lookup (hw2+)
use_gazebo = rospy.get_param("sim")

# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")

log("supervisor settings:")
log("use_gazebo:", use_gazebo)
log("mapping:", mapping)

# ==================================================================================================
# Supervisor node.

class Supervisor:

    def __init__(self):
        """Initialized ROS node with the state machine in the `Mode.START` mode."""
        rospy.init_node('turtlebot_supervisor', anonymous=True)

        # ==========================================================================================
        # Robot state.

        # Current mode.
        self.mode = Mode.MANUAL
        self._mode_lock = threading.Lock()

        # Current robot pose.
        self.x = 0
        self.y = 0
        self.theta = 0

        # Robot final destination.
        self.nav_goal_pose_x = None
        self.nav_goal_pose_y = None
        self.nav_goal_pose_theta = None

        # Navigation path containing steps towards destination.
        self.path = None
        self.path_index = 0

        # Detected objects.
        self.food_coordinates = {}  # "name" --> (x, y)

        # Logging helpers.
        self._last_mode_printed = None

        # ==========================================================================================
        # Subscribers.

        # TF frames with respect to different components.
        self.trans_listener = tf.TransformListener()

        # Commands to manually change mode of state machine.
        rospy.Subscriber('/state_machine', String, self.state_machine_callback)

        # Path in occupancy grid from navigator's motion planning.
        rospy.Subscriber('/nav_path', Path, self.nav_path_callback)

        # Rviz pose setting through click.
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)

        # Detected stop sign.
        # rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_detected_callback)

        # ==========================================================================================
        # Publishers.

        # Robot final destination (for Navigator node).
        self.nav_goal_publisher = rospy.Publisher('/nav_goal_pose', Pose2D, queue_size=10)

        # Robot next step on way to destination (for PoseController node).
        self.step_goal_publisher = rospy.Publisher('/step_goal_pose', Pose2D, queue_size=10)

        # Command velocity (used to make robot idle).
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # ==============================================================================================
    # Subscriber callbacks.

    def state_machine_callback(self, msg):
        cmd = msg.data.lower()
        if cmd == "idle":
            self.mode = IdleMode
        elif cmd == "manual":
            self.mode = ManualMode
        elif cmd.startswith("nav"):
            food_name = cmd.split(":")[1]
            if food_name not in self.food_coordinates:
                error("Invalid food specified:", food_name)
            else:
                target_coordinates = self.food_coordinates[food_name]
                self.nav_goal_pose_x = target_coordinates[0]
                self.nav_goal_pose_y = target_coordinates[1]
                self.mode = NavMode
        else:
            error("Invalid command to state_machine_callback:", cmd)

    def nav_path_callback(self, msg):
        if self.mode != NavMode:
            error("Got to nav_path_callback while in mode:", self.mode)
            return

        self.path = msg
        self.path_index = 0

    # def stop_sign_detected_callback(self, msg):
    #     '''
    #         Callback for when the detector has found a stop sign. Note that
    #         A distance of 0 can mean that the lidar did not pickup the stop sign at all
    #     '''
    #     # distance of the stop sign
    #     dist = msg.distance
    #
    #     log("IN STOP_SIGN_DETECTED_CALLBACK.")
    #     log("distance: ", dist)
    #
    #     # if close enough and in nav mode, stop
    #     if dist > 0 and dist < STOP_MIN_DIST:
    #         self.init_stop_sign()

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
        if self.mode != ManualMode:
            error("Got to rviz_goal_callback while in mode:", self.mode)
            return

        origin_frame = "/map" if self.mapping else "/odom"
        try:
            nav_pose_origin = self.trans_listener.transformPose(origin_frame, msg)
            self.nav_goal_pose_x = nav_pose_origin.pose.position.x
            self.nav_goal_pose_y = nav_pose_origin.pose.position.y
            quaternion = (
                    nav_pose_origin.pose.orientation.x,
                    nav_pose_origin.pose.orientation.y,
                    nav_pose_origin.pose.orientation.z,
                    nav_pose_origin.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.nav_goal_pose_theta = euler[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    # ==============================================================================================
    # Robot state query functions.

    def close_to(self, x, y, theta):
        """Check if the robot is at a pose within some threshold."""
        return (abs(x - self.x) < POS_EPS and
                abs(y - self.y) < POS_EPS and
                abs(theta - self.theta) < THETA_EPS)

    def goal_reached(self):
        """If nav_goal_pose_x, nav_goal_pose_y, nav_goal_pose_theta not set, returns False."""
        if self.nav_goal_pose_x is None: return True
        return self.close_to(self.nav_goal_pose_x, self.nav_goal_pose_y, self.nav_goal_pose_theta)

    # ==============================================================================================
    # State machine.

    def log_mode(self, mode):
        """Log current mode if it has changed."""
        if self._last_mode_printed != mode:
            log("Current mode:", mode)
            self._last_mode_printed = mode

    def set_mode(self, mode):
        """Sets the current mode in thread safe way."""
        self._mode_lock.acquire()
        self.mode = mode
        self._mode_lock.release()

    def run(self):
        """Loop to execute state machine logic."""
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():

            # Get location from mapping if using real robot.
            if not self.use_gazebo:
                try:
                    origin_frame = "/map" if self.mapping else "/odom"
                    (translation, rotation) = self.trans_listener.lookupTransform(origin_frame,
                                                                                  '/base_footprint',
                                                                                  rospy.Time(0))
                    self.x = translation[0]
                    self.y = translation[1]
                    euler = tf.transformations.euler_from_quaternion(rotation)
                    self.theta = euler[2]
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    pass

            self.mode.run(self)
            rate.sleep()


# ==================================================================================================
# Modes for state machine.
# ------------------------
# The class design follows the State Pattern: https://en.wikipedia.org/wiki/State_pattern


class Mode(object):
    @staticmethod
    def enter(robot):
        pass

    @staticmethod
    def run(robot):
        raise NotImplementedError

    @staticmethod
    def exit(robot):
        pass

    def __str__(self):
        return self.__name__


class ManualMode(Mode):
    @staticmethod
    def run(robot):
        if robot.x is not None:
            msg = Pose2D()
            msg.x = robot.nav_goal_pose_x
            msg.y = robot.nav_goal_pose_y
            msg.theta = robot.nav_goal_pose_theta
            robot.step_goal_publisher.publish(msg)


class IdleMode(Mode):
    @staticmethod
    def run(robot):
        msg = Twist()
        robot.cmd_vel_publisher.publish(msg)



class NavMode(Mode):
    @staticmethod
    def run(robot):
        if not robot.goal_reached():
            next_goal = robot.path[robot.path_index]
            robot.path_index += 1

            msg = Pose2D()
            msg.x = next_goal[0]
            msg.y = next_goal[1]
            msg.theta = next_goal[2]
            robot.step_goal_publisher.publish(msg)


class StopMode(Mode):
    timer = None

    @staticmethod
    def enter(robot):
        pass

    @staticmethod
    def run(robot):
        pass


class CrossMode(Mode):
    @staticmethod
    def run(robot):
        pass


if __name__ == '__main__':
    sup = Supervisor()
    sup.run()














# if self.goal_reached():             # If goal reached, idle
# state = Mode.IDLE
# elif self.stop_sign_detected():     # If stop sign detected, go to mode.STOP
# state = Mode.STOP
# else:                               # If normal, continue driving
# self.go_to_pose()

# # '''
# #    NAV
# # '''
# elif state == Mode.NAV:
#     if self.goal_reached():             # If goal reached, idle
#         state = Mode.IDLE
#     elif self.stop_sign_detected():     # If stop sign detected, go to mode.STOP
#         state = Mode.STOP
#     else:                               # If normal, continue driving
#         self.go_to_pose()

# '''
#    STOP
# '''
# elif state == Mode.STOP:
#     if self.is_currently_stopping():    # If currently stopping
#         if self.is_done_stopping():     # If done, end then go to CROSS
#             self.end_stopping()
#             state = Mode.CROSS
#         else:                           # If not done, continue idling
#             self.idle()
#     else:                               # If not currently stopping, start
#         self.start_stopping()

# '''
#    CROSS
# '''
# elif state == Mode.CROSS:
#     if self.is_currently_crossing():    # If currently crossing
#         if self.is_done_crossing():     # If done, end then go to POSE
#             self.end_crossing()
#             state = Mode.NAV
#         elif self.goal_reached():       # If goal reached, idle
#             self.end_crossing()
#             state = Mode.IDLE
#         else:                           # If not done, continue driving
#             self.go_to_pose()

#     else:                               # If not currently crossing, start
#         self.start_crossing

# ####################### Stopping #######################
    #
    # def is_currently_stopping(self):
    #     '''
    #         Check if currently stopping.
    #         If value has not been set yet, set it.
    #     '''
    #     return self._is_currently_stopping
    #
    # def start_stopping(self):
    #     '''
    #         Set flag. Begin timer.
    #     '''
    #     if self.is_currently_stopping():
    #         raise Exception("Called start_stopping() while already stopping.")
    #     self._is_currently_stopping = True
    #     self.timers.start_timer(Timers.STOP_TIMER)
    #
    # def end_stopping(self):
    #     '''
    #         Set _is_currently_stopping flag to False.
    #         Clear stop_sign_detected flag.
    #     '''
    #     if not self.is_currently_stopping():
    #         raise Exception("Called end_stopping() while not stopping.")
    #     self._is_currently_stopping = False
    #     self.clear_stop_sign_detected()
    #
    # def is_done_stopping(self):
    #     '''
    #         Check if stopping timer is finished.
    #     '''
    #     return self.timers.timer_finished(Timers.STOP_TIMER)



    # ####################### Crossing #######################
    #
    # def is_currently_crossing(self):
    #     '''
    #         Check if currently crossing.
    #         If value has not been set yet, set it.
    #     '''
    #     return self._is_currently_crossing
    #
    # def start_crossing(self):
    #     '''
    #         Set flag. Begin timer.
    #     '''
    #     if self.is_currently_crossing():
    #         raise Exception("Called start_crossing() while already crossing.")
    #     self._is_currently_crossing = True
    #     self.timers.start_timer(Timers.CROSS_TIMER)
    #
    # def end_crossing(self):
    #     '''
    #         Set _is_currently_crossing flag to False.
    #     '''
    #     if not self.is_currently_crossing():
    #         raise Exception("Called end_crossing() while not crossing.")
    #     self._is_currently_crossing = False
    #
    # def is_done_crossing(self):
    #     '''
    #         Check if crossing timer is finished.
    #     '''
    #     return self.timers.timer_finished(Timers.CROSS_TIMER)
    #
    #
    # ####################### Stop Sign Detection #######################
    #
    # def stop_sign_detected(self):
    #     '''
    #         If stop sign was detected, reset value
    #     '''
    #     if self._stop_sign_detected:
    #         print("FSM DETECTED STOP SIGN.")
    #         self.clear_stop_sign_detected()
    #         return True
    #     else:
    #         return False
    #
    #
    # def init_stop_sign(self):
    #     '''
    #         Callback function. Do not use in State Machine.
    #     '''
    #     print("STOP SIGN DETECTED.")
    #     self._stop_sign_detected = True
    #
    #
    # def clear_stop_sign_detected(self):
    #     '''
    #         Setter for _stop_sign_detected. DO use in State Machine.
    #     '''
    #     self._stop_sign_detected = False
    #
    #
    #

# def get_object_location(self, msg):
#         '''
#             Get's location from ObjectDetected object.
#         '''
#         if type(msg) != ObjectDetected:
#             raise Exception("Error: Invalid msg type passed into get_object_locaiton(). Requires ObjectedDetected.")
#
#         dist = msg.distance
#         # TODO: FINISH THIS
