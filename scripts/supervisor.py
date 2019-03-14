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
from turtlebot_team5.msg import DetectedObject, ObjectLocationList, PoseControl
from timer import Timer
from utils import log, error, debug, warn
import numpy as np

# ==================================================================================================
# Constants.

# Threshold (m) for robot being "close enough" to a given position/theta for it to be considered
# there.
POS_EPS = .2
THETA_EPS = 1.5*np.pi*(10/180.0)

# Minimum distance (m) from a stop sign at which to obey it by initiating stopping behavior.
STOP_MIN_DIST = 0.5

# Time (s) taken to cross an intersection.
CROSSING_TIME = 3

# Time (s) to stop at a stop sign.
STOP_TIME = 3

# Timeout for nav mode
NAV_TIMEOUT = 10

# ==================================================================================================
# Parameters.

# Whether running in simulation gazebo, therefore want to subscribe to /gazebo/model_states.
# Otherwise, they will use a TF lookup.
use_gazebo = rospy.get_param("sim")

# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")

log("Supervisor settings:")
log("use_gazebo:", use_gazebo)
log("mapping:", mapping)


# ==================================================================================================
# Helper functions.

def convert_pose_to_tuple(pose):
    x = pose.position.x
    y = pose.position.y
    quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    )
    euler = tf.transformations.euler_from_quaternion(quaternion)
    theta = euler[2]
    return x, y, theta


# ==================================================================================================
# Supervisor node.

class Supervisor:

    def __init__(self):
        """Initialized ROS node with the state machine in the `ManualMode` mode."""
        rospy.init_node('turtlebot_supervisor', log_level=rospy.DEBUG, anonymous=True)

        # ==========================================================================================
        # Robot state.

        # Current mode.
        self.mode = ManualMode
        self._mode_lock = threading.Lock()

        # Current robot pose.
        self.x = 0
        self.y = 0
        self.theta = 0

        # Current robot destination.
        self.nav_goal_pose_x = None
        self.nav_goal_pose_y = None
        self.nav_goal_pose_theta = None

        # Navigation path containing steps towards current destination.
        self.path = None
        self.path_index = 0

        # Current computed step commands.
        self.step_cmd_vel = Twist()
        self.step_is_done = True

        # Last time stopped at stop sign, do no stopping again for a period.
        self.last_time_stopped = rospy.get_rostime()

        # Current delivery request queue.
        self.delivery_requests = []

        # Detected objects.
        self.obj_coordinates = {}  # "name" --> (x, y)

        # Keep track of time that planning is happening.
        self.nav_mode_enter_time = -1.0

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

        # Commanded food items to pick up.
        rospy.Subscriber('/delivery_request', String, self.delivery_request_callback)

        # Detected object locations.
        rospy.Subscriber('/object_location', ObjectLocationList, self.object_location_callback)

        # Control velocity and state from pose controller.
        rospy.Subscriber('/step_cmd', PoseControl, self.step_cmd_callback)

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
        cmd = msg.data.lower().strip()
        debug("Supervisor: state_machine_callback: Got cmd '{}'".format(cmd))
        if cmd == "idle":
            self.set_mode(IdleMode)
        elif cmd == "manual":
            self.set_mode(ManualMode)
        elif cmd == "request":
            self.set_mode(RequestMode)
        elif cmd == "nav":
            self.set_mode(NavMode)
        elif cmd == "home":
            self.set_mode(HomeMode)
        else:
            warn("Invalid command to state_machine_callback:", cmd)

    def nav_path_callback(self, msg):
        debug("Supervisor: nav_path_callback: Got nav_path of length {}.".format(len(msg.poses)))
        if self.mode != NavMode:
            error("Supervisor: nav_path_callback: Path received in mode:", self.mode.__name__)
            return

        self.path = [convert_pose_to_tuple(ps.pose) for ps in msg.poses]  # [(x, y, theta)]
        self.path_index = 0

    def gazebo_callback(self, msg):
        pose = msg.pose[msg.name.index("turtlebot3_burger")]
        self.x, self.y, self.theta = convert_pose_to_tuple(pose)

    def rviz_goal_callback(self, msg):
        debug("Supervisor: rviz_goal_callback: Got nav_pose from rviz.")
        origin_frame = "/map" if mapping else "/odom"
        try:
            nav_pose_origin = self.trans_listener.transformPose(origin_frame, msg)
            self.nav_goal_pose_x, self.nav_goal_pose_y, self.nav_goal_pose_theta = \
                convert_pose_to_tuple(nav_pose_origin.pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def delivery_request_callback(self, msg):
        items = msg.data.lower().strip().split(",")
        debug("Supervisor: delivery_request_callback: Got {} items:".format(len(items)), items)
        if len(items) > 0:
            self.delivery_requests.extend(items)

    def object_location_callback(self, msg):
        debug("Supervisor: object_location_callback: Got {} locations.".format(len(msg.locations)))
        for loc in msg.locations:
            self.obj_coordinates[loc.name] = (loc.x, loc.y)

    def step_cmd_callback(self, msg):
        if msg.is_done != self.step_is_done:
            debug("Supervisor: step_cmd_callback: Got command and is_done changed to", msg.is_done)
        self.step_cmd_vel = msg.cmd_vel
        self.step_is_done = msg.is_done

    # ==============================================================================================
    # Robot state helper functions.

    def is_close_to(self, point):
        """Check if the robot is at a pose within some threshold."""
        x, y, theta = point
        return (abs(x - self.x) < POS_EPS and
                abs(y - self.y) < POS_EPS and
                abs(theta - self.theta) < THETA_EPS)

    def is_detecting_stop_sign(self):
        # TODO: Check if any stop sign is within active area for triggering.
        return False

    def stop_moving(self):
        msg = Twist()
        self.cmd_vel_publisher.publish(msg)

    # ==============================================================================================
    # State machine.

    def set_mode(self, mode):
        """Sets the current mode in thread safe way."""
        # Must protect mode with a mutex since this method may be called in callbacks. Also must
        # not throw exceptions but rather catch them, otherwise will kill the thread that called
        # this method.
        self._mode_lock.acquire()
        if self.mode != mode:
            log("StateMachine: exiting", self.mode.__name__, "--> entering", mode.__name__)
            try:
                self.mode.exit(self)
            except Exception, e:
                error("StateMachine: Got exception from exit() of {}:\n".format(self.mode.__name_, str(e)))
            self.mode = mode
            try:
                self.mode.enter(self)
            except Exception, e:
                error("StateMachine: Got exception from enter() of {}:\n".format(self.mode.__name_, str(e)))
            log("StateMachine: running", self.mode.__name__)
        else:
            log("StateMachine: already running", self.mode.__name__)
        self._mode_lock.release()

    def run(self):
        """Loop to execute state machine logic."""
        rate = rospy.Rate(10)  # 10 Hz
        log("StateMachine: starting in", self.mode.__name__)
        self.mode.enter(self)
        while not rospy.is_shutdown():

            if use_gazebo:
                # The gazebo_callback already populates self.x/y/theta.
                pass
            else:
                # Get location from mapping if using real robot.
                try:
                    origin_frame = "/map" if mapping else "/odom"
                    (translation, rotation) = self.trans_listener.lookupTransform(
                        origin_frame, '/base_footprint', rospy.Time(0)
                    )
                    self.x = translation[0]
                    self.y = translation[1]
                    euler = tf.transformations.euler_from_quaternion(rotation)
                    self.theta = euler[2]
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    debug("Supervisor: got tf exception: ", e)
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


class ManualMode(Mode):
    """Manually drive the robot around by clicking in Rviz."""
    @staticmethod
    def enter(robot):
        debug("Entering manual mode")
        robot.nav_goal_pose_x = None
        robot.nav_goal_pose_y = None
        robot.nav_goal_pose_theta = None
        robot.stop_moving()

    @staticmethod
    def run(robot):
        if robot.nav_goal_pose_x is not None:
            msg = Pose2D()
            msg.x = robot.nav_goal_pose_x
            msg.y = robot.nav_goal_pose_y
            msg.theta = robot.nav_goal_pose_theta
            robot.step_goal_publisher.publish(msg)

        # Move according to pose controller.
        robot.cmd_vel_publisher.publish(robot.step_cmd_vel)


class IdleMode(Mode):
    """Just sit there, not moving."""
    @staticmethod
    def run(robot):
        robot.stop_moving()


class RequestMode(Mode):
    """Wait for requests to come in, then fulfill them one by one."""
    @staticmethod
    def enter(robot):
        debug("RequestMode: Stopping movement until there are delivery requests.")
        robot.stop_moving()

    @staticmethod
    def run(robot):
        if len(robot.delivery_requests) > 0:
            debug("RequestMode: There are {} delivery requests.".format(len(robot.delivery_requests)))
            curr_request = robot.delivery_requests.pop(0)
            if curr_request not in robot.obj_coordinates:
                warn("RequestMode: Invalid request specified:", curr_request)
            else:
                debug("RequestMode: Valid request specified:", curr_request)
                x, y = robot.obj_coordinates[curr_request]
                if np.isnan(x) or np.isnan(y):
                    warn(curr_request, " has nan in coordinates, skipping.")
                    return

                robot.nav_goal_pose_x = x
                robot.nav_goal_pose_y = y
                robot.set_mode(NavMode)


class NavMode(Mode):
    """Navigate to a particular object on the map. Each time we enter this state, we will re-compute
    the path to the nav_goal_pose."""
    @staticmethod
    def enter(robot):
        if robot.nav_goal_pose_x is None:
            error("NavMode: Entering when robot.nav_goal_pose is not set.")

        msg = Pose2D()
        msg.x = robot.nav_goal_pose_x
        msg.y = robot.nav_goal_pose_y
        msg.theta = robot.nav_goal_pose_theta  # Does not matter
        robot.nav_mode_enter_time = rospy.get_rostime().to_sec()
        robot.nav_goal_publisher.publish(msg)

    @staticmethod
    def run(robot):
        # Wait until path has been computed
        if robot.path is None:
            debug("NavMode: No path computed yet, waiting for navigator.")
            return

        # Publish the currently executing step.
        debug("NavMode: Path has {} steps, currently on step {}.".format(len(robot.path), robot.path_index))
        # curr_step = robot.path[robot.path_index]
        # msg = Pose2D()
        # msg.x, msg.y, msg.theta = curr_step
        # robot.step_goal_publisher.publish(msg)

        if rospy.get_rostime().to_sec() - robot.nav_mode_enter_time < NAV_TIMEOUT:
            log("NavMode: Planning path has timed out.")
            robot.set_mode(RequestMode)
            return

        if robot.step_is_done:
            if robot.path_index >= len(robot.path):
                # Finished executing all steps.
                debug("NavMode: Finished executing all steps.")
                robot.path = None
                robot.path_index = 0
                robot.set_mode(RequestMode)
                return
            
            # Finished executing step so move on to next.
            debug("NavMode: Finished executing step, so moving to next.")
            robot.step_is_done = False

            curr_step = robot.path[robot.path_index]
            msg = Pose2D()
            msg.x, msg.y, msg.theta = curr_step
            robot.step_goal_publisher.publish(msg)

            robot.path_index += 1
            
        elif robot.is_detecting_stop_sign() and robot.last_time_stopped > CROSSING_TIME:
            # Detecing new stop sign and window for ignoring has elapsed.
            debug("NavMode: Detecting new stop sign.")
            robot.set_mode(StopMode)
        else:
            # Keep moving according to pose controller.
            robot.cmd_vel_publisher.publish(robot.step_cmd_vel)


class StopMode(Mode):
    """Stopping at stop sign for set amount of time. After stopping, will continue on navigation as
    usual without reacting to the same stop sign."""
    stop_timer = Timer(STOP_TIME)

    @staticmethod
    def enter(robot):
        debug("StopMode: Stopping at stop sign.")
        robot.stop_moving()
        StopMode.stop_timer.start()

    @staticmethod
    def run(robot):
        if StopMode.stop_timer.is_finished():
            debug("StopMode: Moving on from stop sign.")
            robot.last_time_stopped = rospy.get_rostime()
            robot.set_mode(NavMode)


class HomeMode(Mode):
    """Navigate to starting position and enter request mode."""
    @staticmethod
    def enter(robot):
        debug("HomeMode: Going to starting position.")
        robot.stop_moving()
        robot.nav_goal_pose_x = 0
        robot.nav_goal_pose_y = 0

    @staticmethod
    def run(robot):
        robot.set_mode(NavMode)


if __name__ == '__main__':
    sup = Supervisor()
    sup.run()
