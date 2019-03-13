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
from turtlebot_team5.msg import DetectedObject, ObjectLocationList
from timer import Timer
from utils import log, error, debug

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
        rospy.init_node('turtlebot_supervisor', anonymous=True)

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

        # Current delivery request queue.
        self.delivery_requests = []

        # Detected objects.
        self.obj_coordinates = {}  # "name" --> (x, y)

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
        if cmd == "idle":
            self.set_mode(IdleMode)
        elif cmd == "manual":
            self.set_mode(ManualMode)
        elif cmd == "request":
            self.set_mode(RequestMode)
        else:
            error("Invalid command to state_machine_callback:", cmd)

    def nav_path_callback(self, msg):
        if self.mode != NavMode:
            error("Got to nav_path_callback while in mode:", self.mode)
            return
        self.path = [convert_pose_to_tuple(ps.pose) for ps in msg.poses]  # [(x, y, theta)]
        self.path_index = 0

    def gazebo_callback(self, msg):
        pose = msg.pose[msg.name.index("turtlebot3_burger")]
        twist = msg.twist[msg.name.index("turtlebot3_burger")]
        self.x, self.y, self.theta = convert_pose_to_tuple(pose)

    def rviz_goal_callback(self, msg):
        if self.mode != ManualMode:
            error("Got to rviz_goal_callback while in mode:", self.mode)
            return

        origin_frame = "/map" if mapping else "/odom"
        try:
            nav_pose_origin = self.trans_listener.transformPose(origin_frame, msg)
            self.nav_goal_pose_x, self.nav_goal_pose_y, self.nav_goal_pose_theta = \
                convert_pose_to_tuple(nav_pose_origin.pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def delivery_request_callback(self, msg):
        items = msg.data.lower().strip().split(",")
        if len(items) > 0:
            debug("delivery_request_callback: Got {} items:".format(len(items)), items)
            self.delivery_requests.extend(items)

    def object_location_callback(self, msg):
        debug("object_location_callback: Got {} locations".format(len(msg.locations)))
        for loc in msg.locations:
            self.obj_coordinates[loc.name] = (loc.x, loc.y)

    # ==============================================================================================
    # Robot state query functions.

    def close_to(self, point):
        """Check if the robot is at a pose within some threshold."""
        x, y, theta = point
        return (abs(x - self.x) < POS_EPS and
                abs(y - self.y) < POS_EPS and
                abs(theta - self.theta) < THETA_EPS)

    # ==============================================================================================
    # State machine.

    def set_mode(self, mode):
        """Sets the current mode in thread safe way."""
        self._mode_lock.acquire()
        if self.mode != mode:
            log("State machine: exiting", self.mode.__name__, "--> entering", mode.__name__)
            self.mode.exit(self)
            self.mode = mode
            self.mode.enter(self)
            log("State machine: running", self.mode.__name__)
        else:
            log("State machine: already running", self.mode.__name__)
        self._mode_lock.release()

    def run(self):
        """Loop to execute state machine logic."""
        rate = rospy.Rate(10)  # 10 Hz
        self.mode.enter(self)
        while not rospy.is_shutdown():
            # Get location from mapping if using real robot.
            if not use_gazebo:
                try:
                    origin_frame = "/map" if mapping else "/odom"
                    (translation, rotation) = self.trans_listener.lookupTransform(
                        origin_frame, '/base_footprint', rospy.Time(0)
                    )
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


class ManualMode(Mode):
    """Manually drive the robot around by clicking in Rviz."""
    @staticmethod
    def run(robot):
        if robot.nav_goal_pose_x is not None:
            msg = Pose2D()
            msg.x = robot.nav_goal_pose_x
            msg.y = robot.nav_goal_pose_y
            msg.theta = robot.nav_goal_pose_theta
            robot.step_goal_publisher.publish(msg)


class IdleMode(Mode):
    """Just sit there, not moving."""
    @staticmethod
    def run(robot):
        msg = Twist()
        robot.cmd_vel_publisher.publish(msg)


class RequestMode(Mode):
    """Wait for requests to come in, then fulfill them one by one."""
    @staticmethod
    def enter(robot):
        msg = Twist()
        robot.cmd_vel_publisher.publish(msg)

    @staticmethod
    def run(robot):
        if len(robot.delivery_requests) > 0:
            debug("RequestMode: There are {} delivery requests.".format(len(robot.delivery_requests)))
            curr_request = robot.delivery_requests.pop(0)
            if curr_request not in robot.obj_coordinates:
                error("RequestMode: Invalid request specified:", curr_request)
            else:
                debug("RequestMode: Valid request specified:", curr_request)
                robot.nav_goal_pose_x, robot.nav_goal_pose_y = robot.obj_coordinates[curr_request]
                robot.set_mode(NavMode)


class NavMode(Mode):
    """Navigate to a particular object on the map."""
    @staticmethod
    def enter(robot):
        if robot.path is not None:
            error("Entering NavMode when robot.path is not None")
        if robot.nav_goal_pose_x is None:
            error("Entering NavMode when robot.nav_goal_pose is not set.")

        msg = Pose2D()
        msg.x = robot.nav_goal_pose_x
        msg.y = robot.nav_goal_pose_y
        msg.theta = robot.nav_goal_pose_theta  # Does not matter
        robot.nav_goal_publisher.publish(msg)

    @staticmethod
    def run(robot):
        # Wait until path has been computed
        if robot.path is None:
            debug("NavMode: No path computed yet.")
            return

        # Currently executing step
        curr_step = robot.path[robot.path_index]
        msg = Pose2D()
        msg.x = curr_step[0]
        msg.y = curr_step[1]
        msg.theta = curr_step[2]
        robot.step_goal_publisher.publish(msg)

        if robot.close_to(curr_step):
            # Finished executing step so move on to next.
            debug("NavMode: Finished executing step, so moving to next.")
            robot.path_index += 1
        if robot.path_index >= len(robot.path):
            # Finished executing all steps.
            debug("NavMode: Finished executing all steps.")
            robot.path = None
            robot.path_index = 0
            robot.set_mode(RequestMode)


class CrossMode(Mode):
    @staticmethod
    def run(robot):
        pass


if __name__ == '__main__':
    sup = Supervisor()
    sup.run()
