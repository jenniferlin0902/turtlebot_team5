import rospy
import numpy as np

def log(*args):
    rospy.loginfo(" ".join([str(arg) for arg in args]))

def debug(*args):
    rospy.logdebug(" ".join([str(arg) for arg in args]))

def warn(*args):
    rospy.logwarn(" ".join([str(arg) for arg in args]))

def error(*args):
    rospy.logerr(" ".join([str(arg) for arg in args]))

def wrapToPi(a):
    if isinstance(a, list):
        return [(x + np.pi) % (2*np.pi) - np.pi for x in a]
    return (a + np.pi) % (2*np.pi) - np.pi

def wrapTo2Pi(a):
	# wrap to 0-2pi
    if isinstance(a, list):
        return [(x + 2*np.pi) % (2*np.pi) for x in a]
    return (a + 2*np.pi) % (2*np.pi)
