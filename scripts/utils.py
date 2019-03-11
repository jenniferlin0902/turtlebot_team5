import rospy
import numpy as np

def log(*args):
    rospy.loginfo(" ".join([str(arg) for arg in args]))

def error(*args):
    rospy.logerror(" ".join([str(arg) for arg in args]))

def wrapToPi(a):
    if isinstance(a, list):
        return [(x + np.pi) % (2*np.pi) - np.pi for x in a]
    return (a + np.pi) % (2*np.pi) - np.pi
