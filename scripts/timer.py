import rospy


class Timer(object):

    def __init__(self, duration):
        self.start_time = None
        self.duration = duration  # Elapsed time (s).
        return self

    def start(self):
        self.start_time = rospy.get_rostime()

    def reset(self):
        self.start_time = None

    def is_started(self):
        return self.start_time is not None

    def is_finished(self):
        return rospy.get_rostime() - self.start_time > rospy.Duration.from_sec(self.duration)