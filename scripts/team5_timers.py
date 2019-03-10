'''
	Team5 Timers

	How to use:
		- Add a global constant for the duration. See CROSSING_TIME
		- Add the timer name to the class. Set the value to the constant. See STOP_TIMER = STOP_TIME
		- To use in state-machine, use self.timers.start_timer(Timers.TIMER_NAME). For example,
			in supervisor.py, find these lines:
			- self.timers.start_timer(Timers.STOP_TIMER)
			- return self.timers.timer_finished(Timers.STOP_TIMER)
'''

import rospy
from enum import Enum


####################### Durations #######################

# Time taken to cross an intersection
CROSSING_TIME = 3

# Time to stop at a stop sign
STOP_TIME = 3


####################### Flag #######################

# Flag to see if timer is set or not.
TIMER_NOT_SET = -1


####################### Timer #######################

class Timers(Enum):
    STOP_TIMER = STOP_TIME
    CROSS_TIMER = CROSSING_TIME 

class TimersObject(Enum):


    def __init__(self):
        '''
            Initialize timers.
        '''

        self.start_times = { timer.key : TIMER_NOT_SET for timer in Timers }
        self.durations = { timer.key : timer.value for timer in Timers }

    def start_timer(self, timer_name):
        '''
            Start a timer.
            Must first call reset_timer before starting a timer.
        '''
        if self.start_times[timer_name] != TIMER_NOT_SET:
            raise Exception("Clobbering previously set timer start time.")

        self.start_times[timer_name] = rospy.get_rostime()


    def timer_finished(self, timer_name):
        '''
            Check if timer is finished.
            If yes, resets the timer.
        '''
        finished = rospy.get_rostime() - self.start_times[timer_name] > rospy.Duration.from_sec(self.durations[mode])

        # If timer expired, reset it.
        if finished:
            self.reset_timer(timer_name)
        return finished


    def reset_timer(self, timer_name):
        '''
            Set star
            Cannot reset a timer twice.
        '''
        error = False
        try:
            if self.start_times[timer_name] == TIMER_NOT_SET:
                error = True
        except:
            pass
        if error:
            raise Exception("Reseting timer twice.")

        self.start_times[timer_name] = TIMER_NOT_SET



	####################### Other #######################
    '''
		Extra methods.
		Might be useful later.
	'''

    def reset_all_timers(self):
    	'''
			Set the start_time for all timers to TIMER_NOT_SET
    	'''
    	self.start_times = { timer.key : TIMER_NOT_SET for timer in self }

  	def set_timer_duration(self, timer_name, duration):
  		'''
			Set new duration for timer.
			Cannot set new duration for a timer that is currently running.
  		'''
  		if self.start_times[timer_name] != TIMER_NOT_SET:
  			raise Exception("Error: Attempting to change timer duration while timer is running.")
  		
  		self.durations[timer_name] = duration

  	def reset_all_timer_durations(self):
  		'''
			Resets all timer durations to their original values.
			Will print warning if 
  		'''
    	for time in self.start_times:
    		if time != TIMER_NOT_SET:
    			log("WARNING: You just reset a timer duration while the timer is running.")

    	self.__init__()