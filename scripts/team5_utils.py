'''
	Team 5 misc functions
'''

import rospy

def log(message, *args):
    '''
        Log message in rospy and print message to terminal.
    '''
    for arg in args:
    	message += " " + str(arg)
    rospy.loginfo(message)
    print message



def validate_value(value, value_name, allowed_values):
    '''
        Throw an exception if value is not in allowed_values.
        Use this to cleanly check inputs.
        Example usage:
            valdiate_value(mode, "mode", ["cross", "stop"])

            ^ This throws an exception if the variable mode is not equal to "cross" or "stop"
    '''
    if value not in allowed_values:
        error_message = '''
        Error: Invalid value of ''' + value_name + " used (" + str(value) + ''').
        Allowed values are: \n'''
        for allowed_value in allowed_values:
            error_message += str(allowed_value) + "\n"

        raise Exception(error_message)


