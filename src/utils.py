import rospy

def read_param(param_name):
    param_value = rospy.get_param(param_name)
    if param_value is None:
        rospy.logerr(f"Failed to read {param_name} from the parameter server.")
        return None
    rospy.loginfo(f"Successfully read {param_name} from the parameter server: {param_value}")
    return param_value