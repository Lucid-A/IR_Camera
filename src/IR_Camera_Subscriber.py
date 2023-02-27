#! /usr/bin/env python

import rospy
from std_msgs.msg import String

def string_callback(data):
    if "true" == data.data:
        rospy.loginfo(f"[{rospy.get_caller_id()}] IR_Camera is available!")
    elif "false" == data.data:
        rospy.loginfo(f"[{rospy.get_caller_id()}] IR_Camera is not available!")
    else:
        rospy.loginfo(f"[{rospy.get_caller_id()}] Invalid Information!")


def listener():
    rospy.init_node("listener")
    rospy.Subscriber("IR_Camera/is_available", String, string_callback)

    rospy.spin()

if __name__ == "__main__":
    listener()
