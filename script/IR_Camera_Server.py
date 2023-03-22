#! /usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String
from IR_Camera.msg import IR_Camera_data
from IR_Camera.srv import ir_srv, ir_srvResponse

class IR_Server(object):
    def __init__(self):
        rospy.init_node('ir_server')
        s = rospy.Service('ir_service', ir_srv, self.handle_service)
        
        self.high_temp_threshold = 167 # >40C
        self.low_temp_threshold = 132 # <5C
        self.high_counter_threshold = 1
        self.low_counter_threshold = 1
        
        rospy.loginfo(f'[   IR] IR service initialized!')
        rospy.spin()
        
    def handle_service(self, req):
        high_temp_counter = 0
        low_temp_counter = 0
        
        if req.ir_data.is_available == "false":
            result_string = "not available"
        else:
            result_string = ""
            if sum(req.ir_data.temp_map) == 0:
                rospy.loginfo(f'[   IR] sum of IR temperature map is 0!')
            for data in req.ir_data.temp_map:
                if data > self.high_temp_threshold:
                    high_temp_counter += 1
                if data < self.low_temp_threshold:
                    low_temp_counter += 1
            if high_temp_counter > self.high_counter_threshold:
                result_string += "high "
            if low_temp_counter > self.low_counter_threshold:
                result_string += "low "
        
        return ir_srvResponse(result=result_string, high_count=high_temp_counter, low_count=low_temp_counter)
    
if "__main__" == __name__:
    IR_Server()
                
