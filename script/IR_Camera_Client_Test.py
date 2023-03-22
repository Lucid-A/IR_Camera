#! /usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Header, String
from IR_Camera.msg import IR_Camera_data
from IR_Camera.srv import ir_srv, ir_srvResponse

from playsound import playsound

class ir_client(object):
    def __init__(self):
        rospy.wait_for_service('ir_service')
        self.service = rospy.ServiceProxy('ir_service', ir_srv)
        self.sub = rospy.Subscriber("IR_Camera/data", IR_Camera_data, self.callback)
        
        self.play = True
        # self.play = False
        self.counter = 0
        
    def callback(self, msg):
        if not msg.is_available == "true":
            rospy.loginfo("[   IR] IR Client: get msg but not available")
            print(msg)
        else:
            resp = self.service(msg)
            print(msg)
            print(resp)
            res_str = resp.result
            rospy.loginfo("[   IR] IR Client: get msg")
            self.counter += 1
            if self.counter > 50:
                self.play = True
                self.counter = 0
                print(f'IR sound play reset counter {self.counter}!')
                #playsound('/home/nx/zlz_ws/src/IR_Camera/script/high.wav')
            if self.play and res_str != '':
                self.play = False
                if 'high' in res_str:
                    rospy.loginfo("[   IR] IR Client: 高温警报!")
                    playsound('/home/nx/zlz_ws/src/IR_Camera/script/high.wav')
                if 'low' in res_str:
                    rospy.loginfo("[   IR] IR Client: 低温警报!")
                    playsound('/home/nx/zlz_ws/src/IR_Camera/script/low.wav')

            
    
if __name__ == "__main__":
    rospy.init_node("IR_Camera_Client_Test", anonymous=False)
    ir_client()
    rospy.spin()
