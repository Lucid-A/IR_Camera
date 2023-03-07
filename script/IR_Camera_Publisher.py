#! /usr/bin/env python

import rospy
from std_msgs.msg import Header, String
from IR_Camera.msg import IR_Camera_data
from IR_sensor import IR_sensor
 
def IR_Camera_Node(cam_path=r"/dev/ttyTHS0"):
    sensor = IR_sensor(dev=cam_path)
    sensor.open()

    pub_available = rospy.Publisher("IR_Camera/is_available", String, queue_size=10)
    pub_data = rospy.Publisher("IR_Camera/data", IR_Camera_data, queue_size=10)

    rospy.init_node("IR_Camera_Node", anonymous=False)
    rate = rospy.Rate(10) # 10Hz
    
    header = Header(
        seq = 1,
        stamp = rospy.Time.now(),
        frame_id = "Not Specified"
    )
    available_str = "false"
    temp_map = None
    while not rospy.is_shutdown():
        if not sensor.isOpen():
            sensor.open()
            if not sensor.isOpen():
                available_str = "false"
                temp_map = None
        
        if sensor.isOpen():
            available_str = "true"
            temp_map = sensor.get_temp_map()

        header.seq = header.seq + 1
        header.stamp = rospy.Time.now()
        data = IR_Camera_data(
            header = header,
            is_available = available_str,
            temp_map = temp_map
        )
        pub_available.publish(available_str)
        pub_data.publish(data)
        rate.sleep()

if __name__ == '__main__':
    try:
        IR_Camera_Node()
    except rospy.ROSInterruptException:
        print("ROSInterruptException")

