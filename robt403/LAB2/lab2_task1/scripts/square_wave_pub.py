#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import time

def publish_step_response():
    rospy.init_node('step_response_pub', anonymous=True)
    pub_base = rospy.Publisher('/robot/joint1_position_controller/command', Float64, queue_size=10)
    pub_end_effector = rospy.Publisher('/robot/joint5_position_controller/command', Float64, queue_size=10)
    rate = rospy.Rate(0.5)  
    high_value = 1.57  
    low_value = 0.0    
    duration = 30      
    toggle = True
    start_time = time.time()
    while not rospy.is_shutdown() and (time.time() - start_time < duration):
        if toggle:
            value = high_value  
        else:
            value = low_value   

        pub_base.publish(value)
        pub_end_effector.publish(value)
        rospy.loginfo(f"Publishing step response: {value}")
        toggle = not toggle  
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_step_response()
    except rospy.ROSInterruptException:
        pass
