#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import math
import time

def publish_snake_motion():
    rospy.init_node('snake_motion_pub', anonymous=True)

    pub_joint1 = rospy.Publisher('/robot/joint1_position_controller/command', Float64, queue_size=10)
    pub_joint2 = rospy.Publisher('/robot/joint2_position_controller/command', Float64, queue_size=10)
    pub_joint3 = rospy.Publisher('/robot/joint3_position_controller/command', Float64, queue_size=10)
    pub_joint4 = rospy.Publisher('/robot/joint4_position_controller/command', Float64, queue_size=10)
    pub_joint5 = rospy.Publisher('/robot/joint5_position_controller/command', Float64, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz
    
    amplitude = 1.0 
    frequency = 0.5  
    phase_shift = math.pi / 4  

    start_time = time.time()

    while not rospy.is_shutdown():
        current_time = time.time() - start_time
        
        joint1_value = amplitude * math.sin(2 * math.pi * frequency * current_time)
        joint2_value = amplitude * math.sin(2 * math.pi * frequency * current_time + phase_shift)
        joint3_value = amplitude * math.sin(2 * math.pi * frequency * current_time + 2 * phase_shift)
        joint4_value = amplitude * math.sin(2 * math.pi * frequency * current_time + 3 * phase_shift)
        joint5_value = amplitude * math.sin(2 * math.pi * frequency * current_time + 4 * phase_shift)

        pub_joint1.publish(joint1_value)
        pub_joint2.publish(joint2_value)
        pub_joint3.publish(joint3_value)
        pub_joint4.publish(joint4_value)
        pub_joint5.publish(joint5_value)

        rospy.loginfo(f"Joint1: {joint1_value}, Joint2: {joint2_value}, Joint3: {joint3_value}, Joint4: {joint4_value}, Joint5: {joint5_value}")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_snake_motion()
    except rospy.ROSInterruptException:
        pass
