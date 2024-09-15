#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float64
import time

def publish_sine_wave():
    rospy.init_node('sine_wave_pub', anonymous=True)

    pub_base = rospy.Publisher('/robot/joint1_position_controller/command', Float64, queue_size=10)
    pub_end_effector = rospy.Publisher('/robot/joint5_position_controller/command', Float64, queue_size=10)
    
    rate = rospy.Rate(50)  
    amplitude = 1.57  
    frequency = 0.5   
    duration = 60   

    start_time = time.time()

    while not rospy.is_shutdown() and (time.time() - start_time < duration):
        elapsed_time = time.time() - start_time
        sine_value = amplitude * math.sin(2 * math.pi * frequency * elapsed_time)
        pub_base.publish(sine_value)
        pub_end_effector.publish(sine_value)
        rospy.loginfo(f"Publishing sine wave value: {sine_value:.2f}")
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_sine_wave()
    except rospy.ROSInterruptException:
        pass
