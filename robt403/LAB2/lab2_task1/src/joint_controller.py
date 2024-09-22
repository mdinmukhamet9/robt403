#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

class JointController:
    def __init__(self):        
        rospy.init_node('joint_controller', anonymous=True)        
        self.joint_pub = rospy.Publisher('/robot/joint5_position_controller/command', Float64, queue_size=10)      
        rospy.Subscriber('/incoming_data', Float64, self.callback)     
        self.last_value = -float('inf')  # initial value, which is minimal
        
    def callback(self, msg):
        # Here I check if the given value is bigger than existing 
        if msg.data > self.last_value:
            rospy.loginfo(f"Publishing new higher value: {msg.data}")
            self.joint_pub.publish(msg)
            self.last_value = msg.data 

if __name__ == '__main__':
    try:       
        controller = JointController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

