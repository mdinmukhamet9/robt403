#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
import time

def inverse_kinematics_3dof(X, Y, theta, L1, L2, L3):
    x_wrist = X - L3 * np.cos(theta)
    y_wrist = Y - L3 * np.sin(theta)

    # Calculate the distance to the wrist center
    D = (x_wrist ** 2 + y_wrist ** 2 - L1 ** 2 - L2 ** 2) / (2 * L1 * L2)

    if abs(D) > 1:
        raise ValueError("Target is outside of the reachable workspace.")

    # Compute q2 (elbow angle)
    q2 = np.arctan2(np.sqrt(1 - D ** 2), D)  # Elbow-up solution

    # Compute q1 (shoulder angle)
    q1 = np.arctan2(y_wrist, x_wrist) - np.arctan2(L2 * np.sin(q2), L1 + L2 * np.cos(q2))
    q3 = theta - q1 - q2

    return q1, q2, q3

def interpolate_points(x1, y1, x2, y2, num_steps):
    x_vals = np.linspace(x1, x2, num_steps)
    y_vals = np.linspace(y1, y2, num_steps)
    return x_vals, y_vals

def calculate_and_publish_ik(msg):
    try:
        x1 = msg.data[0]
        y1 = msg.data[1]

        x2 = msg.data[2]
        y2 = msg.data[3]

        theta = np.pi / 2

        num_steps = 10
        x_vals, y_vals = interpolate_points(x1, y1, x2, y2, num_steps)

        for i in range(num_steps):
            q1, q2, q3 = inverse_kinematics_3dof(x_vals[i], y_vals[i], theta, L1, L2, L3)
            joint1_pub.publish(q1)
            joint2_pub.publish(q2)
            end_pub.publish(q3)

            rospy.loginfo('Published angles: joint1=%.2f, joint2=%.2f, End=%.2f' % (np.degrees(q1), np.degrees(q2), np.degrees(q3)))
            time.sleep(1)
    except ValueError as e:
        rospy.logwarn("Error in calculating IK: %s" % e)

if __name__ == '__main__':
    try:
        rospy.init_node('inverse_kinematics_node', anonymous=True)
        L1 = 0.675 * 2  # Length of the first link
        L2 = 0.675 * 2  # Length of the second link
        L3 = 3.167 - 0.675 * 4  # Length of the third link
        joint1_pub = rospy.Publisher('/motortom2m/command', Float64, queue_size=10)
        joint2_pub = rospy.Publisher('/joint4/command', Float64, queue_size=10)
        end_pub = rospy.Publisher('/end/command', Float64, queue_size=10)

        rospy.Subscriber('/target_positions', Float64MultiArray, calculate_and_publish_ik)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
