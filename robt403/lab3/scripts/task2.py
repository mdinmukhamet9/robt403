import numpy as np

def cubic_coefficients(theta_0, theta_f, t_0, t_f, vel_0=0, vel_f=0):
    A = np.array([[1, t_0, t_0 ** 2, t_0 ** 3],
                  [0, 1, 2 * t_0, 3 * t_0 ** 2],
                  [1, t_f, t_f ** 2, t_f ** 3],
                  [0, 1, 2 * t_f, 3 * t_f ** 2]])
    B = np.array([theta_0, vel_0, theta_f, vel_f])
    coefficients = np.linalg.solve(A, B)
    return coefficients


def cubic_trajectory(coefficients, t):
    a0, a1, a2, a3 = coefficients
    return a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3
import matplotlib.pyplot as plt

t_0 = 0  
t_f = 10  
times = np.linspace(t_0, t_f, 100)  

theta_0_q1, theta_f_q1 = np.pi/2, np.pi/4  
theta_0_q2, theta_f_q2 = 0, np.pi/4  
theta_0_q3, theta_f_q3 = 0, -np.pi/4  
coeff_q1 = cubic_coefficients(theta_0_q1, theta_f_q1, t_0, t_f)
coeff_q2 = cubic_coefficients(theta_0_q2, theta_f_q2, t_0, t_f)
coeff_q3 = cubic_coefficients(theta_0_q3, theta_f_q3, t_0, t_f)
q1_traj = [cubic_trajectory(coeff_q1, t) for t in times]
q2_traj = [cubic_trajectory(coeff_q2, t) for t in times]
q3_traj = [cubic_trajectory(coeff_q3, t) for t in times]
plt.figure()
plt.plot(times, q1_traj, label='q1 (Joint 1)')
plt.plot(times, q2_traj, label='q2 (Joint 2)')
plt.plot(times, q3_traj, label='q3 (Joint 3)')
plt.title('Cubic Trajectories for Joints q1, q2, q3')
plt.xlabel('Time (s)')
plt.ylabel('Joint Angle (rad)')
plt.legend()
plt.grid(True)
plt.show()