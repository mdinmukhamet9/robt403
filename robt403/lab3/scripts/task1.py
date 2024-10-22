import numpy as np
import matplotlib.pyplot as plt
L0 = 0
L1 = 0.675*2 
L2 = 0.675*2  
L3 = 3.167-0.675*4  

def forward_kinematics(q1, q2, q3):
    x = L0 + L1 * np.cos(q1) + L2 * np.cos(q1 + q2) + L3 * np.cos(q1 + q2 + q3)
    y = L0 + L1 * np.sin(q1) + L2 * np.sin(q1 + q2) + L3 * np.sin(q1 + q2 + q3)
    return x, y

def plot_workspace():
    q1_range = np.linspace(-np.pi / 2, np.pi / 2, 100)
    q2_range = np.linspace(-np.pi / 2, np.pi / 2, 100)
    q3_range = np.linspace(-np.pi / 2, np.pi / 2, 100)
    x_positions = []
    y_positions = []
    for q1 in q1_range:
        for q2 in q2_range:
            for q3 in q3_range:
                x, y = forward_kinematics(q1, q2, q3)
                x_positions.append(x)
                y_positions.append(y)
    plt.figure(figsize=(8, 8))
    plt.plot(x_positions, y_positions, 'b.', markersize=1)
    plt.title('Workspace of 3 DoF Planar Robot')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.grid(True)
    plt.axis('equal')
    plt.show()
plot_workspace()