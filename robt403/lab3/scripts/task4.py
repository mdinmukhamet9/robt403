import numpy as np
import matplotlib.pyplot as plt

def plot_manipulator(configurations, L1, L2, L3):
    fig, axs = plt.subplots(1, len(configurations), figsize=(10, 5))

    for idx, (q1, q2, q3) in enumerate(configurations):
        x0, y0 = 0, 0  
        x1, y1 = L1 * np.cos(q1), L1 * np.sin(q1)
        x2, y2 = x1 + L2 * np.cos(q1 + q2), y1 + L2 * np.sin(q1 + q2)
        x3, y3 = x2 + L3 * np.cos(q1 + q2 + q3), y2 + L3 * np.sin(q1 + q2 + q3)
        ax = axs[idx]
        ax.plot([x0, x1], [y0, y1], 'ro-', linewidth=2, markersize=8, label='Link 1')
        ax.plot([x1, x2], [y1, y2], 'go-', linewidth=2, markersize=8, label='Link 2')
        ax.plot([x2, x3], [y2, y3], 'bo-', linewidth=2, markersize=8, label='Link 3')
        arrow_length = 0.2 
        arrow_dx = arrow_length * np.cos(q1 + q2 + q3)
        arrow_dy = arrow_length * np.sin(q1 + q2 + q3)
        ax.arrow(x3, y3, arrow_dx, arrow_dy, head_width=0.1, head_length=0.1, fc='r', ec='r')
        ax.set_aspect('equal')
        ax.set_xlim([min(x0, x1, x2, x3) - 1, max(x0, x1, x2, x3) + 1])
        ax.set_ylim([min(y0, y1, y2, y3) - 1, max(y0, y1, y2, y3) + 1])
        ax.grid(True)
        ax.set_title(f"Configuration at t={idx * 10}s")
        ax.set_xlabel('X Position')
        ax.set_ylabel('Y Position')
        ax.legend()
    plt.tight_layout()
    plt.show()


initial_angles = (0, 0, 0) 
final_angles = (np.pi / 4, np.pi / 4, -np.pi / 4)
L1 = 0.675*2  
L2 = 0.675*2  
L3 = 3.167-0.675*4 

plot_manipulator([initial_angles, final_angles], L1, L2, L3)