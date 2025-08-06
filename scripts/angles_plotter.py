import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation as R

def animate(frame):
    data = pd.read_csv("../logs/fusion_output.csv")
    last_row = data.iloc[-1]
    q = last_row[['orient_w', 'orient_x', 'orient_y', 'orient_z']].to_numpy()
    r = R.from_quat(q[:], scalar_first=True)
    #euler = r.as_euler('xyz', degrees=True)
    matrix = r.as_matrix()

    plt.cla()

    origin = np.zeros(3)
    x_axis = matrix[:, 0]
    y_axis = matrix[:, 1]
    z_axis = matrix[:, 2]

    ax = plt.gcf().add_subplot(111, projection='3d')
    roll = ax.quiver(*origin, *x_axis, color='r', label="X (Roll)")
    pitch = ax.quiver(*origin, *y_axis, color='g', label="Y (Pitch)")
    yaw = ax.quiver(*origin, *z_axis, color='b', label="Z (Yaw)")
    ax.set_xlim((-1, 1))
    ax.set_ylim((-1, 1))
    ax.set_zlim((-1, 1))
    ax.set_title("Device orientation")
    ax.legend()
    return (roll, pitch, yaw)

ani = FuncAnimation(plt.gcf(), func=animate, frames=1, interval=1000)

plt.show()
