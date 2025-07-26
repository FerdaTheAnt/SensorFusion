import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation as R


def animate(frame):
    data = pd.read_csv("../logs/fusion_output.csv")
    q = data[['orient_w', 'orient_x', 'orient_y', 'orient_z']].to_numpy()
    r = R.from_quat(q[:], scalar_first=True)
    euler = r.as_euler('xyz', degrees=True)
    plt.cla()
    roll = plt.plot(data['timestamp'][1:], euler[1:, 0], label="Roll")[0]
    pitch = plt.plot(data['timestamp'][1:], euler[1:, 1], label="Pitch")[0]
    yaw = plt.plot(data['timestamp'][1:], euler[1:, 2], label="Yaw")[0]
    return (roll, pitch, yaw)

ani = FuncAnimation(plt.gcf(), func=animate, frames=1, interval=1000)

plt.show()
