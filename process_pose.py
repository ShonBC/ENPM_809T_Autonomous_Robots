"""Process Back Right and Front Left encoder data and plot
"""

import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':

    title = 'Position History'

    # Load imu angular data in the 5th column of the data file
    x = np.loadtxt('Pose_data/5_7_1448pm/xpos_data.txt')
    y = np.loadtxt('Pose_data/5_7_1448pm/ypos_data.txt')

    # Time delta Data Plotting on same figure
    fig, axs = plt.subplots(1)
    fig.suptitle(title)

    axs.plot(x, y)
    axs.set(ylabel='Y Position')
    axs.set(xlabel='X Position')

    plt.show()
