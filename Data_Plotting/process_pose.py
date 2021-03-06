"""Process Back Right and Front Left encoder data and plot
"""

import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':

    title = 'Position History'

    # Load imu angular data in the 5th column of the data file
    x = np.loadtxt('xpos_data.txt')
    y = np.loadtxt('ypos_data.txt')

    # Time delta Data Plotting on same figure
    fig, axs = plt.subplots(1)
    fig.suptitle(title)

    axs.plot(x, y)
    axs.set(ylabel='Y Position [m]')
    axs.set(xlabel='X Position [m]')

    plt.show()
