"""Plot the robots trajectory
"""

import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':

    title = 'Trajectory Commands'

    # Load imu angular data in the 5th column of the data file
    # BRstates = np.loadtxt('Homweork_7/docs/Reverse_BRencoderstates.txt')
    # FLstates = np.loadtxt('Homweork_7/docs/Reverse_FLencoderstates.txt')
    x = [0, -1, -1, 0, 0]
    y = [0, 0, -.5, -.5, 0]

    # Time delta Data Plotting on same figure
    fig, axs = plt.subplots(1)
    fig.suptitle(title)

    axs.plot(x, y)
    axs.set(ylabel='Y Position')
    axs.set(xlabel='X Position')

    plt.show()
