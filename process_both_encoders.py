"""Process Back Right and Front Left encoder data and plot
"""

import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':

    title = 'Motor Encoder Analysis Driving in Reverse 1 meter'

    # Load imu angular data in the 5th column of the data file
    BRstates = np.loadtxt('Homweork_7/docs/Reverse_BRencoderstates.txt')
    FLstates = np.loadtxt('Homweork_7/docs/Reverse_FLencoderstates.txt')
    x1 = np.arange(0, BRstates.size, step=1)
    x2 = np.arange(0, FLstates.size, step=1)

    # Time delta Data Plotting on same figure
    fig, axs = plt.subplots(2)
    fig.suptitle(title)

    axs[0].plot(x1, BRstates)
    axs[0].set(ylabel='Back Right Encoder')

    axs[1].plot(x2, FLstates, 'tab:red')
    axs[1].set(ylabel='Front Left Encoder')

    plt.show()
