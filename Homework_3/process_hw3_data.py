"""Plot time deltas showing how long the Raspberry Pi 3 took to process 
each frame detecting the green traffic light recorded.
"""

import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':

    # Load imu angular data in the 5th column of the data file
    time_deltas = np.loadtxt('Homework_3/hw3data.txt')
    x = np.arange(0, time_deltas.size, step=1)

    # Time delta Data Plotting
    fig = plt.figure()
    hist = plt.figure()
    ax = fig.add_subplot(111)
    ax2 = hist.add_subplot(111)
    
    ax.plot(x, time_deltas, 'o-', linewidth=.5, label='Raw Data')

    ax.legend(loc='best')
    ax.set(title='Object Tracking: Processing Time',
    ylabel='Processing Time [sec]',
    xlabel='Frame')

    ax2.hist(time_deltas, density=True, bins=time_deltas.size)

    ax2.set(title='Object Tracking: Processing Time',
    ylabel='Number of Frames',
    xlabel='Processing Time [sec]')

    plt.show()