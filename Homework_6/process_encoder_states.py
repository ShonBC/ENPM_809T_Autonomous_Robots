"""Process encoder data and plot
"""

import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':
    
    # Load imu angular data in the 5th column of the data file
    states = np.loadtxt('Homework_6/docs/encoderstates.txt')
    x = np.arange(0, states.size, step=1)

    # Time delta Data Plotting
    fig = plt.figure()
    ax = fig.add_subplot(111)

    ax.plot(x, states, 'o-', linewidth=.5, label='Raw Data')

    ax.legend(loc='best')
    ax.set(title='Motor Encoder Analysis',
    ylabel='Encoder State',
    xlabel='GPIO Input Reading')

    plt.show()
