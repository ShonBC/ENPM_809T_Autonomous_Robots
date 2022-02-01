import numpy as np
import matplotlib.pyplot as plt

def PlotIMUData(imu_angle, increment):

    # Generate x-axis data
    x = np.linspace(1, imu_angle.size, imu_angle.size)

    # Raw IMU Data Plotting
    fig = plt.figure()
    ax = fig.add_subplot(111)
    # plt.plot(x, imu_angle, 'k')
    ax.plot(x, imu_angle, color='red', linewidth=.5, label='IMU_Data')

    # Moving Average Plotting
    mavg = np.array(MovingAvg(imu_angle, increment))
    std_dev = np.std(mavg)
    mean_data = mavg.mean()
    x1 = np.linspace(1, imu_angle.size, mavg.size)
    ax.plot(x1, mavg, color='blue', linewidth=.5, label=f'Moving Average [{increment}]')
    ax.text(1,1,
            f'Mean: {mean_data}, \nStandard Deviation: {std_dev}',
            style='italic')

    ax.legend(loc='best')
    ax.set(title='IMU Data History',
    ylabel='IMU Angle Reading [Degrees]',
    xlabel='Data Entry Number')

    plt.show()
    
    

def MovingAvg(imu_angle, increment = 10):
    
    avg_list = []
    ang_list = []
    for i in range(len(imu_angle)):
        ang_list.append(imu_angle[i])
        if len(ang_list) >= increment:
            avg_list.append(sum(ang_list) / increment)
            ang_list = []

    return avg_list

if __name__ == '__main__':

    # Load imu angular data in the 5th column of the data file
    imu_angle = np.loadtxt('Homework_1/imudata.txt', usecols= 4)

    PlotIMUData(imu_angle, 2)
    PlotIMUData(imu_angle, 4)
    PlotIMUData(imu_angle, 8)
    PlotIMUData(imu_angle, 16)
    PlotIMUData(imu_angle, 64)
    PlotIMUData(imu_angle, 128)
