import numpy as np
import matplotlib.pyplot as plt
import rospy
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Float64, Float64MultiArray
import threading
import math

# settings
topic_1 = 'data_1'
topic_2 = 'data_2'
topic_3 = 'data_3'
topic_4 = 'data_4'
topic_5 = 'data_all'
delta_x = 0.0005
delta_y = 0.0005
delta = 0.0

# global data
data_1 = 0.0
data_2 = 0.0
data_3 = 0.0
data_4 = 0.0
data_all = [0, 0, 0, 0, 0]

lock_1 = threading.Lock()
lock_2 = threading.Lock()
lock_3 = threading.Lock()
lock_4 = threading.Lock()
lock_all = threading.Lock()


# ros topic callback
def sub_data_1(msg):
    global data_1, lock_1
    lock_1.acquire()
    data_1 = msg.data
    lock_1.release()
    # print(data_1)


def sub_data_2(msg):
    global data_2, lock_2
    lock_2.acquire()
    data_2 = msg.data
    lock_2.release()


def sub_data_3(msg):
    global data_3, lock_3
    lock_3.acquire()
    data_3 = msg.data
    lock_3.release()


def sub_data_4(msg):
    global data_4, lock_4
    lock_4.acquire()
    data_4 = msg.data
    lock_4.release()


def sub_data_all(msg):
    global data_all, lock_all
    lock_all.acquire()
    # data_all = np.array(msg.data, dtype = np.float64)
    # for i in range(len(data_all)):
    #     data_all[i] = math.log(data_all[i], 10)
    data_all = msg.data

    lock_all.release()
    # print(data_all)


def spin():
    rospy.spin()


if __name__ == '__main__':
    '''
    ros
    '''
    # ros init
    rospy.init_node('subscriber')
    # sub_1 = rospy.Subscriber(topic_1, Float64, callback=sub_data_1, queue_size=1)
    # sub_2 = rospy.Subscriber(topic_2, Float64, callback=sub_data_2, queue_size=1)
    # sub_3 = rospy.Subscriber(topic_3, Float64, callback=sub_data_3, queue_size=1)
    # sub_4 = rospy.Subscriber(topic_4, Float64, callback=sub_data_4, queue_size=1)
    sub_5 = rospy.Subscriber(topic_5, Float64MultiArray, callback=sub_data_all, queue_size=1)

    # data update thread
    sub_thread = threading.Thread(target=spin)
    sub_thread.start()

    '''
    plot
    '''
    # plot area init
    fig, ax = plt.subplots()
    ax.grid(False)

    # plot data array init
    xdata_1, ydata_1 = [], []
    xdata_2, ydata_2 = [], []
    xdata_3, ydata_3 = [], []
    xdata_4, ydata_4 = [], []

    # set line format
    ln_1, = ax.plot([], [], '--', color = '#0072BD', label = 'dist', animated=False)
    ln_2, = ax.plot([], [], '-', color = '#7E2F8E', label = 'TM_ploy', animated=False)
    ln_3, = ax.plot([], [], '-', color = '#EDB120', label = 'TM_UB', animated=False)
    ln_4, = ax.plot([], [], '-', color = '#D95319', label = 'TM_LB', animated=False)

    ax.legend(loc = 'upper right')

    # animation data update
    def update(frames):
        global data_all
        frame = rospy.Time.now().to_time()
        # print(frame)
        xdata_1.append(frame)
        xdata_2.append(frame)
        xdata_3.append(frame)
        xdata_4.append(frame)
        ax.set_xlim(frame - delta_x * 2, frame)

        # print(lock_1.locked(), " ", lock_2.locked(), " ", lock_3.locked(), " ", lock_4.locked())
        #
        # lock_1.acquire()
        # ydata_1.append(data_1)
        # ax.set_ylim(data_1 - delta_y, data_1 + delta_y)
        # lock_1.release()
        #
        # lock_2.acquire()
        # ydata_2.append(data_2)
        # lock_2.release()
        #
        # lock_3.acquire()
        # ydata_3.append(data_3)
        # lock_3.release()
        #
        # lock_4.acquire()
        # ydata_4.append(data_4)
        # lock_4.release()

        if data_all[0] > data_all[2] or data_all[0] < data_all[3]:
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

        lock_all.acquire()
        ydata_1.append(data_all[0])
        ydata_2.append(data_all[1])
        ydata_3.append(data_all[2])
        ydata_4.append(data_all[3])

        delta = 2 * (data_all[2]-data_all[3])
        # delta = 0.000005
        ax.set_ylim(data_all[0]-delta, data_all[0]+delta)
        # ax.set_ylim(-10, 10)

        lock_all.release()

        ln_1.set_data(xdata_1, ydata_1)
        ln_2.set_data(xdata_2, ydata_2)
        ln_3.set_data(xdata_3, ydata_3)
        ln_4.set_data(xdata_4, ydata_4)
        return ln_1, ln_2, ln_3, ln_4


    # start animation ploting
    # ani = FuncAnimation(fig, update, frames=None,
    #                     interval=10, blit=True)
    ani = FuncAnimation(fig, update, frames=None,
                        interval=1, blit=False)
    plt.show()