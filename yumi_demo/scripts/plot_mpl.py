import numpy as np
import matplotlib.pyplot as plt
import rospy
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Float64
import threading

# settings
topic_1 = '/plot/dist'
topic_2 = '/plot/TM_poly'
topic_3 = '/plot/TM_UB'
topic_4 = '/plot/TM_LB'
delta_x = 0.0005
delta_y = 0.0005
delta = 0.0

# global data
data_1 = 0.0
data_2 = 0.0
data_3 = 0.0
data_4 = 0.0

lock_1  = threading.Lock()
lock_2  = threading.Lock()
lock_3  = threading.Lock()
lock_4  = threading.Lock()




# ros topic callback
def sub_data_1(msg):
    global data_1, lock_1
    lock_1.acquire()
    data_1 = msg.data
    lock_1.release()
    # print(data_1)


def sub_data_2(msg):
    global data_2,lock_2
    lock_2.acquire()
    data_2 = msg.data
    lock_2.release()


def sub_data_3(msg):
    global data_3,lock_3
    lock_3.acquire()
    data_3 = msg.data
    lock_3.release()


def sub_data_4(msg):
    global data_4,lock_4
    lock_4.acquire()
    data_4 = msg.data
    lock_4.release()


def spin():
    rospy.spin()


if __name__ == '__main__':
    '''
    ros
    '''
    # ros init
    rospy.init_node('subscriber')
    sub_1 = rospy.Subscriber(topic_1, Float64, callback=sub_data_1, queue_size=1)
    sub_2 = rospy.Subscriber(topic_2, Float64, callback=sub_data_2, queue_size=1)
    sub_3 = rospy.Subscriber(topic_3, Float64, callback=sub_data_3, queue_size=1)
    sub_4 = rospy.Subscriber(topic_4, Float64, callback=sub_data_4, queue_size=1)

    # data update thread
    sub_thread = threading.Thread(target=spin)
    sub_thread.start()

    '''
    plot
    '''
    # plot area init
    fig, ax = plt.subplots()

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

    # animation init
    def init():
        ax.grid(False)
        return ln_1, ln_2, ln_3, ln_4


    # animation data update
    def update(frames):
        frame = rospy.Time.now().to_time()
        xdata_1.append(frame)
        xdata_2.append(frame)
        xdata_3.append(frame)
        xdata_4.append(frame)
        ax.set_xlim(frame - delta_x * 2, frame)

        lock_1.acquire()
        ydata_1.append(data_1)
        lock_1.release()

        lock_2.acquire()
        ydata_2.append(data_2)
        lock_2.release()

        lock_3.acquire()
        ydata_3.append(data_3)
        lock_3.release()

        lock_4.acquire()
        ydata_4.append(data_4)
        lock_4.release()

        if data_1 > data_3 or data_1 < data_4:
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

        # ax.set_ylim(data_1 - delta_y, data_1 + delta_y)
        delta = 0.5 * (data_3-data_4)
        ax.set_ylim(data_4-delta, data_3+delta)
        # ax.set_ylim(-0.2, 0.8)

        ln_1.set_data(xdata_1, ydata_1)
        ln_2.set_data(xdata_2, ydata_2)
        ln_3.set_data(xdata_3, ydata_3)
        ln_4.set_data(xdata_4, ydata_4)

        return ln_1, ln_2, ln_3, ln_4


    # start animation ploting
    ani = FuncAnimation(fig, update, frames=None,
                        init_func=init, interval=10, blit=False)

    # plt.xlabel('time')
    # plt.ylabel('distance')
    plt.show()
