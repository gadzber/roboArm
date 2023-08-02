import matplotlib.pyplot as plt
import numpy as np
import time

def plot_robot(q1, q2, ax):
    rot1 = np.array([[np.cos(q1), -np.sin(q1)], [np.sin(q1), np.cos(q1)]])
    rot2 = np.array([[np.cos(q1 + q2), -np.sin(q1 + q2)], [np.sin(q1 + q2), np.cos(q1 + q2)]])

    link1 = np.array([[0, -400, -400], [0, 0, 60]])
    link2 = np.array([[0, 400, 400], [0, 0, 60]])

    link1new = rot1 @ link1
    link2new = rot2 @ link2 + link1new[:, [-1]]

    plt.cla()
    ax.plot(link1new[0, :], link1new[1, :])
    ax.plot(link2new[0, :], link2new[1, :])
    ax.plot(0,0,'ro')
    ax.plot(link1new[0,-1],link1new[1,-1],'ro')
    ax.plot(link2new[0,-1],link2new[1,-1],'ro')

    ax.axis([-600, 600, -200, 1000])
    # ax.axis('equal')
    ax.grid('on')

    plt.draw()
    plt.pause(0.0001)


fig, ax = plt.subplots()

ax.axis([-600,600,-200,1000])
plt.ion()
plt.show()

t = np.linspace(0,2*np.pi,100)
q1 = np.sin(t)
q2 = np.sin(2*t)

for i in range(100):
    plot_robot(q1[i], q2[i],ax)
    # time.sleep(0.01)


