#! /usr/bin/env python3

"""Plot collision force on bumper"""

__version__ = '1.0'
__author__ = 'Vaibhav Gupta'

import queue
from itertools import count

import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import rospy
from std_msgs.msg import Float32MultiArray


# --------------------------------------------------
# Global Params
# --------------------------------------------------
INNER_R = 1
WIDTH_R = 1
THETA_MAX = 75
BAR_WIDTH = np.deg2rad(4)
BAR_HISTORY = 10
F_THRESHOLD = 10.0
F_MAX = 200.0

# Internal Global Parameters
_norm = mpl.colors.Normalize(vmin=0.0, vmax=F_MAX)
_CMAP = mpl.cm.ScalarMappable(norm=_norm, cmap=plt.cm.YlOrRd)
_CMAP.set_array([])

theta_queue = queue.Queue()
[theta_queue.put(0) for i in range(BAR_HISTORY)]

Fmag_queue = queue.Queue()
[Fmag_queue.put(0) for i in range(BAR_HISTORY)]

# Figure Set-up
fig = plt.figure(figsize=(6,5), constrained_layout=True)
gs = fig.add_gridspec(ncols=1, nrows=2, wspace=0, hspace=0, height_ratios=[12,1])
ax = fig.add_subplot(gs[0, :], polar=True, facecolor='#e7e7e7')

ax.set_ylim((0, WIDTH_R))
ax.set_rorigin(-INNER_R)

ax.set_theta_zero_location('N')
ax.set_thetamin(-THETA_MAX)
ax.set_thetamax(THETA_MAX)

ax.tick_params(axis='both',
            which='both',
            bottom=False,
            top=False,
            labelbottom=False,
            right=False,
            left=False,
            labelleft=False)
ax.grid(False)
ax.set_position([0.1, -0.55, 0.8, 2.0])

bars = ax.bar([0]*BAR_HISTORY, [WIDTH_R]*BAR_HISTORY, width=BAR_WIDTH, bottom=0.0)

cax = fig.add_subplot(gs[1, :])
cbar = fig.colorbar(_CMAP, cax=cax, orientation='horizontal')
cbar.set_label(label='Collision Force [N]', size=18)
cbar.ax.tick_params(labelsize=14)

# ---- Collision Force Callback ----
def collision_force_callback(data):
    Fmag_queue.get()
    theta_queue.get()
    Fmag_queue.put(data.data[0])
    theta_queue.put(data.data[1])


def frames():
    while True:
        yield list(Fmag_queue.queue), list(theta_queue.queue)


# Animation function
def animate(data):
    for i, bar, Fmag, theta in zip(count(), bars, *data):
        bar.set_x(theta)
        bar.set_color(_CMAP.to_rgba(Fmag))
        if Fmag < F_THRESHOLD:
            bar.set_alpha(0)
        else:
            bar.set_alpha(i*0.1 + 0.1)
    return bars,

def main():
    # ---- Starting ROS Node ----
    rospy.init_node('bumper_prediction', anonymous=True)
    rate = rospy.Rate(50)

    # ---- Suscribe to raw ft sensor ----
    ftsub = rospy.Subscriber(
        "/qolo/compliance/bumper_loc",
        Float32MultiArray,
        collision_force_callback,
        queue_size=1
    )

    # Create the Animation object
    ani = animation.FuncAnimation(fig, animate, frames=frames, interval=30)
    
    plt.show()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass