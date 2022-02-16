#! /usr/bin/env python3

"""Bumper Prediction Node"""

__version__ = '1.0'
__author__ = 'Vaibhav Gupta'

import os
import time

import numpy as np
from termcolor import colored

import rospy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Header

from prediction_model import BumperModel

try:
    os.nice(-10)
except PermissionError:
    # Need to run via sudo for high priority
    print(colored("Cannot set niceness for the process...", "red"))
    print(colored("Run the script as sudo...", "red"))


# ---- Global Variables ----
model = None

offset_ft_data = np.zeros((6,))
raw_ft_data = np.zeros((6,))
ft_data = np.zeros((6,))
out_data = np.zeros((3,))
initialising_ft = True
init_ft_data = {
    'Fx': [],
    'Fy': [],
    'Fz': [],
    'Mx': [],
    'My': [],
    'Mz': [],
    }

# ---- Headers for rospy messages ----
def make_header(frame_name):
    header = Header()
    header.frame_id = frame_name
    header.stamp = rospy.get_rostime()
    return header

# ---- FT Sensor Callback ----
def ft_sensor_callback(data):
    global raw_ft_data, init_ft_data, ft_data
    _x = data.wrench
    raw_ft_data = np.array([
        _x.force.x,
        _x.force.y,
        _x.force.z,
        _x.torque.x,
        _x.torque.y,
        _x.torque.z,
    ])

    if initialising_ft:
        init_ft_data['Fx'] = np.append(init_ft_data['Fx'], _x.force.x)
        init_ft_data['Fy'] = np.append(init_ft_data['Fy'], _x.force.y)
        init_ft_data['Fz'] = np.append(init_ft_data['Fz'], _x.force.z)
        init_ft_data['Mx'] = np.append(init_ft_data['Mx'], _x.torque.x)
        init_ft_data['My'] = np.append(init_ft_data['My'], _x.torque.y)
        init_ft_data['Mz'] = np.append(init_ft_data['Mz'], _x.torque.z)
    else:
        ft_data = raw_ft_data - offset_ft_data

# ---- Main Function ----
def main():
    global model, initialising_ft, offset_ft_data

    model = BumperModel()

    pub_compliance_svr = rospy.Publisher('qolo/compliance/svr', WrenchStamped, queue_size=1)
    dat_compliance_svr = WrenchStamped()

    # ---- Starting ROS Node ----
    rospy.init_node('bumper_prediction', anonymous=True)
    rate = rospy.Rate(200)

    # ---- Suscribe to raw ft sensor ----
    ftsub = rospy.Subscriber(
        "/rokubi_node_front/ft_sensor_measurements",    # Change to correct ROS topic
        WrenchStamped,
        ft_sensor_callback,
        queue_size=1
    )
    initialising_ft = True
    rospy.loginfo('Waiting for FT Sensor Offset: 5 sec')
    time.sleep(5)
    initialising_ft = False
    offset_ft_data = np.array([
        np.mean(init_ft_data['Fx']),
        np.mean(init_ft_data['Fy']),
        np.mean(init_ft_data['Fz']),
        np.mean(init_ft_data['Mx']),
        np.mean(init_ft_data['My']),
        np.mean(init_ft_data['Mz']),
    ])

    print(colored(":"*80, "green"))
    print(colored("Bumper Prediction Initialization... done", "green"))
    print(colored(":"*80, "green"))

    while not rospy.is_shutdown():
        svr_data = model.predict(np.reshape(ft_data, (1,-1)))

        dat_compliance_svr.header = make_header("tf_ft_front")
        dat_compliance_svr.wrench.force.x = svr_data[0]
        dat_compliance_svr.wrench.force.y = svr_data[1]
        dat_compliance_svr.wrench.torque.z = svr_data[2]
        pub_compliance_svr.publish(dat_compliance_svr)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass