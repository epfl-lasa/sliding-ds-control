#!/usr/bin/env python

import os, math, time

import rospy
import numpy as np
from geometry_msgs.msg import Wrench, WrenchStamped

from prediction_model import BumperModel
from filters import MultiLowPassFilter

# Set to high priority (max -20, min 19)
try:
    os.nice(-10)
except:
    # Need to run via sudo for high priority
    rospy.logerr("Cannot set niceness for the process...")
    rospy.logerr("Run the script as sudo...")


# Parameters
bumper_l = 0.2425      # (210+32.5) mm
bumper_r = 0.33 # 330 mm
Ts = 1.0/100    # 100 Hz
Damping_gain = 1           # 1 N-s/m 
robot_mass = 120         # 120 kg

# Global Variables
raw_ft_data = np.zeros((6,))
offset_ft_data = np.zeros((6,))
initialising = True
init_ft_data = {
    'Fx': [0],
    'Fy': [0],
    'Fz': [0],
    'Mx': [0],
    'My': [0],
    'Mz': [0],
}

# Prediction Models
bumperModel = None

# Filter
lp_filt = None

def ft_sensor_callback(data):
    _x = data.wrench
    raw_ft_data = np.array([
        _x.force.x,
        _x.force.y,
        _x.force.z,
        _x.torque.x,
        _x.torque.y,
        _x.torque.z,
    ])

    if initialising:
        init_ft_data['Fx'] = np.append(init_ft_data['Fx'], _x.force.x)
        init_ft_data['Fy'] = np.append(init_ft_data['Fy'], _x.force.y)
        init_ft_data['Fz'] = np.append(init_ft_data['Fz'], _x.force.z)
        init_ft_data['Mx'] = np.append(init_ft_data['Mx'], _x.torque.x)
        init_ft_data['My'] = np.append(init_ft_data['My'], _x.torque.y)
        init_ft_data['Mz'] = np.append(init_ft_data['Mz'], _x.torque.z)


def compliance_control(v_prev, omega_prev, Fmag, h, theta):
    # F = robot_mass \Delta \ddot{x} + Damping_gain \Delta \dot{x} + K \Delta x
    # And set reference to 0 and discretize w/ ZOH
    
    stheta = math.sin(theta)    # Small optimization
    ctheta = math.cos(theta)    # Small optimization

    # Position wrt center of rotatiion
    R = math.sqrt((bumper_r*stheta)**2 + (bumper_l + bumper_r*ctheta)**2 )
    beta = math.atan2(bumper_r * stheta, bumper_l)

    sbeta = math.sin(beta)      # Small optimization
    cbeta = math.cos(beta)      # Small optimization
    
    a = ctheta
    b = R*(stheta*cbeta - ctheta*sbeta)

    # Admittance Control
    v_eff_prev = (a * v_prev) + (b * omega_prev)

    v_eff_dot = (-Fmag - Damping_gain*v_eff_prev) / robot_mass
    v_eff = v_eff_dot * Ts + v_eff_prev

    # # Calculate new v and omega
    # c_prev = (-b * v_prev) + (a * omega_prev)
    # den = a**2 - b**2
    # v = (a*v_eff - b*c_prev) / den
    # omega = (-b*v_eff + a*c_prev) / den

    # Ensure non-zero 'a' and 'b'
    eps = 0.01
    if (a < eps):
        return (v_prev, v_eff/b)
    if (b < eps):
        return (v_eff/a, omega_prev)

    # Calculate new v and omega in parameterized form
    t = 0.5     # \in [0,1]
    v = t * v_prev + (1-t) * (v_eff - b*omega_prev) / a
    omega = t * (v_eff - a*v_prev) / b + (1-t) * omega_prev

    return (v, omega)


def damper_correction(ft_data):
    # Dummy function for future
    (Fx, Fy, Mz) = bumperModel.predict(np.reshape(ft_data, (1, -1)))

    h = 0.1
    (a, b, c) = (Fx, Fy, Mz/bumper_r)
    theta = np.real(-1j * np.log(
        (c + 1j*np.sqrt(a**2 + b**2 - c**2)) /
        (a + 1j*b)
    ))
    Fmag = Fx*np.sin(theta) + Fy*np.cos(theta)
    
    rospy.loginfo(
        "\n\tFx = {}\n\tFy = {}\n\tMz = {}\n\ttheta = {}\n\tFmag = {}\n-------------------------\n".format(
            Fx, Fy, Mz, theta, Fmag
        )
    )

    return (Fmag, h, theta)


def transform_to_bumper_surface(ft_data, h, theta):
    transformed_data = np.zeros((6,))

    # Fx
    transformed_data[0] = (ft_data[0] * math.cos(theta)
        - ft_data[1] * math.sin(theta))
    
    # Fy
    transformed_data[1] = ft_data[2]

    # Fz
    transformed_data[2] = (- ft_data[0] * math.sin(theta)
        - ft_data[1] * math.cos(theta))

    # Mx
    transformed_data[3] = (ft_data[3] * math.cos(theta)
        - ft_data[4] * math.sin(theta)
        - transformed_data[2] * h
        + transformed_data[1] * bumper_r)
    
    # My
    transformed_data[4] = (ft_data[5]
        - transformed_data[0] * bumper_r)

    # Mz
    transformed_data[5] = (- ft_data[3] * math.sin(theta)
        - ft_data[4] * math.cos(theta))
    
    return transformed_data


if __name__ == "__main__":
    lp_filt = MultiLowPassFilter(size=6)
    bumperModel = BumperModel()

    rospy.init_node("listener", anonymous=True)
    rospy.Subscriber("/rokubi_node_front/ft_sensor_measurements", 
                     WrenchStamped,
                     ft_sensor_callback)
    
    pub_rate = rospy.Rate(100)
    pub = rospy.Publisher("/compliance_control", Wrench, queue_size=10)

    v_prev = 5
    omega_prev = 1
    start_time = time.time()
    while (True):
        if initialising and (time.time() - start_time > 4): # 4 s for initialising
            initialising = False
            offset_ft_data = np.array([
                np.mean(init_ft_data['Fx']),
                np.mean(init_ft_data['Fy']),
                np.mean(init_ft_data['Fz']),
                np.mean(init_ft_data['Mx']),
                np.mean(init_ft_data['My']),
                np.mean(init_ft_data['Mz']),
            ])

        if not initialising:
            Fmag, h, theta = damper_correction(lp_filt.filter(raw_ft_data - offset_ft_data))
            # ft_data = transform_to_bumper_surface(ft_data, h, theta)
            v, omega = compliance_control(v_prev, omega_prev, Fmag, h, theta)

            v_prev = v
            omega_prev = omega
        
        pub_rate.sleep()