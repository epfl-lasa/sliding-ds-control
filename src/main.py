#! /usr/bin/env python3

"""Example Code for running compliance controller"""

__version__ = "1.0"
__author__  = "Vaibhav Gupta"

import time
import os

import numpy as np

import signal
import rospy
from geometry_msgs.msg import WrenchStamped, Pose2D
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from termcolor import colored

from compliance_controller import AdmittanceController, PassiveDSController


try:
    os.nice(-10)
except PermissionError:
    # Need to run via sudo for high priority
    print(colored("Cannot set niceness for the process...", "red"))
    print(colored("Run the script as sudo...", "red"))


# Fast Clipper Function
clipper = lambda x, l, u: l if x < l else u if x > u else x


#########################################################
##################   FAKE ROBOT    ######################
#########################################################
# Fake Robot Input 
robot_input = lambda: (1, 0)

# Fake Robot Command 
robot_command = lambda V, W: True


#########################################################
################   ROBOT PARAMETERS   ###################
#########################################################
MAX_SPEED = 1.5
MIN_SPEED = 0.7
MAX_OMEGA = 4.124
W_RATIO = 3.5
CONTROL_FREQ = 200  # [Hz]


#########################################################
#########    Settings for Compliant Control   ###########
#########################################################
compliant_V =0.
compliant_W =0.

# Type of compliance controller to use
#   * admittance (default)
#   * passive_ds
COMPLIANCE_TYPE = "passive_ds"

compliance_control = None
dat_control_pt = None

# Global Variables for Compliant mode
svr_data =  np.zeros((3,))

# Global variables for timing
FULL_time = 0.


# ==== For interruption ====
def exit(signum, frame):
    # Exit any robot specific control 
    print(colored("===== You chose to interrupt =====", "red"))
    quit()

signal.signal(signal.SIGINT, exit)
signal.signal(signal.SIGTERM, exit)


# ==== Compliant control functions ====
def ft_sensor_callback(data):
    global svr_data
    _x = data.wrench
    svr_data = np.array([
        _x.force.x,
        _x.force.y,
        _x.torque.z,
    ])


# ==== Main control function ====
def control():
    global User_V, User_W, Output_V, Output_W, last_w, last_v
    global compliant_V, compliant_W, dat_control_pt, compliance_control
    
    (User_V, User_W) = robot_input()

    (compliant_V, compliant_W) = compliance_control.step(
        compliant_V, compliant_W,
        User_V, User_W,
        svr_data
    )

    # Update Control Point
    if abs(compliance_control._Fmag) > compliance_control.activation_F:
        dat_control_pt.x = compliance_control.bumper_l + compliance_control.bumper_R * np.cos(compliance_control._theta)
        dat_control_pt.y = compliance_control.bumper_R * np.sin(compliance_control._theta)
    else:
        dat_control_pt.x = compliance_control.bumper_l + compliance_control.bumper_R
        dat_control_pt.y = 0

    Output_V = round(compliant_V,6)
    Output_W = round(compliant_W,6)

    Output_V = clipper(Output_V, -MIN_SPEED, MAX_SPEED)
    Output_W = clipper(Output_W, -MAX_OMEGA, MAX_OMEGA)

    last_v = Output_V
    last_w = Output_W

    robot_command(Output_V, Output_W)


# ==== Control node ====
def control_node():
    global FULL_time, compliance_control, dat_control_pt
    prevT = 0
    
    if COMPLIANCE_TYPE is "passive_ds":
        compliance_control = PassiveDSController(
            bumper_l=0.2425,
            bumper_R=0.33,
            Ts=1.0/CONTROL_FREQ,
            robot_mass=2.0,
            lambda_t=0.0,
            lambda_n=1.5,
            Fd=45,
            activation_F=15,
            max_contact_speed=0.5,
            logger=None
        )
    else:
        compliance_control = AdmittanceController(
            v_max=MAX_SPEED,
            omega_max=(MAX_OMEGA/W_RATIO),
            bumper_l=0.2425,
            bumper_R=0.33,
            Ts=1.0/CONTROL_FREQ,
            Damping_gain=0.1,
            robot_mass=5,
            collision_F_max=45,
            activation_F=15,
            logger=None
        )
    
    pub_compliance_bumper_loc = rospy.Publisher('qolo/compliance/bumper_loc', Float32MultiArray, queue_size=1)
    dat_compliance_bumper_loc = Float32MultiArray()
    dat_compliance_bumper_loc.layout.dim.append(MultiArrayDimension())
    dat_compliance_bumper_loc.layout.dim[0].label = 'F_mag bumper_theta bumper_h'
    dat_compliance_bumper_loc.layout.dim[0].size = 3
    dat_compliance_bumper_loc.data = [0]*3

    pub_control_pt = rospy.Publisher('qolo/control_pt', Pose2D, queue_size=1)
    dat_control_pt = Pose2D()
    dat_control_pt.x = compliance_control.bumper_l + compliance_control.bumper_R
    dat_control_pt.y = 0
    
    ########### Starting ROS Node ###########
    rospy.init_node('control_node', anonymous=True)
    rate = rospy.Rate(CONTROL_FREQ)

    ftsub = rospy.Subscriber("qolo/compliance/svr",WrenchStamped,ft_sensor_callback, queue_size=1)
    rospy.loginfo('Starting Compliant Mode')
    
    print(colored(":"*80, "green"))
    print(colored("Ready to start experiments...", "green"))
    print(colored(":"*80, "green"))

    while not rospy.is_shutdown():
        prevT = time.clock()

        control()

        # ----- Publish ROS topics -----
        dat_compliance_bumper_loc.data = compliance_control.bumper_loc
        pub_compliance_bumper_loc.publish(dat_compliance_bumper_loc)

        pub_control_pt.publish(dat_control_pt)

        FULL_time = time.clock() - prevT
        
        # Ensure correct integration time
        compliance_control.update_Ts(FULL_time)

        rate.sleep()


if __name__ == '__main__':
    try:
        control_node()
    except rospy.ROSInterruptException:
        pass