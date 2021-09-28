# -*- coding: utf-8 -*-

import numpy as np
import math

class PassiveDSController:
    def __init__(
        self,
        bumper_l=0.2425,    # Bumper location [m] (210+32.5) mm
        bumper_R=0.33,      # Bumper size for contact location estiamtion [m] 330 mm
        Ts=1.0/50,          # Integration time step [s] 50 Hz
        robot_mass=2,       # Virtual Mass [kg]
        lambda_t=0.0,       # 
        lambda_n=0.5,       # 
        Fd=30,              # Desired Contact at the surface [N]
        activation_F=15,    # Minimal Contact to Switch to Passive-DS Control [N]
        logger=None
    ):
        self.bumper_l = bumper_l
        self.bumper_R = bumper_R
        self.Ts = Ts
        self.robot_mass = robot_mass
        self.activation_F = activation_F
        self.Lambda = np.diag([lambda_t, lambda_n])
        self.Fd = Fd
        self.D = self.Lambda

        # Internal State Variables
        self._Fmag = 0.0
        self._theta = 0.0
        self._h = 0.0
        self._Fx = 0.0
        self._Fy = 0.0
        self._Mz = 0.0

        # Loggers
        if logger:
            self.logger = logger
            self.logger.init_topic("bumper_loc", "compliance", ["t", "Fmag", "theta(rad)", "h"])

    def update_Ts(self, Ts):
        self.Ts = Ts

    def step(self, v_prev, omega_prev, v_cmd, omega_cmd, svr_data):
        (self._Fx, self._Fy, self._Mz) = (svr_data[0], svr_data[1], svr_data[2])
        self.damper_correction()
        if abs(self._Fmag) > self.activation_F:
            return self.get_control(v_prev, omega_prev, v_cmd, omega_cmd)
        else:
            return (v_cmd, omega_cmd)

    def log(self):
        self.logger.log('bumper_loc', self._Fmag, self._theta, self._h)

    def damper_correction(self):
        # Correcting based on trained SVR damping model
        self._h = 0.1

        (a, b, c) = (self._Fx, self._Fy, self._Mz/self.bumper_R)
        temp = a**2 + b**2 - c**2
        if temp == 0:
            self._theta = 0
        elif temp > 0:
            self._theta = np.real(-1j * np.log(
                (c + 1j*np.sqrt(temp)) /
                (a + 1j*b)
            ))
        else:
            self._theta = np.real(-1j * np.log(
                (c - np.sqrt(-temp)) /
                (a + 1j*b)
            ))

        self._Fmag = self._Fx*np.sin(self._theta) + self._Fy*np.cos(self._theta)

        self.bumper_loc = [self._Fmag, self._theta, self._h]

    def get_control(self, v_prev, omega_prev, v_cmd, omega_cmd):
        # Jacobian
        control_pt_x = self.bumper_l + self.bumper_R * np.cos(self._theta)
        control_pt_y = self.bumper_R * np.sin(self._theta)
        self.jacobian = np.array([
            [1., -control_pt_y],
            [0., control_pt_x],
        ])
        self.inv_jacobian = np.array([
            [1., control_pt_y/control_pt_x],
            [0., 1./control_pt_x],
        ])

        n_hat = np.array([np.cos(self._theta), np.sin(self._theta)])
        t_hat = np.array([-np.sin(self._theta), np.cos(self._theta)])

        Q = np.array([t_hat, n_hat]).T
        self.D = np.matmul(Q, np.matmul(self.Lambda, Q.T))

        V_prev = np.array(self.__differential_to_cartesian(v_prev, omega_prev))
        V_cmd = np.array(self.__differential_to_cartesian(v_cmd, omega_cmd))

        V = self.Ts / self.robot_mass * (
            - np.matmul(self.D, V_prev)
            + self.Fd * n_hat
            - self._Fmag * n_hat
        ) + np.matmul(t_hat.T, V_cmd) * t_hat

        # Vd = np.matmul(t_hat.T, V_cmd) * t_hat + (self.Fd - self._Fmag)/self.Lambda[1, 1]*n_hat
        # V = self.Ts / self.robot_mass * np.matmul(self.D, (Vd - V_prev))

        self.V_contact = np.matmul(n_hat.T, V)

        return self.__cartesian_to_differential(V[0], V[1])

    def __differential_to_cartesian(self, v, omega):
        return np.matmul(self.jacobian, np.array([v, omega]))

    def __cartesian_to_differential(self, vx, vy):
        return np.matmul(self.inv_jacobian, np.array([vx, vy]))
