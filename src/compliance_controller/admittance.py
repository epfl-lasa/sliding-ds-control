import numpy as np
import math

class AdmittanceController:
    def __init__(
        self,
        v_max,
        omega_max,
        bumper_l=0.2425,    # [m] (210+32.5) mm
        bumper_R=0.33,      # [m] 330 mm
        Ts=1.0/50,          # [s] 50 Hz
        Damping_gain=0.2,   # [N-s/m]
        robot_mass=5,       # [kg]
        collision_F_max=45, # [N]
        activation_F=15,    # [N]
        logger=None
    ):
        self.v_max = v_max
        self.omega_max = omega_max
        self.bumper_l = bumper_l
        self.bumper_R = bumper_R
        self.Ts = Ts
        self.Damping_gain = Damping_gain
        self.robot_mass = robot_mass
        self.collision_F_max = collision_F_max
        self.activation_F = activation_F
        self.nominal_F = self.collision_F_max

        # Internal State Variables
        self._Fmag = 0.0
        self._theta = 0.0
        self._h = 0.0
        self._p = 0.0
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
        self._Fx = svr_data[0]
        self._Fy = svr_data[1]
        self._Mz = svr_data[2]
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
        # F = robot_mass \Delta \ddot{x} + Damping_gain \Delta \dot{x} + K \Delta x
        # And set reference to 0 and discretize w/ ZOH
        stheta = math.sin(self._theta)    # Small optimization
        ctheta = math.cos(self._theta)    # Small optimization

        # Position wrt center of rotatiion
        O = math.sqrt((self.bumper_R*stheta)**2 + (self.bumper_l + self.bumper_R*ctheta)**2 )
        beta = math.atan2(self.bumper_R * stheta, self.bumper_l + self.bumper_R*ctheta)

        sbeta = math.sin(beta)      # Small optimization
        cbeta = math.cos(beta)      # Small optimization

        # Admittance Control      
        a = ctheta
        b = O * (stheta*cbeta - ctheta*sbeta)
        
        V_prev = (a * v_prev) + (b * omega_prev)
        V_cmd  = (a * v_cmd)  + (b * omega_cmd)
        
        eff_robot_mass = self.robot_mass
        if (abs(V_cmd) > (self.collision_F_max * self.Ts) / self.robot_mass):
            eff_robot_mass = (self.collision_F_max * self.Ts) / abs(V_cmd)

        V_dot = (self.nominal_F - self._Fmag - self.Damping_gain*V_prev) / eff_robot_mass
        V = V_dot * self.Ts  # + V_cmd

        # Calculate new v and omega in parameterized form
        a = 1.0 
        b = -(stheta*cbeta - ctheta*sbeta) / self.omega_max * self.v_max

        # Ensure non-zero 'a' and 'b'
        eps = 0.01
        if (abs(a) < eps):
            return (v_cmd, V/b)
        if (abs(b) < eps):
            return (V/a, omega_cmd)

        _ = V - a*v_cmd / b
        if _ > self.omega_max:
            t_max = (self.omega_max - omega_cmd) / (_ - omega_cmd)
        elif _ < -self.omega_max:
            t_max = (-self.omega_max - omega_cmd) / (_ - omega_cmd)
        else:
            t_max = 1.0

        _ = V - b*omega_cmd / a
        if _ > self.v_max:
            t_min = (self.v_max - omega_cmd) / (_ - omega_cmd)
        elif _ < -self.v_max:
            t_min = (-self.v_max - omega_cmd) / (_ - omega_cmd)
        else:
            t_min = 0.0

        self._p = self.__map(math.fabs(self._theta),
                       [0.0, np.pi],
                       [t_min, t_max])

        v = self._p * v_cmd + (1-self._p) * (V - b*omega_cmd) / a
        omega = self._p * (V - a*v_cmd) / b + (1-self._p) * omega_cmd

        return (v, omega)

    def __map(self, x, from_range, to_range):
        return (to_range[0]
                + ((x - from_range[0])
                   * (to_range[1] - to_range[0])
                   / (from_range[1] - from_range[0])))
