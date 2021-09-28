import math
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from matplotlib.patches import FancyArrow, Arc, RegularPolygon
import numpy as np

def draw_v(ax, centX, centY, v,
           v_max=1.5,
           max_len=0.75,
           color='black'):
    # Make sure you keep the axes scaled or else arrow will distort
    
    # ---- v to length ----
    length = (v / v_max) * max_len
        
    # ---- Arrow -----
    arrow = FancyArrow(
        centX, centY, 0, length, 0.02,
        capstyle='round', linestyle='-', 
        linewidth=3, color=color
    )
    ax.add_patch(arrow)


def draw_omega(ax, centX, centY, omega,
              omega_max=4,
              radius=0.5,
              angle=0,
              color='black'):
    # Make sure you keep the axes scaled or else arrow will distort

    # ---- omega to angle ----
    theta = (omega / omega_max) * 300
        
    # ---- Line -----
    _theta = abs(theta)
    if omega < 0:
        _angle = angle + 360 - _theta
    else:
        _angle = angle
    arc = Arc([centX,centY], radius, radius, angle=_angle,
              theta1=0, theta2=_theta,
              capstyle='round', linestyle='-', 
              linewidth=5, color=color)
    ax.add_patch(arc)

    # ---- Create the arrow head ---
    # Do trig to determine end position
    orientation = math.radians(theta+angle)
    endX=centX+(radius/2)*math.cos(orientation)
    endY=centY+(radius/2)*math.sin(orientation)
    
    if omega < 0:
        orientation += math.pi

    ax.add_patch(           # Create triangle as arrow head
        RegularPolygon(
            (endX, endY),   # (x,y)
            3,              # number of vertices
            radius/9,       # radius
            orientation,    # orientation
            color=color
        )
    )


class Simulation:
    """Simulation class
    """
    # Vehicle Parameters
    l = 0.2425      # [m] : (210+32.5) mm
    r = 0.33        # [m] : 330 mm
    Ts = 1.0/100    # [s] : 100 Hz
    C = 1           # [N-s/m] 
    M = 3          # [kg]

    v_max = 1.5
    omega_max = 4

    # Simulation Values
    t = 0.5
    v = 0.0
    omega = 0.0
    Fmag = 0.0
    theta = 0.0
    h = 0.0

    # Plot Settings
    axcolor = 'lightgoldenrodyellow'
    sliders = {}
    ax = None

    def __init__(self):
        fig, self.ax = plt.subplots()

        plt.subplots_adjust(left=0.2, bottom=0.3, right=0.8, top=0.9)

        # Slider axis
        axsliders = {}
        axsliders["Fmag"] = plt.axes([0.2, 0.2, 0.6, 0.03], facecolor=self.axcolor)
        axsliders["theta"] = plt.axes([0.2, 0.15, 0.6, 0.03], facecolor=self.axcolor)
        axsliders["h"] = plt.axes([0.2, 0.1, 0.6, 0.03], facecolor=self.axcolor)

        axsliders["t"] = plt.axes([0.1, 0.3, 0.03, 0.6], facecolor=self.axcolor)

        axsliders["v"] = plt.axes([0.85, 0.3, 0.03, 0.6], facecolor=self.axcolor)
        axsliders["omega"] = plt.axes([0.9, 0.3, 0.03, 0.6], facecolor=self.axcolor)

        # Sliders
        self.sliders["Fmag"] = Slider(
            axsliders["Fmag"], "Force", 0.0, 200.0,
            valinit=self.Fmag,
            valstep=0.5,
            orientation="horizontal"
        )
        self.sliders["theta"] = Slider(
            axsliders["theta"], "theta", -60.0, 60.0,
            valinit=self.theta,
            valstep=5,
            orientation="horizontal"
        )
        self.sliders["h"] = Slider(
            axsliders["h"], "h", 0.0, 0.2,
            valinit=self.h,
            valstep=0.04,
            orientation="horizontal"
        )

        self.sliders["t"] = Slider(
            axsliders["t"], "t", 0.0, 1.0,
            valinit=self.t,
            orientation="vertical"
        )

        self.sliders["v"] = Slider(
            axsliders["v"], "v", -self.v_max, self.v_max,
            valinit=self.v,
            valstep=0.1,
            orientation="vertical"
        )
        self.sliders["omega"] = Slider(
            axsliders["omega"], "omega", -self.omega_max, self.omega_max,
            valinit=self.omega,
            valstep=0.1,
            orientation="vertical"
        )

        # Slider Updates
        for fieldname in self.sliders:
            self.sliders[fieldname].on_changed(self.update)

        # Visualisation
        self.visualise()

        plt.show()

    def update(self, _):
        self.v = self.sliders["v"].val
        self.omega = self.sliders["omega"].val
        self.t = self.sliders["t"].val
        self.Fmag = self.sliders["Fmag"].val
        self.theta = self.sliders["theta"].val
        self.h = self.sliders["h"].val
        self.visualise()

    def visualise(self):
        self.ax.clear()
        self.ax.set_xlim([-1, 1])
        self.ax.set_ylim([-1, 1])

        draw_v(self.ax, -0.5, 0, self.v, color='red')
        draw_omega(self.ax, -0.5, 0, self.omega, 4, color='red')

        v, omega = self.compliance_control(self.v, self.omega, self.Fmag, self.h, np.deg2rad(self.theta))
        v = np.clip(v, -self.v_max, self.v_max)
        omega = np.clip(omega, -self.omega_max, self.omega_max)

        draw_v(self.ax, 0.5, 0, v, v_max=self.v_max, color='green')
        draw_omega(self.ax, 0.5, 0, omega, omega_max=self.omega_max, color='green')

    def compliance_control(self, v_prev, omega_prev, Fmag, h, theta):
        """Compliance Control
        
        F = M \Delta \ddot{x} + C \Delta \dot{x} + K \Delta x
        And set reference to 0 and discretize w/ ZOH
        """

        # Vehicle Parameters
        l = self.l
        r = self.r
        Ts = self.Ts
        C = self.C
        M = self.M
        
        stheta = math.sin(theta)    # Small optimization
        ctheta = math.cos(theta)    # Small optimization

        # Position wrt center of rotatiion
        R = math.sqrt( (r*stheta)**2 + (l + r*ctheta)**2 )
        beta = math.atan2(r * stheta, l)

        sbeta = math.sin(beta)      # Small optimization
        cbeta = math.cos(beta)      # Small optimization
        
        a = ctheta
        b = R*(stheta*cbeta - ctheta*sbeta)
        
        # Admittance Control
        v_eff_prev = (a * v_prev) + (b * omega_prev)

        v_eff_dot = (-Fmag - C*v_eff_prev) / M
        v_eff = v_eff_dot * Ts + v_eff_prev

        # # Calculate new v and omega
        # c_prev = (-b * v_prev) + (a * omega_prev)
        # den = a**2 - b**2
        # v = (a*v_eff - b*c_prev) / den
        # omega = (-b*v_eff + a*c_prev) / den

        # Ensure non-zero 'a' and 'b'
        eps = 0.001
        if (abs(a) < eps):
            return (v_prev, v_eff/b)
        if (abs(b) < eps):
            return (v_eff/a, omega_prev)

        # Calculate new v and omega in parameterized form
        t = self.t     # \in [0,1]
        v = t * v_prev + (1-t) * (v_eff - b*omega_prev) / a
        omega = t * (v_eff - a*v_prev) / b + (1-t) * omega_prev

        return (v, omega)



if __name__ == "__main__":
    sim = Simulation()