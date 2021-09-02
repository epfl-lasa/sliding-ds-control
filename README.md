# sliding-ds-control

Repository with controller in Python and Matlab for Passsive Dynamical Systems control for mobile robots applications in 2D navigation with compliance repsonse to impact and Sliding response for advancing around pedestrians/obstacles.

<p align="center">
<img src="https://github.com/epfl-lasa/sliding-ds-control/blob/main/images/Complaint_DS.png"  width="750"></>

<p align="center">
<img src="https://github.com/epfl-lasa/sliding-ds-control/blob/main/images/controller-qolo-compliant.png"  width="750"></>

Work published in:

[1] Paez-Granados D., Gupta V. and Billard, A. “Unfreezing Social Navigation: Dynamical Systems based Compliance for Contact Control in Robot Navigation”. 2022. (Under review)

-------------
Requirements:
```
Scripts for 2D simulation require matlab2020+
Alternatively 3D simulaiton is available through pybullet

Setup:
git clone https://github.com/epfl-lasa/sliding-ds-control.git

```
-------------

## Repository Structure


### Data

data/ :

### Scripts

scripts/ : 


### Visualization

figures/ : 

images/ : 


### Controller

passive_DS/ : 



-------------

# Experimental Setup

<p align="center">
<img src="https://github.com/epfl-lasa/sliding-ds-control/blob/main/images/qolo_experiments.png"  width="750"></>


<p align="center">
<img src="https://github.com/epfl-lasa/sliding-ds-control/blob/main/images/qolo_schematic.png"  width="350"></>


-------------
## Related packages:

[P1] Main ROS controller for Qolo-robot
https://github.com/DrDiegoPaez/qolo_ros

[P2] Obstacle avoidance for tight shape and non-holonomic constraints (used in shared control) [4] 

https://github.com/epfl-lasa/rds

[P3] Obstacle avoidance based on dynamical systems [5]:
https://github.com/epfl-lasa/qolo_modulation
https://github.com/epfl-lasa/dynamic_obstacle_avoidance/


## References:

> [1] Paez-Granados D., Gupta V. and Billard, A. “Unfreezing Social Navigation: Dynamical Systems based Compliance for Contact Control in Robot Navigation”. 2022. (Under review)


### Qolo Design:

> [2] Paez-Granados, D. F., Kadone, H., & Suzuki, K. (2018). Unpowered Lower-Body Exoskeleton with Torso Lifting Mechanism for Supporting Sit-to-Stand Transitions. IEEE International Conference on Intelligent Robots and Systems, 2755–2761. https://doi.org/10.1109/IROS.2018.8594199

### Qolo Hands-free control:

> [3] Chen, Y., Paez-Granados, D., Kadone, H., & Suzuki, K. (2020). Control Interface for Hands-free Navigation of Standing Mobility Vehicles based on Upper-Body Natural Movements. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS-2020). https://doi.org/10.1109/IROS45743.2020.9340875

### Qolo shared control:

> [4] Gonon, D. Paez-Granados, D., Billard, A. (2021). Reactive Controller for a Convex Non-holonomic Robot to Travel in Crowds. IEEE Robotics and Automation Letters (IEEE-RAL).

### Obstacle avoidance through modulated-DS:

> [5] Huber, Lukas, Aude Billard, and Jean-Jacques E. Slotine. (2019) "Avoidance of Convex and Concave Obstacles with Convergence ensured through Contraction." IEEE Robotics and Automation Letters (IEEE-RAL).


**Contact**: 
[Dr. Diego Paez]
https://diegofpaez.wordpress.com/

**Acknowledgments**
This project was partially founded by:

> The EU Horizon 2020 Project CROWDBOT (Grant No. 779942): http://crowdbot.eu

> The Toyota Mobility Foundation (TMF) through the Grant: Mobility Unlimited Challenge 2019: https://mobilityunlimited.org
