# Source code for running on robot

Python code for sliding-ds implementation on QOLO using ROS.

-------------

## Code Structure

- `main.py`
: Demo file for running control ROS node using compliance controller

- `bumper_prediction.py`
: Demo file for running bumper prediction ROS node

- `plot_force_bumper.py` 
: Demo file for plotting real-time bumper contact

- `compliance_controller`
: Python module implementing various controllers
    <br> $\rightarrow$ `admittance.py` : Admittance controller
    <br> $\rightarrow$ `passive_ds.py` : Sliding passive ds controller

- `prediction_model`
: Prediction models for contact force on the bumper surface
    <br> $\rightarrow$ `bumper_model.py` : Main bumper model class
    <br> $\rightarrow$ `svr.py` : SVR
    <br> $\rightarrow$ `nn.py` : RNN

-------------

## Passive-DS Compliance Parameters

| Controller Parameters | Name                  | Description                                                                                             |
|-----------------------|-----------------------|---------------------------------------------------------------------------------------------------------|
| $T_s$                 | Integration Time      | Sampling time for the controller dynamics \[s\] <br> _Default_: Same as control loop frequency (50 Hz)  |
| $M$                   | Robot Mass            | Virtual mass of the robot \[kg\] <br> _Default_: 2 kg                                                   |
| $\lambda_t$           | Tangential damping    | Damping coefficient over tangential direction of the collision surface <br> _Default_: 0                |
| $\lambda_n$           | Normal damping        | Damping coefficient over the normal direction of the collision surface <br> _Default_: 0.5              |
| $F_n$                 | Contact Force Limit   | Maximum contact force bounded by safety and acceptability \[N\] <br> _Default_: 45 N                    |
| $F_a$                 | Activation Force      | Activation contact force to overcome false positives \[N\] <br> _Default_: 15 N                         |
| $\dot{\xi}_{max}$     | Max speed at contact  | Maximum allowed sliding speed at contact \[m/s\] <br> _Default_: 0.5 m/s                                |

