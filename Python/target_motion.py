import numpy as np
from Structs import DroneState, Param
from drone_3dfull_dyn import drone_3dfull_dyn
from helper_funcs import *
from drone_mellinger_ctrl import drone_mellinger_ctrl

def target_motion(target: DroneState, param: Param):
    """
    Propagates the target drone one timestep.

    Parameters
    ----------
    target : dict
        Target state with keys:
            - "pos" : (3,1)
            - "vel" : (3,1)
            - "attitude" : (3,3) rotation matrix
            - "ang_vel" : (3,1)
    p_d : ndarray
        Desired position (3,1)
    v_d : ndarray
        Desired velocity (3,1)
    Param : dict
        Simulation parameters.
    """

    # Extract state
    pos = target.position
    vel = target.velocity
    R = target.attitude.reshape(3,3)
    om = target.ang_vel

    # Desired quantities (trajectory tracking)
    dpsi_d = (target.desired.psi - target.desired.old_psi)/param.sim.dTi
    target.desired.old_psi = target.desired.psi

    # Controller
    thrust, torque = drone_mellinger_ctrl(
        target,
        dpsi_d, 
        param
    )

    # Drone dynamics
    dot_p, dot_v, dot_om = drone_3dfull_dyn(
        vel,
        R,
        om,
        thrust,
        torque,
        param,
    )

    # Euler integration
    dt = param.sim.dTi

    target.position = pos + dt * dot_p
    target.velocity = vel + dt * dot_v
    target.attitude = rot_integrate(R, om, dt).reshape(-1,1)
    target.ang_vel = om + dt * dot_om