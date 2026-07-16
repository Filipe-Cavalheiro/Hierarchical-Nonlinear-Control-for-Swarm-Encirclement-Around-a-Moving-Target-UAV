import numpy as np
from scipy.integrate import solve_ivp

from drone_mellinger_ctrl import drone_mellinger_ctrl
from drone_3dfull_dyn import drone_3dfull_dyn
from helper_funcs import rot_integrate
from Structs import DroneState, Param


# Persistent variable replacement
_old_psi_d = 0.0


def actuate_control(chaser: DroneState, param: Param):
    global _old_psi_d

    if param.controller.selected_controller == "Mellinger":
        pos = chaser.position
        vel = chaser.velocity
        attitude = chaser.attitude.reshape((3, 3))
        ang_vel = chaser.ang_vel

        # Reference yaw rate
        dpsi_d = (chaser.desired.psi - _old_psi_d) / param.sim.dTi
        _old_psi_d = chaser.desired.psi

        # Mellinger controller
        thrust, torque = drone_mellinger_ctrl(
            chaser,
            dpsi_d,
            param
        )

        # Continuous-time drone dynamics
        dot_p, dot_v, dot_om = drone_3dfull_dyn(
            vel,
            attitude,
            ang_vel,
            thrust,
            torque,
            param,
        )

        # Euler discretization
        chaser.position = pos + param.sim.dTi * dot_p
        chaser.velocity = vel + param.sim.dTi * dot_v

        Rp = rot_integrate(
            attitude,
            ang_vel,
            param.sim.dTi,
        )

        chaser.attitude = Rp.reshape(-1,1)

        chaser.ang_vel = (
            ang_vel + param.sim.dTi * dot_om
        )

    else:
        raise ValueError(
            f"Unknown controller type: {param.controller}"
        )

    return chaser