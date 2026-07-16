import numpy as np
from helper_funcs import *
from Structs import Param

def drone_3dfull_dyn(v, R, om, T, tau, param: Param):
    """
    Full 3D drone dynamics.

    Parameters
    ----------
    v : np.ndarray
        Velocity in world frame (3,)
    R : np.ndarray
        Rotation matrix body -> world (3,3)
    om : np.ndarray
        Angular velocity in body frame (3,)
    T : float
        Thrust magnitude
    tau : np.ndarray
        Body torque (3,)
    param : object
        Drone parameters

    Returns
    -------
    dp : np.ndarray
        Position derivative
    dv : np.ndarray
        Velocity derivative
    dR : np.ndarray
        Rotation matrix derivative
    dom : np.ndarray
        Angular velocity derivative
    """

    # Auxiliary variables
    zW = np.array([[0.0], [0.0], [1.0]])
    zB = R[:, 2].reshape(3,1)

    # Equations of motion
    dp = v

    dv = (
        -param.sim.gravity * zW
        + (T / param.sim.drone_mass) * zB
        - (param.sim.drag*R) @ R.T @ v
    )

    dom = np.linalg.inv(param.sim.J) @ ( -skew(om) @ param.sim.J @ om + tau)

    return dp, dv, dom