import numpy as np

from Structs import DroneState, Param

def estimate_target_variables(
    target: DroneState,
    chaser: DroneState,
    ema_estimated,
    idx,
    connection_laplacian,
    param: Param,
):
    """
    Estimate target position using an exponential moving average and
    a consensus update.

    Parameters
    ----------
    target : object
        Must have attribute `position` (3,1).
    ema_estimated : ndarray
        Array of shape (3, N) containing EMA estimates.
    idx : int
        Column index corresponding to this agent.
    connection_laplacian : ndarray
        Graph Laplacian matrix (N x N).
    param : object
        Must have attributes:
            noise_std
            ema_alpha
            dTi

    Updates
    -------
    estimated_target_position : ndarray
        Estimated 3D target position for agent idx.
    ema_estimated : ndarray
        Updated EMA estimates.
    """

    # Noisy target measurement
    noisy_target_pos = target.position + (param.noise_std * np.random.randn(3)).reshape(-1,1)
    noisy_target_vel = target.velocity + (param.noise_std * np.random.randn(3)).reshape(-1,1)
    
    # Exponential Moving Average (EMA)
    ema_estimated[:, idx:idx+1] = np.concatenate(
        [(1.0 - param.ema_alpha) * ema_estimated[0:3, idx:idx+1]
        + param.ema_alpha * noisy_target_pos,
        (1.0 - param.ema_alpha) * ema_estimated[3:6, idx:idx+1]
        + param.ema_alpha * noisy_target_vel]
    )

    # Consensus update
    estimated_target_variables = (
        ema_estimated[:, idx:idx+1]
        - param.sim.dTi
        * ema_estimated @ connection_laplacian[:, idx:idx+1]
    )

    chaser.estimated.target_position = estimated_target_variables[0:3]
    chaser.estimated.target_velocity = estimated_target_variables[3:6]

    return ema_estimated