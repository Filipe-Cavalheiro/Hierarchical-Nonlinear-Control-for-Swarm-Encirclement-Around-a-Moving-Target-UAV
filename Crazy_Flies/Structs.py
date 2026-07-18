import numpy as np
import math
from dataclasses import dataclass, field
from typing import Optional

from cflib.crazyflie.log import LogConfig

# CONSTS
BASE_URI = "radio://0/10/2M/E7E7E7E70"
CANDIDATE_URIS = [
    BASE_URI + f"{i}"
    for i in range(1, 6)   # 01 → 05
]
GOLDEN_ANGLE = math.pi * (3 - math.sqrt(5))

@dataclass
class EstimatedState:        
    target_position: np.ndarray = field(default_factory=lambda: np.zeros((3,1)))
    target_velocity: np.ndarray = field(default_factory=lambda: np.zeros((3,1)))

@dataclass
class DesiredState:
    position: np.ndarray = field(default_factory=lambda: np.zeros((3,1)))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros((3,1)))
    acceleration: np.ndarray = field(default_factory=lambda: np.zeros((3,1)))
    jerk: np.ndarray = field(default_factory=lambda: np.zeros((3,1)))
    snap: np.ndarray = field(default_factory=lambda: np.zeros((3,1)))
    psi : float = 0.0
    old_psi: float = 0.0
    b1: np.ndarray = field(default_factory=lambda: np.array([1.0,0.0,0.0]).reshape(-1,1))
    b1_dot: np.ndarray = field(default_factory=lambda: np.zeros((3,1)))
    b1_2dot: np.ndarray = field(default_factory=lambda: np.zeros((3,1)))
    radii_to_target: np.floating = np.float32(0.0)


class MotionConstraints:
    def __init__(self, param):
        # params for obstacle avoidance/ vel and acc constraints 
        self.obstacle_center = np.array([0, 2.0, 1.0])
        self.obstacle_radius = np.diag([1.5, 1.5, 1.5])

        self.q: np.ndarray = np.zeros([(param.n_degree+1)*3,1])
        self.b_obs: np.ndarray = np.zeros((3 * param.horizon, 1))
        self.A_obs: np.ndarray = np.zeros((3 * param.horizon, (param.n_degree+1)*3))
        self.s: Optional[np.ndarray] = None
        self.lambda_obs: Optional[np.ndarray] = None
        self.lambda_ineq: Optional[np.ndarray] = None
        self.C: Optional[np.ndarray] = None

class DroneState:
    def __init__(self, param):
        self.position: np.ndarray = np.zeros((3,1))
        self.velocity: np.ndarray =  np.zeros((3,1))
        self.acceleration: np.ndarray =  np.zeros([3,1], dtype=np.float64)
        self.attitude: np.ndarray =  np.eye(3, dtype=np.float64).reshape(-1,1)
        self.ang_vel: np.ndarray =  np.zeros([3,1], dtype=np.float64)
        self.ei: np.ndarray =  np.zeros([3,1], dtype=np.float64)
        self.eI: np.ndarray =  np.zeros([3,1], dtype=np.float64)
        self.radii_to_target: np.floating = np.float32(0.0)

        self.estimated: EstimatedState = EstimatedState()
        self.desired: DesiredState = DesiredState()
        self.constraints: MotionConstraints = MotionConstraints(param)


class Param:
    def __init__(self, number_of_drones: int, traj:int =2):
        # ----------------------------
        # Simulation
        # ----------------------------
        self.Tend = 30
        self.dTi = 0.1

        self.nD = number_of_drones
        self.sphere_radius = 0.4        # (m)

        self.target_traj_radius = 1.5  # (m)
        self.default_speed = 0.5        # Default speed (m/s)
        self.default_height = 1.3       # Default takeoff altitude (m)
        self.deadline_time = 10

        # Fibonacci controller gains
        self.k_rad = 0.3
        self.k_geometric = 5
        self.k_apf = 1

        # constraints
        self.n_degree = 5
        self.horizon = 20       # horizon
        self.obs_horizon = 5       # horizon
        self.rho_obs = 100.0      # Obstacle avoidance penalty
        self.rho_ineq = 20.0      # max/min vel and acc penalty
        self.max_iter = 100     # max iterations per update
        self.w_goal = 1000.0    # gain to reduce dist between agent and goal
        self.w_smooth = 0.1     # gain to reduce high accelerations
        self.v_max = 10          # max velocity constraint 
        self.a_max = 50          # max acceleration constraint 

        # Trajectory
        self.ref_target_traj = self._make_traj(traj)
        self.initial_target_pos = self.ref_target_traj["pos"](0)
        

    # ----------------------------
    # Trajectory generator
    # ----------------------------
    def _make_traj(self, traj):
        if traj == 1:
            return {
                "pos": lambda t: np.array([0, 0, 5]),
                "vel": lambda t: np.array([0, 0, 0]),
            }

        if traj == 2:
            return {
                "pos": lambda t: np.array([
                     [self.target_traj_radius * np.sin(self.default_speed * t)],
                     [self.target_traj_radius * np.cos(self.default_speed * t)],
                     [self.default_height]
                ])
            }

        if traj == 3:
            return {
                "pos": lambda t: np.array([
                    5 * np.sin(t),
                    5 * np.sin(2 * t),
                    5
                ]),
                "vel": lambda t: np.array([
                    5 * np.cos(t),
                    10 * np.cos(2 * t),
                    0
                ]),
            }

        if traj == 4:
            return {
                "pos": lambda t: np.array([
                    5 * np.sin(t),
                    5 * np.sin(2 * t),
                    5 + 0.2 * np.cos(2 * t),
                ]),
                "vel": lambda t: np.array([
                    5 * np.cos(t),
                    10 * np.cos(2 * t),
                    -0.4 * np.sin(2 * t),
                ]),
            }

        raise ValueError("Unknown trajectory")

class Voronoi:
    def __init__(self, param: Param):
        self.mesh_pos = np.array(None)
        self.surf_direction = [1, 0, 0]
        self.v_circle = np.array(None)
        self.v_cone = np.array(None)
        self.v_sphere = np.array(None)
        self.vertices = np.array(None)
        self.faces = np.array(None)
        self.sites = np.array(None)
        self.newSite = np.zeros((1, param.nD))
        self.ran_u2_eq_1 = False

class MeshVertex:
    def __init__(self, idx, pos):
        self.idx = idx
        self.pos = np.array(pos, dtype=float)

        self.dist = np.inf
        self.origin_site = -1
        self.prev = -1

        self.neigh = []
        self.neighW = []