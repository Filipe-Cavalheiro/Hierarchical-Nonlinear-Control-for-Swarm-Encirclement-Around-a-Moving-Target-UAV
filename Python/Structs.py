import numpy as np
from numpy.typing import NDArray
from typing import Optional
import math
from dataclasses import dataclass, field

# CONSTS
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
    
class SimulationConfig:
    def __init__(self):
        # Simulation
        self.Tend = 30
        self.dTi = 0.01
        self.Nsim = round(self.Tend / self.dTi) + 1

        self.t = np.arange(
            0,
            self.Tend + self.dTi,
            self.dTi
        )

        self.gravity = 9.81

        # Drones
        self.nD = 3
        self.sphere_radius = 2.0

        # Drop parameters
        self.nD_drop = 0
        self.nD_drop_time = 500

        # Quadrotor
        self.drone_mass = 5
        self.J = np.diag([
            2e-2,
            2e-2,
            3e-2
        ])

        self.drag = 0.0
        self.ctf = 0.0135

        # Measurements
        # 1: position
        # 2: position + velocity
        self.measurable_states = 2

        # Formation/distribution
        # options: fibonacci, dijkstra 
        self.distribution = "dijkstra"

class ControllerConfig:
    def __init__(self):
        self.selected_controller = "Mellinger"

        self.Mellinger = {
            "kp": np.diag([50, 50, 50]),
            "kv": np.diag([15, 15, 15]),
            "kR": np.diag([8, 8, 8]),
            "kom": np.diag([0.5, 0.5, 0.5]),
        }

        self.Lee = {
            "x": 10,
            "v": 8,
            "i": 10,
            "R": 1.5,
            "W": 0.35,
            "I": 10,
            "y": 0.8,
            "wy": 0.15,
            "yI": 2,
        }

        # Consensus parameters
        self.c1 = 1.5
        self.sigma = 10
        self.c2 = 2
        self.c3 = 2

class TrajectoryConfig:
    def __init__(
        self,
        traj=2,
        radius=2.2,
        speed=0.5,
        height=1.3
    ):

        self.target_traj_radius = radius
        self.default_speed = speed
        self.default_height = height

        self.ref_target_traj = self.make_traj(traj)

        self.initial_target_pos = (
            self.ref_target_traj["pos"](0)
        )


    def make_traj(self, traj):
        if traj == 1:
            # Hover
            return {
                "pos": lambda t: np.array([
                    0,
                    0,
                    5
                ]),

                "vel": lambda t: np.zeros(3)
            }


        elif traj == 2:
            # Half-circle, then stationary at the end of the path
            vel_gain = 0.25

            return {
                "pos": lambda t: np.array([
                    [-5 + 15*np.sin(vel_gain*t)],
                    [15*np.cos(vel_gain*t)],
                    [5]
                ]),

                "vel": lambda t: np.array([
                    [15*vel_gain*np.cos(vel_gain*t)],
                    [-15*vel_gain*np.sin(vel_gain*t)],
                    [0]
                ])
            }


        elif traj == 3:
            # Lissajous
            return {
                "pos": lambda t: np.array([
                    5*np.sin(t),
                    5*np.sin(2*t),
                    5
                ]),

                "vel": lambda t: np.array([
                    5*np.cos(t),
                    10*np.cos(2*t),
                    0
                ])
            }


        elif traj == 4:
            # 3D Lissajous
            return {
                "pos": lambda t: np.array([
                    5*np.sin(t),
                    5*np.sin(2*t),
                    5+0.2*np.cos(2*t)
                ]),

                "vel": lambda t: np.array([
                    5*np.cos(t),
                    10*np.cos(2*t),
                    -0.4*np.sin(2*t)
                ])
            }


        else:
            raise ValueError(
                f"Unknown trajectory {traj}"
            )

class GraphicalDisplay:
    def __init__(self, dTi):
        self.verbose = True
        self.composed = True
        self.radial_hist = True
        self.consensus = False
        self.save_to_mp4 = True
        self.animate = True
        self.store_data = True

        self.save_location = "Results"
        self.trail_length = 100
        self.step = 5

        # self.animation = {
        #     "create_pdf": True,
        #     "save_to_mp4": False,
        #     "frameStep": 10,
        # }

        # self.sim_dt = (
        #     dTi *
        #     self.animation["frameStep"]
        # )

        # self.videoFPS = 1 / self.sim_dt

@dataclass
class SimulationData:
    target_true_hist: list
    removed_initial_pos: bool
    colors: np.ndarray

    position_history: NDArray[np.float64]
    p_d_hist: NDArray[np.float64]
    radial_history: NDArray[np.float64]

    def __post_init__(self):
        assert self.position_history.ndim == 3
        assert self.position_history.shape[1] == 3

        assert self.p_d_hist.ndim == 3
        assert self.p_d_hist.shape[1] == 3

class Param:
    def __init__(
        self
    ):
        self.sim = SimulationConfig()

        self.controller = ControllerConfig()

        self.traj = TrajectoryConfig()

        self.display = GraphicalDisplay(
            self.sim.dTi
        )

        # Disturbance
        self.x_delta = np.array([
            0.5,
            0.8,
            -1.0
        ])

        self.R_delta = np.array([
            0.2,
            1.0,
            -0.1
        ])

        self.k_rad = 0.3
        self.k_geometric = 5
        self.k_apf = 1

        # Target estimation
        self.ema_alpha = 0.2
        self.noise_std = 0.05

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

class Voronoi:
    def __init__(self, param: Param):
        self.mesh_pos = np.array(None)
        self.surf_direction = np.array([1.0, 0.0, 0.0]).reshape(-1, 1)
        self.v_circle = np.array(None)
        self.v_cone = np.array(None)
        self.v_sphere = np.array(None)
        self.vertices = np.empty((0, 3))
        self.faces = np.empty((0, 3), dtype=int)
        self.sites = np.array((1, param.sim.nD))
        self.newSite = np.zeros((1, param.sim.nD))
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