# Hierarchical-Nonlinear-Control-for-Swarm-Encirclement-Around-a-Moving-Target-UAV

A MATLAB simulation framework for autonomous multi-UAV swarm encirclement of a moving target using a hierarchical nonlinear control architecture.

The project combines:

- Hierarchical swarm coordination
- Geometric encirclement on a spherical surface
- Dynamic target tracking
- Artificial Potential Fields (APF) for collision avoidance
- Full nonlinear quadrotor dynamics
- Mellinger trajectory tracking controller
- Delaunay-based communication topology

## Overview

This repository implements a centralized swarm control strategy where multiple quadrotors cooperatively surround and follow a moving aerial target while maintaining:

- a desired spherical formation,
- safe inter-agent separation,
- collision avoidance,
- smooth nonlinear flight dynamics.

The target follows a predefined trajectory while each UAV computes its own control actions based on local geometric information.

## Repository Structure

```
.
├── main.m                          % Main simulation script
├── drone_init.m                    % Simulation parameters
├── drone_3dfull_dyn.m              % Quadrotor nonlinear dynamics
├── drone_mellinger_ctrl.m          % Mellinger controller
├── APF_safe_guard.m                % Collision avoidance
├── Delaunay_triangulation_network.m% Communication graph
├── target_motion.m                 % Target dynamics
├── target_ref.m                    % Target trajectory
├── get_fibonacci_points.m          % Sphere reference generation
├── get_min_safe_distance.m         % Safety distance computation
├── projectToSphere.m               % Sphere projection
└── ...                             % Supporting utilities
```


## Control Architecture
The control system is organized into multiple layers:
```
Moving Target
      │
      ▼
Target Position Estimation
      │
      ▼
Reference Sphere Generation
      │
      ▼
Formation Assignment
      │
      ▼
Geometric Formation Controller
      │
      ▼
Artificial Potential Field
      │
      ▼
Mellinger Position Controller
      │
      ▼
Quadrotor Dynamics
```

## Formation Strategy
The swarm surrounds the target by distributing UAVs over a sphere using Fibonacci lattice sampling.

The workflow is:
1. Generate evenly distributed reference points on the sphere.
2. Assign each UAV to its closest reference point using the Hungarian algorithm.
3. Compute geodesic errors.
4. Update the desired formation.
5. Apply collision avoidance using APF.
6. Track the generated references using the nonlinear quadrotor controller.

## Simulation Parameters

Simulation settings are configured in:
```
drone_init.m
```

Main parameters include:

| Parameter | Description |
|-----------|-------------|
| `Param.nD` | Number of drones |
| `Param.Tend` | Simulation duration |
| `Param.dTi` | Simulation timestep |
| `Param.sphere_radius` | Encirclement radius |
| `Param.ref_target_traj` | Target trajectory |
| `Param.k_geometric` | Formation control gain |
| `Param.k_apf` | APF gain |
| `Param.k_rad` | Radius reduction gain |

## Running the Simulation
Open MATLAB, navigate to the repository directory, and run:

```matlab
main.m
```

## Visualization

The simulator supports:
- 3D swarm animation
- Target trajectory visualization
- Formation evolution
- Radial distance plots
- Video export (MP4) (Might not work)
- PDF figure generation

Visualization options can be configured in `drone_init.m`.
```matlab
Param.animation.(params)
```
## Requirements

- MATLAB (was only tested in R2024a)
- No additional toolboxes are required beyond standard MATLAB functionality.

## Citation

If you use this repository in your research, please cite the associated publication (not yet availabe).

```bibtex
@article{NOTYETAVAILABLE,
  title={Hierarchical Nonlinear Control for Swarm Encirclement Around a Moving Target UAV},
  author={Filipe Cavalheiro, Bruno Guerreiro},
  year={2026}
}
```
---

## License

This work uses the AGPL license more information about the same can be viewed at `LICENCSE`.

## Future Work

Potential extensions include:

- Decentralized estimation
- Obstacle avoidance
- Time-varying communication networks
- Hardware-in-the-loop simulation
- ROS 2 integration
- PX4 implementation
- Real-world flight experiments
