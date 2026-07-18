# Crazy Flies

A Python-based multi-**Crazyflie** swarm controller that discovers available drones, takes off as a synchronized swarm, tracks a moving virtual target, and maintains a distributed formation using Fibonacci sphere reference points with geometric control and collision avoidance.

## Features

* Automatic discovery of active Crazyflie drones
* Multi-drone swarm coordination using the Crazyflie Python library
* Virtual moving target tracking
* Formation generation using Fibonacci sphere sampling
* Artificial Potential Field (APF) collision avoidance
* Real-time position logging from the onboard state estimator
* Automated takeoff and landing
* Parallel execution using Python threads and the Crazyflie Swarm API

## Project Structure

```text
Crazy_Flies/
│
├── main.py                 # Main application entry point
├── cf_funcs.py             # Crazyflie communication and control
├── helper_funcs.py         # Math and controller helper functions
├── fiboancci_funcs.py      # Fibonacci sphere generation utilities
├── Structs.py              # Parameters and state definitions
└── requirements.txt        # Python dependencies
```

## Requirements
### Hardware

* One or more Bitcraze Crazyflie drones
* Crazyradio PA USB dongle
* A positioning system compatible with the Crazyflie state estimator (this project was tested using the Bitcraze Lighthouse positioning system)
* Python 3.12.3 (the project has only been tested with this version)

### Software

Install the required packages:

Linux/macOS: 
```bash
# Create a virtual environment (first time only)
python3 -m venv .venv

# Activate the virtual environment
# macOS/Linux
source .venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Run the experiment
python3 main.py
```

Windows (Command Prompt):
```bash
# Create a virtual environment (first time only)
python3 -m venv .venv

# Windows (Command Prompt)
venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Run the experiment
python3 main.py
```

Windows (PowerShell):
```bash
# Create a virtual environment (first time only)
python3 -m venv .venv

# Windows (PowerShell)
venv\Scripts\Activate.ps1

# Install dependencies
pip install -r requirements.txt

# Run the experiment
python3 main.py
```

The project depends primarily on:

* cflib
* NumPy
* SciPy

## Drone Discovery

The system probes every URI listed in `CANDIDATE_URIS` and only includes drones that successfully establish a connection.

> [!NOTE]
> The discovery routine assumes that drone URIs follow the pattern
> `radio://0/10/2M/E7E7E7E70*`.
> If your drones use different URIs, either modify `discover_active_uris()` or reconfigure the drone addresses accordingly.
 
Example configuration for a swarm of five drones:
```text
radio://0/10/2M/E7E7E7E701
radio://0/10/2M/E7E7E7E702
radio://0/10/2M/E7E7E7E703
radio://0/10/2M/E7E7E7E704
radio://0/10/2M/E7E7E7E705
```

## Formation Control

Each drone attempts to occupy one point on a virtual sphere surrounding the moving target.

The controller combines several components:

* **Target tracking**
* **Geometric reduced-attitude control**
* **Fibonacci sphere reference generation**
* **Optimal assignment between drones and reference points**
* **Artificial Potential Field (APF) collision avoidance**

This allows the swarm to continuously surround the moving target while maintaining safe spacing.

> [!WARNING]
> Due to the properties of the Fibonacci sphere sampling algorithm, the swarm size **must be an odd number**.

## Target Trajectory

The target is virtual and runs in its own thread.

Its position is updated periodically using the trajectory defined inside the `Param` class.

Changing the target behavior only requires modifying the trajectory generation function.

## Configuration

Most parameters are centralized inside `Structs.py` under the `Param` class.

Important parameters include:

| Parameter            | Description               |
| -------------------- | ------------------------- |
| `Tend`               | Controller duration       |
| `dTi`                | Control loop period       |
| `default_height`     | Takeoff altitude          |
| `sphere_radius`      | Formation radius          |
| `target_traj_radius` | Radius of target motion   |
| `k_rad`              | Radial controller gain    |
| `k_geometric`        | Geometric controller gain |
| `k_apf`              | Collision avoidance gain  |

> [!NOTE]  
> * All drones must use compatible radio channels and URIs defined in `CANDIDATE_URIS`.
> * Position estimates must be available before takeoff.
> * The controller assumes reliable position feedback throughout the flight.
> * If no drones are detected, execution stops with an error.

## Running

After installing the dependencies and activating the virtual environment, start the experiment with:

```bash
python3 main.py
```

## Future Improvements

Potential extensions include:

* Obstacle-aware trajectory planning
* Dynamic target tracking from external sensors
* Adaptive formation resizing
* Fault tolerance for drone loss
* ROS 2 integration
* Simulation support (e.g., PyBullet or Webots)
* Visualization of swarm states in real time

## License

The project is licensed under the terms of the license included in the repository root.

## Acknowledgements

This project is built using the excellent **Bitcraze Crazyflie Python Library (`cflib`)** and implements distributed swarm formation control through geometric control, Fibonacci sphere sampling, and Artificial Potential Field (APF) collision avoidance.
