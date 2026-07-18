
import threading
import os

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

from cf_funcs import *

if __name__ == "__main__":
    os.system('clear')
    cflib.crtp.init_drivers()

    active_uris = discover_active_uris()

    if not active_uris:
        raise RuntimeError("No drones found")

    param = Param(number_of_drones=len(active_uris), traj=2)

    # init drones
    drone_states = {
        uri: DroneState(param)
        for uri in active_uris
    }
    target = DroneState(param)

    target_lock = threading.Lock()
    chaser_list_ref_lock = threading.Lock()
    stop_event = threading.Event()
    # --------------------------------------------------------
    # Target thread
    # --------------------------------------------------------
    target_thread = threading.Thread(
        target=update_virtual_target,
        args=(target, target_lock, param, stop_event),
        daemon=True
    )

    fib_ref_points = get_fibonacci_points(param.nD, 
                                            param.sphere_radius, 
                                            np.zeros([3,1]))
    min_safe_dist = get_min_safe_distance(fib_ref_points, param) * 0.5

    q_ref_list = []
    for drone in drone_states.values():
        q_s = drone.position - target.position
        norm = np.linalg.norm(q_s)
        if norm > 1e-6:
            q_ref_list.append(q_s / norm)
        else:
            # choose a fallback direction
            q_ref_list.append(np.array([[1.0], [0.0], [0.0]]))

    # --------------------------------------------------------
    # Crazyflies
    # --------------------------------------------------------
    factory = CachedCfFactory(rw_cache="./cache")

    with Swarm(active_uris, factory=factory) as swarm:
        swarm.reset_estimators()

        print("Lights On")
        swarm.parallel_safe(light_check)
        target_thread.start()

        swarm.parallel_safe(
            init_position_getting,
            args_dict={
                uri: (state,)
                for uri, state in drone_states.items()
            }
        )

        swarm.parallel_safe(arm)
        
        swarm.parallel_safe(
            take_off,
            args_dict={
                uri: (state, 
                      param)
                for uri, state in drone_states.items()
            }
        )

        swarm.parallel_safe(
            controller,
            args_dict={
                uri: (
                    target_lock,
                    target,
                    chaser_list_ref_lock,
                    q_ref_list, 
                    fib_ref_points, 
                    min_safe_dist, 
                    active_uris,
                    state,
                    param
                )
                for uri, state in drone_states.items()
            }
        )

        stop_event.set()

        swarm.parallel_safe(
            land,
            args_dict={
                uri: (state, 
                      param)
                for uri, state in drone_states.items()
            }
        )

        swarm.parallel_safe(
            stop_logs,
            args_dict={
                uri: (state,)
                for uri, state in drone_states.items()
            }
        )