import time
import threading
import numpy as np
from scipy.optimize import linear_sum_assignment

from Structs import *
from fiboancci_funcs import *
from helper_funcs import *

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie import Crazyflie

def uri_is_alive(uri, timeout=0.5):
    """
    Try connecting briefly to check if a Crazyflie exists.
    """
    try:
        cf = Crazyflie(rw_cache='./cache')

        with SyncCrazyflie(uri, cf=cf) as scf:
            # If we get here, connection succeeded
            return True

    except Exception:
        return False


def discover_active_uris():
    active = []

    for uri in CANDIDATE_URIS:
        print(f"Probing {uri} ...")
        if uri_is_alive(uri):
            print(f"  ✔ Found: {uri}")
            active.append(uri)
        else:
            print(f"  ✖ Not found: {uri}")

    return active

def stop_logs(scf, state: DroneState):
    state.log_conf.stop()
    
def get_position_callback(data, state: DroneState):
    state.position[0] = data['stateEstimate.x']
    state.position[1] = data['stateEstimate.y']
    state.position[2] = data['stateEstimate.z']

def init_position_getting(scf: SyncCrazyflie, state: DroneState):
    log_conf = LogConfig(name='Position', period_in_ms=500)
    log_conf.add_variable('stateEstimate.x', 'float')
    log_conf.add_variable('stateEstimate.y', 'float')
    log_conf.add_variable('stateEstimate.z', 'float')
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(lambda _timestamp,
                    data,
                    _logconf: get_position_callback(data, state))
    log_conf.start()

    state.log_conf = log_conf
    
def light_check(scf: SyncCrazyflie):
    scf.cf.param.set_value('led.bitmask', 255)
    time.sleep(2)
    scf.cf.param.set_value('led.bitmask', 0)

def arm(scf: SyncCrazyflie):
    scf.cf.supervisor.send_arming_request(True)
    time.sleep(1.0)

def take_off(scf, state: DroneState, param: Param):
    cf = scf.cf
    deadline = time.time() + param.deadline_time
    while state.position[2] < param.default_height*0.9:
        if time.time() > deadline:
            land(scf, state, param)
            raise RuntimeError("Takeoff timeout")
        cf.commander.send_position_setpoint(state.position[0, 0], state.position[1, 0], param.default_height, 0)
        time.sleep(param.dTi)

def land(scf: SyncCrazyflie, state: DroneState, param: Param):
    cf = scf.cf
    x, y = state.position[0, 0], state.position[1, 0]

    deadline = time.time() + param.deadline_time
    while state.position[2] > 0.12 and time.time() < deadline:
        cf.commander.send_position_setpoint(x, y, 0.1, 0)
        time.sleep(param.dTi)

    cf.commander.send_position_setpoint(x, y, 0.0, 0)
    time.sleep(0.3)

    cf.commander.send_notify_setpoint_stop()

def update_virtual_target(
    target: DroneState,
    lock: threading.Lock,
    param: Param,
    stop_event: threading.Event
):
    start_time = time.time()

    while not stop_event.is_set():

        t = time.time() - start_time

        pos = param.ref_target_traj["pos"](t)

        with lock:
            target.position[:] = pos

        time.sleep(param.dTi)

def controller(
    scf: SyncCrazyflie,
    target_lock: threading.Lock,
    target: DroneState, 
    chaser_list_ref_lock: threading.Lock,
    chaser_list_ref, 
    fib_ref_points, 
    min_safe_dist, 
    active_uris,
    state: DroneState,
    param: Param
):
    """
    Runs continuously on each CF.
    """
    cf = scf.cf
    iD = active_uris.index(cf.link_uri)
    end_time = time.time() + param.Tend
    while time.time() < end_time:
        with target_lock:
            target_pos = np.array([
                target.position[0],
                target.position[1],
                target.position[2]
            ]).reshape(-1,1)
        
        with chaser_list_ref_lock:
            q_ref_list = chaser_list_ref.copy()

        chaser_pos = state.position.reshape(-1,1)
        q_s = chaser_pos - target_pos
        state.radii_to_target = np.linalg.norm(q_s)
        if state.radii_to_target < 1e-8:
            continue
        
        q_s = chaser_pos - target_pos
        # Desired reduced-attitude direction: fibonacci point relative to the
        # target, projected onto the unit sphere (q* = (p* - p_T)/||p* - p_T||)
        q_d = fib_ref_points[iD, :].reshape(3,1)
        q_d = q_d / np.linalg.norm(q_d)

        # Position reference generation: integrate the guidance state (eq. (11))
        q_ref_list[iD], e_q = linear_motion_ref(q_ref_list[iD], q_d, param)

        # Safe guard collisions: APF displacement applied to the guidance
        # references (eq. (14)), so references deflect around each other
        q_ref_list[iD] = apf_safe_guard(iD, q_ref_list[iD], q_ref_list,
                                        min_safe_dist, param)
        q_hat = q_ref_list[iD]

        with chaser_list_ref_lock:
            chaser_list_ref[iD] = q_ref_list[iD]

        # Update sphere radius reference (integrates its own state, eq. (15))
        state.desired.radii_to_target = update_radius(state.desired.radii_to_target, e_q, param)

        # ---------- Enforce S² constraint (local) ----------
        goal = state.desired.radii_to_target * q_hat + target_pos
        cf.commander.send_position_setpoint(
            float(goal[0, 0]),
            float(goal[1, 0]),
            float(goal[2, 0]),
            0.0
        )

        time.sleep(param.dTi)
  