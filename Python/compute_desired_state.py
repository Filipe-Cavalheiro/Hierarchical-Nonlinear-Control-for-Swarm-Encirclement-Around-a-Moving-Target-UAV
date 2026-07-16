import numpy as np
from voronoi_mesh_funcs import pos_based_mesh_transform, runCVT
from Structs import Voronoi, Param, DroneState

def compute_desired_state(
    k,
    iD,
    origin_p_d,
    voronoi: Voronoi,
    chaser: DroneState,
    neighbours,
    param: Param,
):
    # Extract target state
    target_pos = chaser.estimated.target_position
    if param.sim.measurable_states == 2:
        target_vel = chaser.estimated.target_velocity
    else:
        target_vel = np.zeros(3)

    if param.sim.distribution == "fibonacci":
        chaser.desired.position = origin_p_d[:,iD:iD+1] + target_pos
        chaser.desired.velocity = target_vel

    elif param.sim.distribution == "dijkstra":
        (voronoi.vertices, voronoi.faces, voronoi.mesh_pos
        ) = pos_based_mesh_transform(target_pos, voronoi, param)

        if k % 100 == 0:
            sites = [voronoi.sites[i] for i in [iD,*neighbours.tolist()]]
            voronoi.newSite[iD] = runCVT(voronoi, sites)

        chaser.desired.position = voronoi.vertices[voronoi.sites[iD], :].reshape(-1,1)

        # MATLAB compares a vector with zero here. Assuming the intent is to
        # test the x-coordinate.
        if target_pos[0] >= 0:
            chaser.desired.velocity = target_vel
        else:
            chaser.desired.velocity = np.array(
                [0.0, target_vel[1,0], target_vel[2,0]]
            ).reshape(-1,1)

    else:
        raise ValueError(
            f"Unknown distribution type: {param.sim.distribution}"
        )

    return voronoi