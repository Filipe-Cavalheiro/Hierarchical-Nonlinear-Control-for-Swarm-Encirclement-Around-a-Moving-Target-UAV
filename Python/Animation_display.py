import numpy as np
import matplotlib.pyplot as plt
import os
import plotly.graph_objects as go

from Structs import Param, SimulationData

def create_figures(data: SimulationData, param: Param):
    if param.display.composed is True:
        composed_fig(data, param)

    if param.display.radial_hist is True:
        radial_hist_plot(data, param)

    if param.display.consensus is True:
        consensus_plot(data, param)

    # if param.display.save_to_mp4 is True:
    #     import os
    #     import cv2

    #     mp4_file = os.path.join(
    #         os.getcwd(),
    #         "control_sphere_v6_laypunov_example_3.mp4"
    #     )

    #     # Initialize video writer
    #     fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    #     v = cv2.VideoWriter(
    #         mp4_file,
    #         fourcc,
    #         videoFPS,     # define this globally or pass as parameter
    #         (width, height)  # define frame size
    #     )

    if param.display.animate is True:
        animate_simulation(data, param)

    # Get all figures created by the plotting functions
    figs = [plt.figure(num) for num in plt.get_fignums()]

    quit_flag = {"pressed": False}

    def on_key(event):
        if event.key == "q":
            quit_flag["pressed"] = True

    # Connect key handler to all figures
    for fig in figs:
        fig.canvas.mpl_connect("key_press_event", on_key)

    # Show all figures without blocking
    plt.show(block=False)

    # Wait without freezing GUI. Only wait for "q" when there actually are
    # figures on an interactive backend — otherwise no window can ever
    # receive the key press and the loop would hang forever.
    import matplotlib
    if figs and matplotlib.get_backend().lower() != "agg":
        while not quit_flag["pressed"]:
            plt.pause(0.1)

    # Close everything
    plt.close("all")

    print("Exited on q")

def composed_fig(data: SimulationData, param: Param):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.grid(True)
    ax.set_box_aspect([1, 1, 1])
    ax.view_init(elev=None, azim=None)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    target_hist = data.target_true_hist
    position_history = data.position_history

    # Remove steps above simulation time
    steps = np.array([50, 300, 600, 1000, 1500, 2000, 3000])
    steps = steps[steps < param.sim.Nsim]

    # === Create legend handles ===
    hdrone = ax.scatter(
        [], [], [],
        s=50,
        c='k',
        marker='x',
        label=r'$p_i$'
    )

    htarget_path, = ax.plot(
        [], [], [],
        'k--',
        label=r'$p_T$ path'
    )

    htarget_pos = ax.scatter(
        [],
        [],
        [],
        c='k',
        marker='o',
        label=r'$p_T$'
    )

    hpath, = ax.plot(
        [],
        [],
        [],
        'k',
        label=r'$p_i$ path'
    )

    hfinal_pos = ax.scatter(
        [],
        [],
        [],
        c='k',
        marker='d',
        label=r'$p^\star$'
    )

    lgd = ax.legend(
        handles=[
            hdrone,
            hpath,
            htarget_pos,
            htarget_path,
            hfinal_pos
        ],
        fontsize=10,
        loc='upper right'
    )
    # === End legend creation ===

    trail_length = param.display.trail_length
    for step in steps:
        idx = np.arange(
            max(0, step - trail_length),
            step
        )

        # --- target position at timestep step ---
        ax.scatter(
            target_hist[0, step],
            target_hist[1, step],
            target_hist[2, step],
            c='k',
            marker='o'
        )

        ax.text(
            target_hist[0, step] - 2,
            target_hist[1, step] - 2,
            target_hist[2, step] + 8,
            f"   {step/100:.2f} sec",
            fontsize=8
        )

        # --- chaser positions ---
        for iD in range(param.sim.nD):
            pos = position_history[iD,:,idx].T

            ax.plot(
                pos[0, :],
                pos[1, :],
                pos[2, :],
                color=data.colors[iD, :]
            )

            ax.scatter(
                pos[0, -1],
                pos[1, -1],
                pos[2, -1],
                marker='x',
                color=data.colors[iD, :]
            )

    if not os.path.isdir(param.display.save_location):
        os.makedirs(param.display.save_location)

    plt.savefig(
        f"{param.display.save_location}/composed_{param.sim.nD}_agents.pdf",
        bbox_inches='tight'
    )

    plt.show(block=False)

def radial_hist_plot(data: SimulationData, param: Param):
    """
    Plot radial distance history between all chasers and target.

    Parameters
    ----------
    data.target_position : ndarray
        Target position history (3 x N)
    data.chaser : list
        List containing chaser state histories
    param : dict
        Contains:
            - dTi : timestep
            - Nsim: number of simulation steps
            - nD  : number of chasers
    """

    fig, ax = plt.subplots(figsize=(8, 5))

    t = np.arange(param.sim.Nsim) * param.sim.dTi

    for iD in range(param.sim.nD):
        ax.plot(
            t,
            data.radial_history[iD],
            linewidth=2,
            color=data.colors[iD],
            label=f"Agent {iD+1}"
        )

    # Desired sphere radius
    ax.axhline(
        param.sim.sphere_radius,
        color="k",
        linestyle="--",
        linewidth=2,
        label="Desired radius"
    )

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Distance to target [m]")
    ax.set_title("Radial Distance History")

    ax.grid(True)
    ax.legend()

    plt.tight_layout()

    os.makedirs(param.display.save_location, exist_ok=True)

    plt.savefig(
        os.path.join(
            param.display.save_location,
            f"radial_hist_{param.sim.nD}_agents.pdf"
        ),
        bbox_inches="tight"
    )

    plt.show(block=False)

def consensus_plot(data: SimulationData, param: Param):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    ax.grid(True)
    ax.set_box_aspect([1, 1, 1])

    target_position = data.target_true_hist
    perceived_target_hist = data.perceived_target_hist

    # Plot true target trajectory
    ax.plot(
        target_position[0, :],
        target_position[1, :],
        target_position[2, :],
        color="k",
        linewidth=3,
        label="True target"
    )

    Ks = len(perceived_target_hist)
    Is = len(perceived_target_hist[0])
    for i in range(Is):
        x = np.full(Ks, np.nan)
        y = np.full(Ks, np.nan)
        z = np.full(Ks, np.nan)

        for k in range(Ks):

            est = perceived_target_hist[k][i]

            if est is not None and len(est) > 0:
                # expected 18x1 vector
                x[k] = est[0]
                y[k] = est[1]
                z[k] = est[2]

        # Plot estimated trajectory
        ax.plot(
            x,
            y,
            z,
            linewidth=2,
            label=f"Drone {i+1} est"
        )

        # Find valid indices
        valid_idx = np.where(~np.isnan(x))[0]

        if len(valid_idx) > 0:
            # Start point
            ax.scatter(
                x[valid_idx[0]],
                y[valid_idx[0]],
                z[valid_idx[0]],
                s=80,
                marker="o"
            )

            # End point
            ax.scatter(
                x[valid_idx[-1]],
                y[valid_idx[-1]],
                z[valid_idx[-1]],
                s=80,
                marker="d"
            )

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    ax.set_title(
        "Consensus Tracking of Moving Target "
        "(EMA + Noisy Measurements)"
    )

    ax.legend()

    if not os.path.isdir(param.display.save_location):
        os.makedirs(param.display.save_location)

    plt.savefig(
        f"{param.display.save_location}/consensus_{param.sim.nD}_agents.pdf",
        bbox_inches="tight"
    )

    plt.show(block=False)

def animate_simulation(data, param):
    """
    Plotly equivalent of MATLAB animate_simulation.
    """

    # -----------------------------
    # Compute axis limits
    # -----------------------------
    all_x = np.concatenate([
        data.position_history[:, 0, :].ravel(),
        data.target_true_hist[0]
    ])

    all_y = np.concatenate([
        data.position_history[:, 1, :].ravel(),
        data.target_true_hist[1]
    ])

    all_z = np.concatenate([
        data.position_history[:, 2, :].ravel(),
        data.target_true_hist[2]
    ])

    margin = 2

    # -----------------------------
    # Initial traces
    # -----------------------------
    traces = []

    # Initial positions
    for iD in range(param.sim.nD):
        traces.append(
            go.Scatter3d(
                x=[data.position_history[iD, 0, 0]],
                y=[data.position_history[iD, 1, 0]],
                z=[data.position_history[iD, 2, 0]],
                mode="markers",
                marker=dict(
                    size=6,
                    symbol="x",
                    color=data.colors[iD]
                ),
                name="Initial position"
            )
        )

    # Drone trajectories
    for iD in range(param.sim.nD):
        traces.append(
            go.Scatter3d(
                x=[],
                y=[],
                z=[],
                mode="lines",
                line=dict(
                    width=2,
                    color=data.colors[iD]
                ),
                name=f"Drone {iD+1}"
            )
        )

    # Drone current positions
    for iD in range(param.sim.nD):
        traces.append(
            go.Scatter3d(
                x=[],
                y=[],
                z=[],
                mode="markers",
                marker=dict(
                    size=8,
                    symbol="x",
                    color=data.colors[iD]
                ),
                name=f"Drone {iD+1} current"
            )
        )

    # Desired positions
    for iD in range(param.sim.nD):
        traces.append(
            go.Scatter3d(
                x=[],
                y=[],
                z=[],
                mode="markers",
                marker=dict(
                    size=8,
                    symbol="diamond",
                    color=data.colors[iD]
                ),
                name=f"Desired {iD+1}"
            )
        )

    # Target
    traces.append(
        go.Scatter3d(
            x=[],
            y=[],
            z=[],
            mode="markers",
            marker=dict(
                size=8,
                symbol="circle",
                color="black"
            ),
            name="Target"
        )
    )

    # Target trajectory
    traces.append(
        go.Scatter3d(
            x=[],
            y=[],
            z=[],
            mode="lines",
            line=dict(
                width=3,
                color="black"
            ),
            name="Target trajectory"
        )
    )

    # Reference circle
    th = np.linspace(0, 2*np.pi, 101)

    traces.append(
        go.Scatter3d(
            x=-5 + 15*np.cos(th),
            y=15*np.sin(th),
            z=np.ones_like(th)*5,
            mode="lines",
            line=dict(
                color="black",
                width=2
            ),
            name="Reference circle"
        )
    )


    # -----------------------------
    # Animation frames
    # -----------------------------
    frames = []

    n_initial = param.sim.nD
    n_traj = n_initial
    n_agents = param.sim.nD

    for k in range(0, param.sim.Nsim, param.display.step):

        frame_data = []

        start = max(0, k-param.display.trail_length)

        # Initial positions
        for iD in range(param.sim.nD):
            frame_data.append(
                go.Scatter3d(
                    visible=(start == 0),
                    x=[data.position_history[iD,0,0]],
                    y=[data.position_history[iD,1,0]],
                    z=[data.position_history[iD,2,0]]
                )
            )

        # Trajectories
        for iD in range(param.sim.nD):

            pos = data.position_history[iD,:,:]

            frame_data.append(
                go.Scatter3d(
                    x=pos[0,start:k+1],
                    y=pos[1,start:k+1],
                    z=pos[2,start:k+1]
                )
            )


        # Drone positions
        for iD in range(param.sim.nD):

            pos = data.position_history[iD,:,:]

            frame_data.append(
                go.Scatter3d(
                    x=[pos[0,k]],
                    y=[pos[1,k]],
                    z=[pos[2,k]]
                )
            )


        # Desired positions
        for iD in range(param.sim.nD):

            frame_data.append(
                go.Scatter3d(
                    x=[data.p_d_hist[iD][0,k]],
                    y=[data.p_d_hist[iD][1,k]],
                    z=[data.p_d_hist[iD][2,k]]
                )
            )


        # Target
        frame_data.append(
            go.Scatter3d(
                x=[data.target_true_hist[0,k]],
                y=[data.target_true_hist[1,k]],
                z=[data.target_true_hist[2,k]]
            )
        )


        # Target trajectory
        frame_data.append(
            go.Scatter3d(
                x=data.target_true_hist[0,:k+1],
                y=data.target_true_hist[1,:k+1],
                z=data.target_true_hist[2,:k+1]
            )
        )


        frames.append(
            go.Frame(
                data=frame_data,
                name=str(k)
            )
        )


    # -----------------------------
    # Figure
    # -----------------------------
    fig = go.Figure(
        data=traces,
        frames=frames
    )


    fig.update_layout(
        width=1000,
        height=800,
        scene=dict(
            xaxis=dict(
                range=[
                    all_x.min()-margin,
                    all_x.max()+margin
                ]
            ),
            yaxis=dict(
                range=[
                    all_y.min()-margin,
                    all_y.max()+margin
                ]
            ),
            zaxis=dict(
                range=[
                    all_z.min()-margin,
                    all_z.max()+margin
                ]
            ),
            aspectmode="data"
        ),

        updatemenus=[
            dict(
                type="buttons",
                buttons=[
                    dict(
                        label="Play",
                        method="animate",
                        args=[
                            None,
                            {
                                "frame": {
                                    "duration": 50,
                                    "redraw": True
                                },
                                "fromcurrent": True
                            }
                        ]
                    )
                ]
            )
        ],

        sliders=[
            dict(
                steps=[
                    dict(
                        method="animate",
                        args=[
                            [str(k)],
                            {
                                "frame": {
                                    "duration": 0,
                                    "redraw": True
                                }
                            }
                        ],
                        label=str(k)
                    )
                    for k in range(
                        0,
                        param.sim.Nsim,
                        param.display.step
                    )
                ]
            )
        ]
    )


    fig.show()

    return fig