import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Delaunay
from sklearn.decomposition import PCA

from Structs import Param

def get_fibonacci_points(P, R, sphere_center=None):
    """
    Returns P approximately uniformly distributed points on a sphere using
    the Fibonacci sphere method.

    Parameters
    ----------
    P : int
        Number of points (must be odd and >= 3).
    R : float
        Sphere radius.
    sphere_center : array-like of shape (3,), optional
        Center of the sphere. Default is [0, 0, 0].

    Returns
    -------
    fibonacci_points : ndarray of shape (3, P)
        Cartesian coordinates of the points.
    """
    if P == 1:
        if sphere_center is None:
            sphere_center = np.array([0.0, 0.0, 0.0])
        else:
            sphere_center = np.asarray(sphere_center, dtype=float)

        fibonacci_points = np.array([[R, 0.0, 0.0]])[..., np.newaxis] + sphere_center
        return fibonacci_points

    if P % 2 == 0:
        raise ValueError("Even number of elements is not allowed.")

    if sphere_center is None:
        sphere_center = np.array([0.0, 0.0, 0.0])
    else:
        sphere_center = np.asarray(sphere_center, dtype=float)

    # Number of points
    N = (P - 1) // 2

    # Golden ratio
    phi_golden = (1 + np.sqrt(5)) / 2

    # Generate indices
    idx = np.arange(-N, N + 1)

    # Latitude and longitude
    theta_pos = np.arccos(2 * idx / P)
    theta = np.diff(theta_pos)

    phi_pos = 2 * np.pi * idx / phi_golden
    phi = np.diff(phi_pos)[0]

    theta_tmp = theta_pos[0]
    phi_tmp = phi_pos[0]

    x_fibonacci = np.zeros(P)
    y_fibonacci = np.zeros(P)
    z_fibonacci = np.zeros(P)

    for i in range(P - 1):
        x_fibonacci[i] = R * np.sin(theta_tmp) * np.cos(phi_tmp)
        y_fibonacci[i] = R * np.sin(theta_tmp) * np.sin(phi_tmp)
        z_fibonacci[i] = R * np.cos(theta_tmp)

        theta_tmp += theta[i]
        phi_tmp += phi

    # Last point
    x_fibonacci[P - 1] = R * np.sin(theta_tmp) * np.cos(phi_tmp)
    y_fibonacci[P - 1] = R * np.sin(theta_tmp) * np.sin(phi_tmp)
    z_fibonacci[P - 1] = R * np.cos(theta_tmp)

    fibonacci_points = np.stack(
        (x_fibonacci, y_fibonacci, z_fibonacci),
        axis=1
    )[..., np.newaxis] + sphere_center

    return fibonacci_points

def delaunay_triangulation_network(positions, verbose=False):
    """
    Creates a network using Delaunay triangulation.

    Parameters
    ----------
    positions : (N, D) ndarray
        Agent positions.
    verbose : bool, optional
        If True, plots the triangulation (only works for 2D).

    Returns
    -------
    connections : (N, N) ndarray of bool
        Adjacency matrix.
    laplacian : (N, N) ndarray
        Graph Laplacian.
    degree_matrix : (N, N) ndarray
        Degree matrix.
    """
    n_agents = len(positions)

    # remove singleton dimensions
    tol = 1e-9
    position_mask = (np.max(np.abs(positions - positions[0,:]), axis=0) >= tol)
    masked_positions = positions[:, position_mask]

    # Center
    X = masked_positions - masked_positions.mean(axis=0)

    # Normalize each coordinate
    scale = np.std(X, axis=0)
    scale[scale == 0] = 1
    X /= scale

    # Compute numerical rank
    r = np.linalg.matrix_rank(X, tol=1e-5)

    # PCA if necessary
    if r < positions.shape[1]:
        X = PCA(n_components=r).fit_transform(X)
    else:
        X = positions

    dt = Delaunay(X)

    # Extract unique edges
    edges = set()
    for simplex in dt.simplices:
        m = len(simplex)
        for i in range(m):
            for j in range(i + 1, m):
                a, b = sorted((simplex[i], simplex[j]))
                edges.add((a, b))

    # Build adjacency matrix
    A = np.zeros((n_agents, n_agents), dtype=bool)

    for i, j in edges:
        A[i, j] = True
        A[j, i] = True

    # Degree matrix
    degrees = A.sum(axis=1)
    degree_matrix = np.diag(degrees)

    # Laplacian
    laplacian = degree_matrix - A.astype(int)

    if not np.any(A):
        input("No connections found. Press Enter to continue...")

    return A, laplacian, degree_matrix

def geo_distances(points, R, sphere_center=None):
    """
    Compute pairwise geodesic distances between points on a sphere.

    Parameters
    ----------
    points : ndarray of shape (3, N)
        Cartesian coordinates of the points.
    R : float
        Radius of the sphere.
    sphere_center : array-like of shape (3,), optional
        Center of the sphere. Default is [0, 0, 0].

    Returns
    -------
    geo : ndarray of shape (N, N)
        Symmetric matrix of geodesic distances.
    """

    if sphere_center is None:
        sphere_center = np.zeros((3, 1))
    else:
        sphere_center = np.asarray(sphere_center, dtype=float).reshape(3, 1)

    points = np.asarray(points, dtype=float).reshape(3, -1)

    shifted = points - sphere_center
    cos_theta = np.clip((shifted.T @ shifted) / (R ** 2), -1.0, 1.0)

    geo = R * np.arccos(cos_theta)
    np.fill_diagonal(geo, 0.0)

    return geo

def get_min_safe_distance(fib_ref_points, param: Param, verbose=False):
    """
    Computes the average minimum neighbor distance on a sphere.

    Parameters
    ----------
    fib_ref_points : ndarray of shape (3, nD)
        Reference points on the sphere.
    param : object
        Must have attribute `nD`.
    verbose : bool, optional
        If True, prints the average neighbor distance.

    Returns
    -------
    min_safe_dist : float
        Average minimum distance to neighboring points.
    """

    if param.sim.nD == 1:
        return np.inf

    if param.sim.nD % 2 == 0:
        raise ValueError("Even number of elements is not allowed.")

    sphere_center = np.zeros([3,1])

    # Canonicalize to (3, nD) and project onto the unit sphere, since the
    # safeguard threshold is compared against unit-sphere geodesic distances
    points = np.asarray(fib_ref_points, dtype=float).reshape(param.sim.nD, 3).T
    points = points / np.linalg.norm(points, axis=0, keepdims=True)

    # Delaunay graph (expects (N, D) positions)
    connections, _, _ = delaunay_triangulation_network(points.T)

    if np.sum(connections) == 0:
        connections = np.ones((param.sim.nD, param.sim.nD), dtype=int) - np.eye(
            param.sim.nD, dtype=int
        )

    edge_dists = []

    for i in range(param.sim.nD):
        # Neighboring points
        neighbors = points[:, connections[i] == 1]

        vals = geo_distances(neighbors, 1, sphere_center)
        vals = vals[vals != 0]

        if vals.size == 0:
            min_val = np.nan
        else:
            min_val = np.min(vals)

        edge_dists.append(min_val)

    min_safe_dist = np.nanmean(edge_dists)

    if verbose:
        print(f"average_neighbor_distance: {min_safe_dist:.4f}")

    return min_safe_dist

import numpy as np


def project_to_sphere(points, r=1.0, center=None):
    """
    Projects 3D Cartesian points onto a spherical surface.

    Parameters
    ----------
    points : ndarray of shape (3, N) or (N, 3)
        Cartesian coordinates of the points.
    r : float, optional
        Radius of the sphere. Default is 1.
    center : array-like of shape (3,), optional
        Sphere center. Default is [0, 0, 0].

    Returns
    -------
    projected_points : ndarray
        Projected points with the same shape as the input.
    """

    if center is None:
        center = np.zeros(3)

    center = np.asarray(center, dtype=float).reshape(3, 1)
    points = np.asarray(points, dtype=float)

    # Accept either (3, N) or (N, 3)
    transpose = False
    if points.shape[0] != 3:
        if points.shape[1] == 3:
            points = points.T
            transpose = True
        else:
            raise ValueError("points must have shape (3, N) or (N, 3).")

    # Shift points so sphere center is at the origin
    shifted_points = points - center

    # Compute norms
    norms = np.linalg.norm(shifted_points, axis=0)
    norms[norms == 0] = np.finfo(float).eps  # avoid division by zero

    # Project onto sphere and shift back
    projected_points = center + r * shifted_points / norms

    if transpose:
        return projected_points.T

    return projected_points