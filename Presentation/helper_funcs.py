import numpy as np
from manim import *

def fibonacci_points(n, R):
        """Return the n-point Fibonacci lattice on a sphere."""
        i = np.arange(n, dtype=float)

        # Equal-area spacing in y
        y = 1.0 - 2.0 * (i + 0.5) / n

        r = np.sqrt(1.0 - y * y)

        # Golden angle
        golden_angle = np.pi * (3.0 - np.sqrt(5.0))
        theta = i * golden_angle

        x = r * np.cos(theta)
        z = r * np.sin(theta)

        return R * np.column_stack((x, y, z))

def box(label, width=3.0, height=1.15, font_size=24):
    rect = RoundedRectangle(
        corner_radius=0.12,
        width=width,
        height=height,
    )
    txt = Text(label, font_size=font_size)
    txt.move_to(rect)
    return VGroup(rect, txt)
