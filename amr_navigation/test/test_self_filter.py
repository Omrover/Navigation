"""Filtro del brazo sobre el LiDAR: geometría pura, sin ROS."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from amr_navigation import scan_self_filter as sf  # noqa: E402


def scan(n=360):
    return -math.pi, 2 * math.pi / n, n


def test_arm_ahead_is_removed_and_walls_kept():
    # Brazo estirado al frente en el plano del láser (lo que se vio en la
    # simulación con la pose cero): de 0.2 a 0.45 m adelante.
    amin, inc, n = scan()
    ranges = [3.0] * n
    ahead = n // 2                       # ángulo 0
    for k in range(-3, 4):
        ranges[ahead + k] = 0.25
    caps = [(np.array([0.15, 0.0, 0.0]), np.array([0.45, 0.0, 0.0]), 0.025)]
    out, removed = sf.filter_ranges(ranges, amin, inc, caps, 0.04)
    assert removed == 7
    assert all(math.isnan(out[ahead + k]) for k in range(-3, 4))
    assert out[0] == 3.0 and out[ahead + 10] == 3.0   # paredes intactas


def test_arm_above_scan_plane_keeps_everything():
    # dock_view: el brazo queda por encima del plano del láser.
    amin, inc, n = scan()
    ranges = [0.3] * n
    caps = [(np.array([0.1, 0.0, 0.15]), np.array([0.3, 0.0, 0.2]), 0.025)]
    out, removed = sf.filter_ranges(ranges, amin, inc, caps, 0.04)
    assert removed == 0 and out == ranges


def test_obstacle_behind_the_arm_is_kept():
    # El brazo tapa poco: un objeto a 0.8 m en la misma dirección no se borra.
    amin, inc, n = scan()
    ranges = [0.8] * n
    caps = [(np.array([0.15, 0.0, 0.0]), np.array([0.45, 0.0, 0.0]), 0.025)]
    out, removed = sf.filter_ranges(ranges, amin, inc, caps, 0.04)
    assert removed == 0


def test_sphere_and_non_finite_ranges():
    amin, inc, n = scan(4)                # ángulos -pi, -pi/2, 0, pi/2
    ranges = [float('inf'), 0.5, 0.3, float('nan')]
    caps = [(np.array([0.3, 0.0, 0.0]), np.array([0.3, 0.0, 0.0]), 0.05)]
    out, removed = sf.filter_ranges(ranges, amin, inc, caps, 0.0)
    assert removed == 1 and math.isnan(out[2])
    assert out[0] == float('inf') and out[1] == 0.5 and math.isnan(out[3])


def test_parse_capsules():
    assert sf.parse_capsules([]) == list(sf.DEFAULT_CAPSULES)
    assert sf.parse_capsules(['a b 0.03']) == [('a', 'b', 0.03)]
    with pytest.raises(ValueError):
        sf.parse_capsules(['a 0.03'])
