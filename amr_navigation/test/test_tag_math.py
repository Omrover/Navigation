"""Geometría del dock: tag -> pose 2D, y detección sobre una imagen sintética."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from amr_navigation import tag_math  # noqa: E402

# Frame óptico (z adelante, x derecha, y abajo) respecto a un frame robot
# (x adelante, y izquierda, z arriba): la rotación estándar de REP-103.
R_ROBOT_OPTICAL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def homogeneous(r, t):
    m = np.eye(4)
    m[:3, :3] = r
    m[:3, 3] = t
    return m


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def tag_facing_camera(distance, lateral=0.0, tilt=0.0):
    """Tag de frente a la cámara (normal hacia ella), con giro opcional en su plano."""
    # Marcador: z hacia la cámara = -z óptico; x a la derecha de la imagen; y arriba.
    r = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
    c, s = math.cos(tilt), math.sin(tilt)
    in_plane = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return r @ in_plane, np.array([lateral, 0.0, distance])


def test_rodrigues_matches_axis_angle():
    r = tag_math.rodrigues([0.0, 0.0, math.pi / 2])
    assert np.allclose(r @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])
    assert np.allclose(tag_math.rodrigues([0, 0, 0]), np.eye(3))


@pytest.mark.parametrize('robot_yaw', [0.0, 0.7, math.pi, -2.0])
@pytest.mark.parametrize('tilt', [0.0, math.pi / 2, 2.5])   # imagen girada (montaje en brazo)
def test_tag_straight_ahead_gives_dock_in_front(robot_yaw, tilt):
    # Robot en (1, 2) con yaw dado; cámara 0.1 m adelante y 0.38 m arriba.
    t_odom_robot = homogeneous(rot_z(robot_yaw), [1.0, 2.0, 0.0])
    t_robot_cam = homogeneous(R_ROBOT_OPTICAL, [0.1, 0.0, 0.38])
    r, t = tag_facing_camera(0.5, tilt=tilt)
    x, y, yaw = tag_math.dock_pose_2d(r, t, t_odom_robot @ t_robot_cam)
    ahead = 0.6
    assert x == pytest.approx(1.0 + ahead * math.cos(robot_yaw), abs=1e-9)
    assert y == pytest.approx(2.0 + ahead * math.sin(robot_yaw), abs=1e-9)
    # La pose del dock mira hacia el panel: misma dirección que el robot.
    assert math.cos(yaw - robot_yaw) == pytest.approx(1.0, abs=1e-9)


def test_rolled_camera_does_not_change_pose():
    # La cámara del brazo va rotada 90° en su eje óptico: misma pose del dock.
    t_cam = homogeneous(R_ROBOT_OPTICAL, [0.1, 0.0, 0.38])
    roll = homogeneous(np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]), [0, 0, 0])
    r, t = tag_facing_camera(0.4, lateral=0.05)
    a = tag_math.dock_pose_2d(r, t, t_cam)
    rr, tt = roll[:3, :3].T @ r, roll[:3, :3].T @ t   # mismo tag visto desde la cámara girada
    b = tag_math.dock_pose_2d(rr, tt, t_cam @ roll)
    assert np.allclose(a, b)


def test_quaternion_and_tf_roundtrip():
    q = tag_math.yaw_to_quaternion(1.2)
    m = tag_math.transform_from_tf([1, 2, 3], q)
    assert np.allclose(m[:3, :3], rot_z(1.2))
    assert np.allclose(m[:3, 3], [1, 2, 3])


cv2 = pytest.importorskip('cv2')


def test_detects_synthetic_tag_and_recovers_pose():
    # Render del tag de 10 cm a 0.6 m con la K de la cámara simulada (640x480, 69°).
    size, dist = 0.10, 0.6
    f = 320.0 / math.tan(1.204 / 2.0)
    k = np.array([[f, 0, 320.0], [0, f, 240.0], [0, 0, 1.0]])
    tag = tag_math.marker_image('36h11', 0, 400)
    r, t = tag_facing_camera(dist, lateral=0.03, tilt=0.4)
    obj = tag_math.marker_object_points(size)
    img_pts = (k @ ((r @ obj.T).T + t).T).T
    img_pts = (img_pts[:, :2] / img_pts[:, 2:]).astype(np.float32)
    src = np.float32([[0, 0], [399, 0], [399, 399], [0, 399]])
    h = cv2.getPerspectiveTransform(src, img_pts)
    gray = cv2.warpPerspective(tag, h, (640, 480), borderValue=255)

    found = tag_math.detect_markers(gray, '36h11')
    assert [i for i, _ in found] == [0]
    rr, tt = tag_math.marker_pose(found[0][1], size, k, np.zeros(5))
    assert np.allclose(tt, t, atol=0.01)
    assert np.dot(rr[:, 2], r[:, 2]) == pytest.approx(1.0, abs=0.02)
