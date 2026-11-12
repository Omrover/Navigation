"""Geometría del dock dummy: AprilTag 36h11 detectado con OpenCV -> pose 2D del dock.

Convención del marcador de OpenCV (aruco/apriltag): origen en el centro, x a la
derecha y y hacia arriba vistos de frente, z saliendo del plano hacia la cámara.
Las esquinas se entregan en orden: sup-izq, sup-der, inf-der, inf-izq.

La pose que se publica para opennav_docking está en el centro del tag con x
apuntando HACIA el panel (la dirección en que mira el robot acoplado) y z arriba,
proyectada al plano del piso. Solo usa la normal del tag, así que no depende de
cómo esté girada la imagen en la cámara (en este robot va rotada 90° por el
montaje en el brazo) ni de la rotación del tag dentro del panel.
"""
import math

import numpy as np

try:  # OpenCV es opcional para la geometría pura (pruebas en Windows)
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None


def marker_object_points(size):
    """Esquinas 3D del marcador en su propio frame (mismo orden que OpenCV)."""
    h = size / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]],
                    dtype=np.float64)


def rodrigues(rvec):
    """Vector de rotación (eje * ángulo) -> matriz 3x3, sin depender de OpenCV."""
    rvec = np.asarray(rvec, dtype=np.float64).reshape(3)
    theta = float(np.linalg.norm(rvec))
    if theta < 1e-12:
        return np.eye(3)
    k = rvec / theta
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(theta) * kx + (1 - math.cos(theta)) * kx @ kx


def dock_pose_2d(r_cam_tag, t_cam_tag, t_out_cam):
    """Pose 2D (x, y, yaw) del dock en el frame de salida.

    r_cam_tag, t_cam_tag: rotación 3x3 y traslación del tag en el frame óptico.
    t_out_cam: transformación homogénea 4x4 frame_salida <- frame óptico.
    """
    t_out_cam = np.asarray(t_out_cam, dtype=np.float64)
    center = t_out_cam @ np.append(np.asarray(t_cam_tag, dtype=np.float64).reshape(3), 1.0)
    normal_out = t_out_cam[:3, :3] @ (-np.asarray(r_cam_tag)[:, 2])  # hacia el panel
    yaw = math.atan2(normal_out[1], normal_out[0])
    return float(center[0]), float(center[1]), yaw


def yaw_to_quaternion(yaw):
    """(x, y, z, w) de una rotación pura en z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def transform_from_tf(translation, rotation_xyzw):
    """Matriz 4x4 a partir de un TransformStamped (traslación y cuaternión)."""
    x, y, z, w = rotation_xyzw
    r = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    t = np.eye(4)
    t[:3, :3] = r
    t[:3, 3] = translation
    return t


# ----------------------------------------------------------------- OpenCV

_FAMILIES = {'36h11': 'DICT_APRILTAG_36h11', '25h9': 'DICT_APRILTAG_25h9',
             '16h5': 'DICT_APRILTAG_16h5'}


def dictionary(family):
    if cv2 is None:
        raise RuntimeError('OpenCV (python3-opencv) no está instalado')
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, _FAMILIES[family]))


def _detector_parameters():
    # En OpenCV 4.6 DetectorParameters() construye un objeto sin inicializar que
    # hace segfault en detectMarkers: ahí hay que usar DetectorParameters_create().
    params = (cv2.aruco.DetectorParameters_create()
              if hasattr(cv2.aruco, 'DetectorParameters_create')
              else cv2.aruco.DetectorParameters())
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return params


def detect_markers(gray, family):
    """Lista de (id, esquinas 4x2) compatible con OpenCV 4.6 (Ubuntu 24.04) y >= 4.7."""
    d = dictionary(family)
    params = _detector_parameters()
    if hasattr(cv2.aruco, 'ArucoDetector'):
        corners, ids, _ = cv2.aruco.ArucoDetector(d, params).detectMarkers(gray)
    else:  # OpenCV 4.6
        corners, ids, _ = cv2.aruco.detectMarkers(gray, d, parameters=params)
    if ids is None:
        return []
    return [(int(i), c.reshape(4, 2)) for i, c in zip(ids.flatten(), corners)]


def marker_pose(corners, size, k, dist):
    """(R 3x3, t 3) del marcador en el frame óptico, o None si solvePnP falla."""
    flag = getattr(cv2, 'SOLVEPNP_IPPE_SQUARE', cv2.SOLVEPNP_ITERATIVE)
    ok, rvec, tvec = cv2.solvePnP(marker_object_points(size),
                                  np.asarray(corners, dtype=np.float64),
                                  np.asarray(k, dtype=np.float64).reshape(3, 3),
                                  np.asarray(dist, dtype=np.float64).reshape(-1), flags=flag)
    if not ok:
        return None
    return rodrigues(rvec), tvec.reshape(3)


def marker_image(family, tag_id, side_px, margin_px=0):
    """Imagen del marcador (negro sobre blanco) para imprimir o texturizar."""
    d = dictionary(family)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        img = cv2.aruco.generateImageMarker(d, tag_id, side_px)
    else:  # OpenCV 4.6
        img = cv2.aruco.drawMarker(d, tag_id, side_px)
    if margin_px:
        img = cv2.copyMakeBorder(img, margin_px, margin_px, margin_px, margin_px,
                                 cv2.BORDER_CONSTANT, value=255)
    return img
