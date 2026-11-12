"""Quita del LiDAR los puntos que caen sobre el propio brazo.

El RPLIDAR barre un plano a la altura del hombro: con el brazo estirado al frente
(pose cero, pick/place sobre la mesa) o en tránsito entre posturas, el láser ve
los eslabones y SLAM, los costmaps y el collision_monitor los toman como
obstáculos pegados al robot ("Start occupied", MPPI frenado).

El brazo se modela como cápsulas entre orígenes de frames de la TF (posición
real de las juntas en el instante del scan). Los puntos a menos de
radio + margen de una cápsula se reemplazan por NaN (misma convención que
laser_filters), así los consumidores los ignoran.

    /scan -> scan_self_filter -> /scan_filtered
"""
import math

import numpy as np

# (frame_a, frame_b, radio [m]). Un frame repetido es una esfera.
DEFAULT_CAPSULES = (
    ('arm_link_1', 'arm_link_2', 0.035),
    ('arm_link_2', 'arm_link_3', 0.025),
    ('arm_link_3', 'arm_link_4', 0.025),
    ('arm_link_4', 'gripper_tcp', 0.05),
    ('camera_link', 'camera_link', 0.05),
)


def parse_capsules(spec):
    """['a b r', ...] -> [(a, b, r)]. Vacío -> DEFAULT_CAPSULES."""
    out = []
    for item in spec or ():
        a, b, r = item.split()
        out.append((a, b, float(r)))
    return out or list(DEFAULT_CAPSULES)


def scan_points(ranges, angle_min, angle_increment):
    """Puntos del scan en su frame (z = 0). Devuelve (N, 3)."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_increment * np.arange(r.size)
    return np.stack([r * np.cos(a), r * np.sin(a), np.zeros_like(r)], axis=1)


def capsule_mask(points, capsules, margin):
    """True donde el punto cae dentro de alguna cápsula (a, b, radio), con a y b
    como arreglos (3,) en el frame del scan."""
    hit = np.zeros(len(points), dtype=bool)
    for a, b, radius in capsules:
        a = np.asarray(a, dtype=float)
        ab = np.asarray(b, dtype=float) - a
        den = float(ab @ ab)
        t = np.zeros(len(points)) if den < 1e-12 else np.clip((points - a) @ ab / den, 0.0, 1.0)
        d = np.linalg.norm(points - (a + t[:, None] * ab), axis=1)
        hit |= d < radius + margin
    return hit


def filter_ranges(ranges, angle_min, angle_increment, capsules, margin):
    """Copia de ranges con NaN en los puntos que tocan el brazo, y cuántos son."""
    pts = scan_points(ranges, angle_min, angle_increment)
    finite = np.isfinite(pts).all(axis=1)
    mask = np.zeros(len(pts), dtype=bool)
    mask[finite] = capsule_mask(pts[finite], capsules, margin)
    out = [math.nan if m else float(r) for r, m in zip(ranges, mask)]
    return out, int(mask.sum())


def main(args=None):
    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer, TransformException, TransformListener

    class ScanSelfFilter(Node):
        def __init__(self):
            super().__init__('scan_self_filter')
            self.declare_parameter('input_topic', 'scan')
            self.declare_parameter('output_topic', 'scan_filtered')
            self.declare_parameter('capsules', [''])
            self.declare_parameter('margin', 0.04)
            self.capsules = parse_capsules(
                [c for c in self.get_parameter('capsules').value if c.strip()])
            self.margin = float(self.get_parameter('margin').value)
            self.tf = Buffer(cache_time=Duration(seconds=5.0))
            # Hilo propio para /tf: la espera dentro del callback no lo bloquea.
            self.tf_listener = TransformListener(self.tf, self, spin_thread=True)
            self.pub = self.create_publisher(
                LaserScan, self.get_parameter('output_topic').value, qos_profile_sensor_data)
            self.create_subscription(LaserScan, self.get_parameter('input_topic').value,
                                     self.on_scan, qos_profile_sensor_data)
            self.get_logger().info(
                f'{len(self.capsules)} cápsulas del brazo, margen {self.margin:.2f} m')

        def origin(self, target, frame):
            # Última TF conocida: los frames del brazo cuelgan del árbol del
            # propio robot (laser_link -> base_link -> arm_*), sin odom de por
            # medio, y el brazo se mueve despacio. Esperar la TF del instante
            # exacto del scan atrasaba el filtro (y con él a SLAM) ~1 s.
            tf = self.tf.lookup_transform(target, frame, Time())
            t = tf.transform.translation
            return np.array([t.x, t.y, t.z])

        def on_scan(self, msg):
            try:
                frames = {f for a, b, _ in self.capsules for f in (a, b)}
                pos = {f: self.origin(msg.header.frame_id, f) for f in frames}
            except TransformException as e:
                # Sin TF del brazo se publica el scan sin filtrar: es preferible
                # ver el brazo como obstáculo a quedar ciego.
                self.get_logger().warn(f'sin TF del brazo, scan sin filtrar: {e}',
                                       throttle_duration_sec=5.0)
                self.pub.publish(msg)
                return
            caps = [(pos[a], pos[b], r) for a, b, r in self.capsules]
            msg.ranges, _ = filter_ranges(msg.ranges, msg.angle_min, msg.angle_increment,
                                          caps, self.margin)
            self.pub.publish(msg)

    rclpy.init(args=args)
    node = ScanSelfFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
