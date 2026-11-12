"""Detector del dock dummy: AprilTag en la imagen RGB -> detected_dock_pose.

Publica geometry_msgs/PoseStamped en `output_frame` (odom por defecto), en 2D,
con el stamp de la imagen, para opennav_docking (SimpleNonChargingDock con
use_external_detection_pose). Solo procesa imágenes mientras haya alguien
suscrito, así no gasta CPU de la NUC fuera del acople.
"""
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from amr_navigation import tag_math


class DockDetector(Node):

    def __init__(self):
        super().__init__('dock_detector')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('tag_family', '36h11')
        self.declare_parameter('tag_id', 0)
        self.declare_parameter('tag_size', 0.10)
        self.declare_parameter('output_frame', 'odom')
        self.declare_parameter('max_rate_hz', 15.0)
        self.declare_parameter('always_on', False)
        p = self.get_parameter
        self.family = p('tag_family').value
        self.tag_id = int(p('tag_id').value)
        self.tag_size = float(p('tag_size').value)
        self.output_frame = p('output_frame').value
        self.min_period = 1.0 / max(float(p('max_rate_hz').value), 0.1)
        self.always_on = bool(p('always_on').value)
        tag_math.dictionary(self.family)  # falla al arrancar si falta OpenCV/aruco

        self.bridge = CvBridge()
        self.k = None
        self.dist = None
        self.last_t = None
        self.detections = 0
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        # Hilo propio para /tf: la espera de lookup_transform dentro del callback
        # de imagen bloquea el executor del nodo y la TF nunca llegaría a tiempo.
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.pub = self.create_publisher(PoseStamped, 'detected_dock_pose', 10)
        self.create_subscription(CameraInfo, p('camera_info_topic').value, self.on_info,
                                 qos_profile_sensor_data)
        # Profundidad 1: con la CPU cargada, procesar una imagen tarda más que el
        # periodo de la cámara; con una cola de 5 el detector llegó a ir >5 s
        # atrasado (la TF de esa imagen ya no estaba y el docking no lo veía).
        latest_only = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, p('image_topic').value, self.on_image, latest_only)
        self.get_logger().info(
            f'buscando tag {self.family} id {self.tag_id} ({self.tag_size:.3f} m) '
            f'-> detected_dock_pose en {self.output_frame}')

    def on_info(self, msg):
        self.k = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist = np.array(msg.d if len(msg.d) else [0.0] * 5, dtype=np.float64)

    def lookup(self, frame, stamp):
        """TF output_frame <- cámara en el instante de la imagen. Con la NUC
        cargada la odometría del bridge llega a atrasarse ~1 s respecto a la
        cámara: se espera hasta 0.5 s y, si no alcanza, se usa la última TF
        (el robot va despacio al acoplarse)."""
        try:
            return self.tf_buffer.lookup_transform(self.output_frame, frame, stamp,
                                                   timeout=Duration(seconds=0.5))
        except TransformException:
            pass
        try:
            return self.tf_buffer.lookup_transform(self.output_frame, frame, Time())
        except TransformException as e:
            self.get_logger().warn(f'sin TF {self.output_frame}<-{frame}: {e}',
                                   throttle_duration_sec=2.0)
            return None

    def on_image(self, msg):
        if self.k is None or (not self.always_on and self.pub.get_subscription_count() == 0):
            return
        stamp = Time.from_msg(msg.header.stamp)
        if self.last_t is not None and (stamp - self.last_t).nanoseconds * 1e-9 < self.min_period:
            return
        self.last_t = stamp
        gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        for tag_id, corners in tag_math.detect_markers(gray, self.family):
            if tag_id != self.tag_id:
                continue
            pose = tag_math.marker_pose(corners, self.tag_size, self.k, self.dist)
            if pose is None:
                return
            tf = self.lookup(msg.header.frame_id, stamp)
            if tf is None:
                return
            tr, rot = tf.transform.translation, tf.transform.rotation
            t_out_cam = tag_math.transform_from_tf([tr.x, tr.y, tr.z],
                                                   [rot.x, rot.y, rot.z, rot.w])
            x, y, yaw = tag_math.dock_pose_2d(pose[0], pose[1], t_out_cam)
            out = PoseStamped()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = self.output_frame
            out.pose.position.x = x
            out.pose.position.y = y
            q = tag_math.yaw_to_quaternion(yaw)
            out.pose.orientation.x, out.pose.orientation.y = q[0], q[1]
            out.pose.orientation.z, out.pose.orientation.w = q[2], q[3]
            self.pub.publish(out)
            self.detections += 1
            if self.detections == 1:
                self.get_logger().info(f'dock visto en ({x:.2f}, {y:.2f}, {yaw:.2f})')
            return


def main(args=None):
    rclpy.init(args=args)
    node = DockDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
