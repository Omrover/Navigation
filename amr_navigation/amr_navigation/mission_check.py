"""Recorrido de verificación de M4 en el gemelo digital (o en el robot).

    ros2 run amr_navigation nav_mission_check [--ros-args -p use_sim_time:=true]

1. Espera Nav2 activo, la odometría y el TF map->base_footprint.
2. Lleva el brazo a la postura dock_view (la cámara mira al frente a 0.38 m).
3. Navega por los waypoints (mesa de pick, mesa de place, centro) esquivando los
   obstáculos del mundo.
4. Acopla en home_dock (staging + aproximación con el AprilTag) y desacopla.

Si existe /sim/odom (verdad de Gazebo, el robot nace en el origen del mapa), mide
el error real de cada llegada y del acople. Imprime un resumen JSON y devuelve 0
solo si todo pasó.
"""
import json
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import DockRobot, NavigateToPose, UndockRobot
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from amr_interfaces.srv import ArmCommand

# Postura dock_view (rad): cámara horizontal a 0.38 m, todo el brazo detrás de
# x = 0.21 m (no choca con el panel del dock). Ver amr_navigation/README.md.
DOCK_VIEW = [0.0, 1.25, 0.1, 1.7]
WAYPOINTS = [  # (nombre, x, y, yaw) en el frame map = mundo de warehouse.sdf
    ('pick', 3.9, 1.8, 0.0),
    ('place', 3.9, -1.8, 0.0),
    ('center', 1.0, 0.0, math.pi),
]
DOCK_POSE = (-1.595, 0.0, math.pi)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class MissionCheck(Node):

    def __init__(self):
        super().__init__('nav_mission_check')
        self.declare_parameter('dock_id', 'home_dock')
        self.declare_parameter('skip_dock', False)
        self.declare_parameter('nav_timeout', 120.0)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.truth = None
        self.odom = None
        self.create_subscription(Odometry, '/sim/odom', self._on_truth, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.dock = ActionClient(self, DockRobot, 'dock_robot')
        self.undock = ActionClient(self, UndockRobot, 'undock_robot')
        self.arm = self.create_client(ArmCommand, 'llc/arm_command')
        self.results = []

    def _on_truth(self, msg):
        self.truth = msg

    def _on_odom(self, msg):
        self.odom = msg

    # ------------------------------------------------------------ utilidades

    def wait(self, predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return True
            time.sleep(0.05)
        return False

    def map_pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', Time())
        except TransformException:
            return None
        return (t.transform.translation.x, t.transform.translation.y,
                yaw_of(t.transform.rotation))

    def truth_pose(self):
        if self.truth is None:
            return None
        p = self.truth.pose.pose
        return p.position.x, p.position.y, yaw_of(p.orientation)

    def record(self, name, ok, target=None, **extra):
        entry = {'step': name, 'ok': bool(ok), **extra}
        for label, pose in (('estimate', self.map_pose()), ('truth', self.truth_pose())):
            if pose is not None:
                entry[label] = [round(v, 3) for v in pose]
                if target is not None:
                    entry[f'{label}_err_xy'] = round(math.hypot(pose[0] - target[0],
                                                                pose[1] - target[1]), 3)
                    entry[f'{label}_err_yaw'] = round(abs(angle_diff(pose[2], target[2])), 3)
        self.results.append(entry)
        self.get_logger().info(json.dumps(entry))
        return ok

    def run_action(self, client, goal, timeout):
        if not client.wait_for_server(timeout_sec=30.0):
            return None, 'servidor de acción no disponible'
        send = client.send_goal_async(goal)
        if not self.wait(send.done, 10.0) or not send.result().accepted:
            return None, 'meta rechazada'
        res = send.result().get_result_async()
        if not self.wait(res.done, timeout):
            send.result().cancel_goal_async()
            return None, 'timeout'
        return res.result(), None

    # --------------------------------------------------------------- pasos

    def setup(self):
        ok = self.wait(lambda: self.odom is not None and self.map_pose() is not None, 60.0)
        return self.record('ready', ok)

    def arm_dock_view(self):
        if not self.arm.wait_for_service(timeout_sec=10.0):
            return self.record('arm_dock_view', False, error='sin llc/arm_command')
        for req in (ArmCommand.Request(op=ArmCommand.Request.ENABLE),
                    ArmCommand.Request(op=ArmCommand.Request.GOAL, positions=DOCK_VIEW)):
            fut = self.arm.call_async(req)
            if not self.wait(fut.done, 5.0) or not fut.result().accepted:
                msg = fut.result().message if fut.done() else 'sin respuesta'
                return self.record('arm_dock_view', False, error=msg)
        time.sleep(4.0)  # perfil trapezoidal del brazo
        return self.record('arm_dock_view', True)

    def navigate(self, name, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        t0 = time.monotonic()
        result, err = self.run_action(self.nav, goal, self.get_parameter('nav_timeout').value)
        ok = err is None and result.status == GoalStatus.STATUS_SUCCEEDED
        return self.record(f'nav_{name}', ok, (x, y, yaw), seconds=round(time.monotonic() - t0, 1),
                           error=err)

    def dock_and_undock(self):
        goal = DockRobot.Goal()
        goal.use_dock_id = True
        goal.dock_id = self.get_parameter('dock_id').value
        goal.navigate_to_staging_pose = True
        t0 = time.monotonic()
        result, err = self.run_action(self.dock, goal, 180.0)
        ok = err is None and result.result.success
        self.record('dock', ok, DOCK_POSE, seconds=round(time.monotonic() - t0, 1),
                    error=err or (None if ok else
                                  f'{result.result.error_code}: {result.result.error_msg}'),
                    retries=None if err else result.result.num_retries)
        if not ok:
            return False
        goal = UndockRobot.Goal()
        goal.dock_type = 'dummy_dock'
        result, err = self.run_action(self.undock, goal, 60.0)
        ok = err is None and result.result.success
        return self.record('undock', ok, error=err)


def main(args=None):
    rclpy.init(args=args)
    node = MissionCheck()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    import threading
    threading.Thread(target=executor.spin, daemon=True).start()
    ok = node.setup() and node.arm_dock_view()
    if ok:
        for wp in WAYPOINTS:
            ok = node.navigate(*wp) and ok
        if not node.get_parameter('skip_dock').value:
            ok = node.dock_and_undock() and ok
    print('MISSION_CHECK ' + json.dumps({'ok': ok, 'steps': node.results}), flush=True)
    executor.shutdown()
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
