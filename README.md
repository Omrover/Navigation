# Navigation

Navegación autónoma del Mobile Manipulator Omrover con ROS 2 Jazzy: SLAM con
slam_toolbox, localización AMCL con el modelo omni (sirve para la base mecanum),
Nav2 con MPPI holonómico y un
dock dummy con AprilTag.

## Paquete `amr_navigation`

| Archivo | Qué hace |
|---|---|
| `config/nav2_params.yaml` | Nav2: MPPI `Omni`, SmacPlanner2D, velocity_smoother con vx/vy/wz, collision_monitor, AMCL `OmniMotionModel` y docking_server con un `SimpleNonChargingDock` |
| `config/slam_mapping.yaml` | slam_toolbox online async para el RPLIDAR A1 (12 m, 8 Hz) |
| `config/dock_detector.yaml` | Tag del dock: 36h11, id 0, 10 cm |
| `launch/slam.launch.py` | Mapeo |
| `launch/localization.launch.py` | map_server + AMCL sobre un mapa guardado |
| `launch/navigation.launch.py` | Servidores de Nav2 + docking, sin route_server |
| `launch/sim_nav.launch.py` | Gazebo + MCU simulado + bridge + SLAM o AMCL + Nav2 + detector del dock |
| `amr_navigation/dock_detector.py` | AprilTag con OpenCV (`cv2.aruco`, sin apriltag_ros) → `detected_dock_pose` |
| `amr_navigation/mission_check.py` | Recorrido de verificación: waypoints + acople + desacople |

Cadena de velocidad:

```
controller_server → cmd_vel_nav → velocity_smoother → cmd_vel_smoothed
  → collision_monitor → cmd_vel → amr_mqtt_bridge → MCU
```

El `docking_server` publica `cmd_vel` directo, a baja velocidad.

### Límites

La base es mecanum: cada rueda pide |vx| + |vy| + (Lx + Ly)|wz| en su superficie.
El firmware satura cada rueda en 7.7 rad/s, lo que da unos 0.59 m/s en x o y puras y
1.77 rad/s de giro. Nav2 queda por debajo para no sacudir el brazo: 0.35 m/s en vx,
0.20 m/s en vy y 0.65 rad/s en wz. Cada par de ejes al máximo cabe en las ruedas. Si se
piden los tres a la vez, el firmware escala la consigna y conserva la dirección.

### Dock dummy

- **Panel:** 0.6 m de alto, con un AprilTag 36h11 id 0 de 10 cm (14 cm con el
  margen blanco) centrado a 0.38 m. La imagen para imprimir está en
  `AMR_master/amr_gazebo/models/omr_dock/materials/textures/tag36h11_0.png`.
- **Postura `dock_view` del brazo:** q = [0, 1.25, 0.1, 1.7] rad. La cámara queda
  horizontal, mirando al frente a 0.38 m de altura, y todo el brazo queda detrás de
  x = 0.21 m, así que no choca con el panel.
- **Pose de acople:** 0.28 m antes de la cara del panel. El frente del robot queda
  a unos 4 cm. El staging está a 0.7 m.
- En `nav2_params.yaml`, `home_dock` usa la pose del mundo de simulación. En el
  laboratorio hay que medirla en el mapa: basta llevar el robot a mano a la
  posición de acople y leer `map → base_footprint`.

## Uso

```bash
# Simulación (NUC, sin GUI): mapeo y luego verificación
ros2 launch amr_navigation sim_nav.launch.py
ros2 run amr_navigation nav_mission_check --ros-args -p use_sim_time:=true
ros2 run nav2_map_server map_saver_cli -f ~/amr_ws/src/Navigation/amr_navigation/maps/warehouse \
    --ros-args -p use_sim_time:=true

# Con el mapa guardado
ros2 launch amr_navigation sim_nav.launch.py mode:=amcl

# Robot real (bridge ya corriendo; falta el driver del RPLIDAR, ver abajo)
ros2 launch amr_navigation slam.launch.py
ros2 launch amr_navigation navigation.launch.py
```

En el robot real hacen falta `ros-jazzy-rplidar-ros` (con la regla udev
`/dev/rplidar`) y `ros-jazzy-realsense2-camera`, que se instalan con sudo.
