"""Coherencia de nav2_params.yaml con el robot y con el launch."""
import ast
import math
import os

import pytest

yaml = pytest.importorskip('yaml')

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Firmware (AMR_LLC/firmware): mecanum, 7.7 rad/s por rueda, r = 0.0762 m,
# ruedas en (±0.1583, ±0.1734) m: brazo de palanca de wz L = Lx + Ly.
WHEEL_MAX, R, L = 7.7, 0.0762, 0.1583 + 0.1734
ROBOT_VX_MAX = WHEEL_MAX * R   # x pura: todas las ruedas a vx / r
ROBOT_WZ_MAX = WHEEL_MAX * R / L


@pytest.fixture(scope='module')
def params():
    with open(os.path.join(PKG, 'config', 'nav2_params.yaml'), encoding='utf-8') as f:
        return yaml.safe_load(f)


def p(params, node):
    return params[node]['ros__parameters']


def test_mppi_is_omni_and_within_hardware(params):
    mppi = p(params, 'controller_server')['FollowPath']
    assert mppi['motion_model'] == 'Omni'
    assert 'TwirlingCritic' in mppi['critics']
    assert 'PreferForwardCritic' not in mppi['critics']
    # Rueda más cargada: |w| = (|vx| + |vy| + L*|wz|) / r. Cada par de ejes al
    # máximo debe caber; con los tres a la vez el firmware escala la consigna
    # conservando la dirección (amr_drive saturate()).
    def wheel(vx, vy, wz):
        return (vx + vy + L * wz) / R
    vx, vy, wz = mppi['vx_max'], mppi['vy_max'], mppi['wz_max']
    vx = max(vx, -mppi['vx_min'])
    assert max(wheel(vx, vy, 0), wheel(vx, 0, wz), wheel(0, vy, wz)) <= WHEEL_MAX
    assert mppi['vx_max'] < ROBOT_VX_MAX and mppi['wz_max'] < ROBOT_WZ_MAX


def test_velocity_chain_is_consistent(params):
    smoother = p(params, 'velocity_smoother')
    mppi = p(params, 'controller_server')['FollowPath']
    assert smoother['max_velocity'] == [mppi['vx_max'], mppi['vy_max'], mppi['wz_max']]
    assert smoother['max_velocity'][1] > 0.0, 'vy debe estar habilitada (base omni)'
    # Rampas del firmware: 1.0 m/s² y 3.0 rad/s².
    assert smoother['max_accel'][0] <= 1.0 and smoother['max_accel'][2] <= 3.0
    cm = p(params, 'collision_monitor')
    assert cm['cmd_vel_in_topic'] == 'cmd_vel_smoothed' and cm['cmd_vel_out_topic'] == 'cmd_vel'


def test_footprints_match(params):
    local = params['local_costmap']['local_costmap']['ros__parameters']
    glob = params['global_costmap']['global_costmap']['ros__parameters']
    assert local['footprint'] == glob['footprint']
    pts = ast.literal_eval(local['footprint'])
    xs = [pt[0] for pt in pts]
    ys = [pt[1] for pt in pts]
    # Las ruedas (x = 0.1583 + r, rectas) y el chasis 0.40 x 0.40 quedan dentro.
    assert max(xs) >= 0.1583 + R and max(ys) >= 0.20
    assert p(params, 'amcl')['robot_model_type'] == 'nav2_amcl::OmniMotionModel'


def test_dock_geometry(params):
    dock = p(params, 'docking_server')
    plugin = dock[dock['home_dock']['type']]
    assert plugin['plugin'] == 'opennav_docking::SimpleNonChargingDock'
    assert plugin['use_external_detection_pose']
    x, y, yaw = dock['home_dock']['pose']
    # Panel del mundo en x = -1.9 (0.05 de espesor): cara a -1.875. La pose de
    # acople queda a |translation_x| de la cara y el frente del robot (0.24) no la toca.
    face = -1.9 + 0.025
    assert x - face == pytest.approx(-plugin['external_detection_translation_x'], abs=1e-3)
    front = x - 0.24  # robot mirando a -x (yaw = pi)
    assert 0.0 < front - face < 0.08
    assert math.cos(yaw) == pytest.approx(-1.0, abs=1e-4)
    # Staging a 0.7 m del dock, en espacio libre del cuarto (muro oeste en x = -2).
    staging_x = x + plugin['staging_x_offset'] * math.cos(yaw)
    assert staging_x == pytest.approx(x + 0.7, abs=1e-3)
    assert dock['controller']['dock_collision_threshold'] > 0.28


def test_launch_lifecycle_nodes_have_params(params):
    with open(os.path.join(PKG, 'launch', 'navigation.launch.py'), encoding='utf-8') as f:
        tree = ast.parse(f.read())
    names = next(ast.literal_eval(n.value) for n in ast.walk(tree)
                 if isinstance(n, ast.Assign) and getattr(n.targets[0], 'id', '') ==
                 'LIFECYCLE_NODES')
    for name in names:
        assert name in params, f'{name} no tiene parámetros'
    assert 'route_server' not in names
