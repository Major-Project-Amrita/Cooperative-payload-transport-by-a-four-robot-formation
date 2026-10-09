"""Shared by the ROS 2 nodes: mission parameters and the lockstep messages.

/swarm/state   (plant -> controller), Float64MultiArray:
    [tick, t, K, x1, y1, yaw1, ..., has_plate, px, py, pyaw,
     hs, hm, pin_gap, tilt_deg, contact_static, contact_mover, contact_fmax]
/swarm/command (controller -> plant), Float64MultiArray:
    [tick, attach, finish, phase, v1, w1, ..., vK, wK]
"""
import os

import numpy as np

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def declare_mission_params(node):
    d = node.declare_parameter
    d('world_sdf', '')                 # '' -> the AWS warehouse slice
    d('contract', '')
    d('mjcf', '')
    d('algo', 'mpc')
    d('formation', 'square')
    d('grid_spacing', 1.18)
    d('payload', 'on_top')             # virtual | on_top | behind
    d('seed', 1)
    d('sense_sigma0', 0.02)
    d('kin', True)                     # load-carrying motion limits
    d('gate', 'exact')
    d('hold_err', 0.6)                 # formation feedback threshold (0 = off)
    d('timesteps', 3000)


def _find(path):
    """Resolve a world file given by name: as-is, in the source tree, or in
    the installed share directory."""
    if not path:
        return None
    if os.path.exists(path):
        return path
    cand = [os.path.join(PKG, 'worlds', os.path.basename(path))]
    try:
        from ament_index_python.packages import get_package_share_directory
        cand.append(os.path.join(get_package_share_directory('ros2_swarm_transport'),
                                 'worlds', os.path.basename(path)))
    except Exception:                                   # noqa: BLE001
        pass
    for c in cand:
        if os.path.exists(c):
            return c
    raise FileNotFoundError(path)


def mission_setup(node, video=None, gui=False):
    from .mujoco_validation import setup
    p = lambda n: node.get_parameter(n).value
    return setup(_find(p('world_sdf')), _find(p('contract')), _find(p('mjcf')),
                 algo=str(p('algo')), formation=str(p('formation')),
                 spacing=float(p('grid_spacing')), payload=str(p('payload')),
                 seed=int(p('seed')), sigma0=float(p('sense_sigma0')),
                 kin=bool(p('kin')), video=video, timesteps=int(p('timesteps')),
                 gate=str(p('gate')), hold_err=float(p('hold_err')), gui=gui)


class PlantProxy:
    """What MissionLog needs from the plant, rebuilt from a state message."""

    def __init__(self, pos, yaw, plate, pin_gap, tilt, contact):
        self._pos, self._yaw, self._plate = pos, yaw, plate
        self._pin, self._tilt, self.contact = pin_gap, tilt, contact

    def state(self):
        return self._pos, self._yaw

    def plate(self):
        return self._plate

    def pin_gap(self):
        return self._pin

    def tilt_deg(self):
        return self._tilt


def pack_state(tick, t, pos, yaw, plate, hs, hm, pin_gap, tilt, contact):
    K = len(pos)
    d = [float(tick), float(t), float(K)]
    for i in range(K):
        d += [float(pos[i][0]), float(pos[i][1]), float(yaw[i])]
    d += ([1.0, float(plate[0][0]), float(plate[0][1]), float(plate[1])]
          if plate is not None else [0.0, 0.0, 0.0, 0.0])
    d += [float(hs), float(hm), float(pin_gap), float(tilt),
          float(contact['static']), float(contact['mover']), float(contact['fmax'])]
    return d


def unpack_state(d):
    K = int(d[2])
    a = np.array(d[3:3 + 3 * K]).reshape(K, 3)
    o = 3 + 3 * K
    plate = (np.array(d[o + 1:o + 3]), float(d[o + 3])) if d[o] > 0 else None
    contact = {'static': int(d[o + 8]), 'mover': int(d[o + 9]), 'fmax': float(d[o + 10])}
    pos, yaw = a[:, :2].copy(), a[:, 2].copy()
    return {'tick': int(d[0]), 't': float(d[1]), 'pos': pos, 'yaw': yaw,
            'plate': plate, 'hs': bool(d[o + 4]), 'hm': bool(d[o + 5]),
            'proxy': PlantProxy(pos, yaw, plate, float(d[o + 6]), float(d[o + 7]), contact)}
