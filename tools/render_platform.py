"""Render the robot platform and the carried-plate formation (docs/figures/robot_platform.png)."""
import os, sys, math
os.environ.setdefault("MUJOCO_GL", "egl")
import numpy as np, mujoco
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src", "ros2_swarm_transport"))
import ros2_swarm_transport.mujoco_validation as mv

base = ('<mujoco><visual><headlight ambient="0.35 0.35 0.35" diffuse="0.55 0.55 0.55"/>'
        '<quality shadowsize="4096"/></visual><asset>'
        '<texture name="g" type="2d" builtin="checker" rgb1="0.80 0.81 0.82" rgb2="0.74 0.75 0.76" width="256" height="256"/>'
        '<material name="g" texture="g" texrepeat="4 4" texuniform="true"/></asset>'
        '<worldbody><light pos="1 -2 4" dir="-0.2 0.4 -1" diffuse="0.6 0.6 0.6"/>'
        '<geom name="floor" type="plane" size="0 0 0.05" material="g"/></worldbody></mujoco>')
def shot(payload, cam_kw, robots_xy, size=(900, 640)):
    xml = mv.build_scene(base, [], 4, "square", 1.18, payload, np.array([[50.0, 0.0]]))
    xml = xml.replace('offwidth="1280" offheight="720"', 'offwidth="1280" offheight="900"')
    m = mujoco.MjModel.from_xml_string(xml); d = mujoco.MjData(m)
    for i, (x, y) in enumerate(robots_xy):
        mv._set_free(m, d, f"robot{i+1}_free", x, y, 0.0, 0.0)
    if payload == "on_top":
        mv._set_free(m, d, "payload_free", 0, 0, 0, 0)
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, size[1], size[0]); c = mujoco.MjvCamera()
    c.type = mujoco.mjtCamera.mjCAMERA_FREE
    c.lookat[:] = cam_kw["lookat"]; c.distance = cam_kw["distance"]
    c.azimuth = cam_kw["azimuth"]; c.elevation = cam_kw["elevation"]
    r.update_scene(d, camera=c); img = r.render().copy(); r.close()
    return img
far = [(20 + 3 * i, 20) for i in range(4)]
one = shot("virtual", dict(lookat=[0, 0, 0.09], distance=0.95, azimuth=140, elevation=-22), [(0, 0)] + far[1:])
side = shot("virtual", dict(lookat=[0, 0, 0.09], distance=0.9, azimuth=90, elevation=-2), [(0, 0)] + far[1:])
sq = [(-0.59, -0.59), (0.59, -0.59), (0.59, 0.59), (-0.59, 0.59)]
form = shot("on_top", dict(lookat=[0, 0, 0.3], distance=4.2, azimuth=130, elevation=-28), sq)
bare = shot("virtual", dict(lookat=[0, 0, 0.1], distance=4.2, azimuth=130, elevation=-28), sq)
fig, ax = plt.subplots(2, 2, figsize=(11, 7.8), facecolor="#fcfcfb")
for a, im, t in zip(ax.ravel(), [one, side, bare, form],
                    ["One robot (3/4 view)", "Side view: wheels at mid-body, a caster front and back",
                     "Square formation, s = 1.18 m, no load", "Same formation carrying the plate + crate (12 kg)"]):
    a.imshow(im); a.set_axis_off(); a.set_title(t, fontsize=10, weight="bold")
fig.text(0.5, 0.012,
         "Generic differential-drive platform: footprint radius 0.18 m, height 0.20 m, mass 6.9 kg, wheel radius 0.05 m, track 0.28 m,\n"
         "wheel speed limit 1.2 m/s, wheel torque limit 2 N m (40 N of traction per wheel), two low-friction ball casters. "
         "The load is pinned to each robot at axle height.", ha="center", fontsize=8.5, color="#52514e")
fig.subplots_adjust(left=0.01, right=0.99, top=0.95, bottom=0.08, wspace=0.03, hspace=0.1)
out = os.path.join(ROOT, "docs", "figures", "robot_platform.png")
fig.savefig(out, dpi=130); print("wrote", out)
