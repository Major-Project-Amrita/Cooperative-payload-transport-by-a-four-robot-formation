# Starting the ROS 2 project — step by step

> **October 2026:** the MATLAB reference was audited and corrected after this
> package was written. The Python code in the bundle still has the pre-audit
> behaviour. `bundle/ROS2_REPLICATION.md` **section 9** lists what must be
> ported, `bundle/reference/matlab/` holds the corrected MATLAB files, and
> `bundle/AGENT_PROMPT.md` is a ready-to-paste prompt that does the port, the
> grid, the Gazebo runs and the report section.

The bundle is `release/ros2_swarm_transport_bundle.zip`. Work in a **fresh
directory**; nothing ROS belongs in the MATLAB repo.

## 1. Unpack

```bash
mkdir -p ~/ros2_ws && cd ~/ros2_ws
unzip /path/to/this/repo/release/ros2_swarm_transport_bundle.zip -d bundle
# the package is bundle/ros2_swarm_transport, the spec is bundle/ROS2_REPLICATION.md
```

## 2. Read the spec first

Open `bundle/ROS2_REPLICATION.md`. It defines exactly what to replicate:

- the **kinematics model** (unicycle, 10 Hz, turn-rate clip, heading
  low-pass) and which parts are model vs implementation,
- the **parameter contract** — every value that must match the MATLAB runs
  (also in `ros2_swarm_transport/params/swarm.yaml`),
- the **dynamic-obstacle contract** (`dynamic_obstacles.json` — the same file
  MATLAB reads; the mover node must reimplement `eval_mover_pose.m`
  faithfully; a Python reference already exists in `geometry.py`),
- the **payload decision**: virtual payload (recommended, matches MATLAB) vs
  physical load (separate experiment),
- the **verification protocol** with `compare_runs.m`.

## 3. Prerequisites

- ROS 2 (tested design targets **Humble** or **Jazzy**; the launch file uses
  `ros_gz_sim`, i.e. new Gazebo / gz-sim 8).
- `sudo apt install ros-<distro>-ros-gz ros-<distro>-turtlebot3-gazebo
  python3-scipy` and `pip install pytest numpy`.
- TurtleBot3 export: `export TURTLEBOT3_MODEL=burger` (or edit
  `worlds/agent.sdf` and spawn your own model — the footprint that matters is
  `robot_radius: 0.18` in `params/swarm.yaml`).

## 4. Build

```bash
cd ~/ros2_ws
mkdir -p src && cp -r bundle/ros2_swarm_transport src/
cd src && rosdep install -i --from-path . -y
cd .. && colcon build --symlink-install
source install/setup.bash
```

## 5. Run

```bash
# terminal 1 (after sourcing install/setup.bash and TURTLEBOT3_MODEL)
ros2 launch ros2_swarm_transport transport.launch.py

# terminal 2 — check the dynamic obstacles are alive
ros2 topic echo /dynamic_obstacles --once
```

You must still complete the robot spawning in
`launch/transport.launch.py` (four namespaced TB3s at the MATLAB start ring,
radius 2.2 m around (−12, 0); goal (12, 0)). The four odom topics must be
`/robot1..4/odom` — that is what the controller and logger subscribe to.

## 6. Log and compare against MATLAB

```bash
ros2 run ros2_swarm_transport swarm_logger --ros-args \
  -p robots:="[robot1,robot2,robot3,robot4]" \
  -p goal:="[12.0,0.0]" \
  -p output:=ros2_dynamic.csv -p rate:=10.0
```

Then copy the CSV back to the MATLAB repo and:

```matlab
compare_runs('matlab/logs/matlab_dynamic.csv', 'ros2_dynamic.csv', ...
             'matlab/swarm_world_dynamic.sdf')
```

The clearance column is recomputed from the SDF for **both** runs, so the
comparison is geometry-identical regardless of what a costmap reported.

## 7. Sanity-check without Gazebo

The geometry/VFH/assignment ports are pure Python and unit-tested:

```bash
cd src/ros2_swarm_transport
python3 -m pytest test/ -q        # 32 tests, no ROS needed
```

`test_pvfh_predict_sees_mover_sooner` is the one that validates the
dynamic-obstacle prediction path.

## What "done" looks like

`compare_runs` prints two metric tables. Expected: `formation_rms` small but
non-zero (that is your controller), `min_clearance` within ~10% of the MATLAB
value (0.51 m), path efficiencies within ~0.1 of each other. Large Fréchet
distance with good clearance usually means a parameter mismatch — re-check
`params/swarm.yaml` against ROS2_REPLICATION.md §5 before touching gains.
