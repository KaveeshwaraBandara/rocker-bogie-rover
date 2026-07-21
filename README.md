<div align="center">

# 🤖 Rocker-Bogie Rover

**A Mars-rover-style 6-wheel robot — designed in SolidWorks, simulated in ROS 2, mapping and navigating fully autonomously with lidar SLAM.**

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange?logo=gazebo&logoColor=white)](https://gazebosim.org/)
[![Nav2](https://img.shields.io/badge/Nav2-Autonomous_Navigation-green)](https://docs.nav2.org/)
[![slam_toolbox](https://img.shields.io/badge/SLAM-slam__toolbox-purple)](https://github.com/SteveMacenski/slam_toolbox)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

<img src="docs/images/mars_gazebo.png" alt="Rover exploring the Mars world in Gazebo" width="850"/>

*The rover exploring the generated Mars world in Gazebo Sim*

</div>

---

## ✨ What is this?

A complete, working simulation stack for a **6-wheel rocker-bogie rover** (the suspension architecture used by NASA's Mars rovers). The robot was designed in SolidWorks, modeled as a parameterized URDF, and driven through the full autonomy pipeline:

**Design → Physics simulation → Lidar SLAM → Click-a-goal autonomous navigation.**

Click a point in RViz and the rover plans a path around obstacles, drives itself there, and maps the world as it goes.

## 📸 Gallery

| SolidWorks Design | URDF Model (RViz) |
|:---:|:---:|
| <img src="docs/images/cad_design.png" width="420"/> | <img src="docs/images/urdf_rviz.png" width="420"/> |
| **SLAM Map** *(10×10 m test room, 4 obstacles)* | **Autonomous Navigation** *(Mars world, live costmaps)* |
| <img src="docs/images/slam_map.png" width="420"/> | <img src="docs/images/mars_slam_nav.png" width="420"/> |

## 🚀 Features

- **Faithful rocker-bogie model** — all four suspension joints passive and live, with the tie-rod **differential actually simulated** (via a custom Gazebo plugin, since no physics engine can build the closed loop); parameterized xacro (every dimension is a named property)
- **Physics simulation** in Gazebo Sim (Harmonic) with a 6-wheel skid-steer drive and a 360° / 12 m lidar
- **Calibrated odometry** — skid-steer yaw error was measured against simulator ground truth and compensated (×1.39 effective wheel separation), giving drift-free enough odometry for clean maps
- **Online SLAM** with slam_toolbox — drive around (teleop or autonomously) and watch the map build live
- **Autonomous navigation** with Nav2 (MPPI controller) — set goals in RViz or via action calls; obstacle avoidance, path planning, and recovery behaviors included
- **Two simulation worlds** — a clean walled test room, and a procedurally generated **Mars world**: boulder fields, a crater with a single entrance, a narrow canyon passage, ramps, and a rocky perimeter ridge (97 models, reproducible from a seeded generator script)
- **Two localization modes** — map-while-navigating (SLAM), or AMCL localization on a previously saved map

## 🏗️ Architecture

```mermaid
flowchart LR
    subgraph Simulation
        GZ["Gazebo Sim<br/>(physics + lidar + drive)"]
    end
    BR["ros_gz_bridge"]
    subgraph Autonomy
        ST["slam_toolbox<br/>(map ↔ odom)"]
        NAV["Nav2<br/>(planner + MPPI controller)"]
    end
    RV["RViz<br/>(2D Goal Pose)"]

    GZ -- "/scan /odom /tf /clock" --> BR
    BR --> ST
    BR --> NAV
    ST -- "map → odom TF" --> NAV
    RV -- "/goal_pose" --> NAV
    NAV -- "/cmd_vel" --> BR
    BR --> GZ
```

| Package | Purpose |
|---|---|
| `rover_description` | URDF/xacro model, RViz visualization, display launch |
| `rover_gazebo` | Gazebo worlds, spawn + bridge launch, Mars world generators |
| `rover_gz_plugins` | `RockerDifferential` Gazebo system plugin (simulates the tie-rod differential) |
| `rover_navigation` | SLAM config, Nav2 config, navigation launch, saved maps |

## ⚡ Quick Start

**Prerequisites:** Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Sim (Harmonic, via `ros-jazzy-ros-gz`) · `ros-jazzy-slam-toolbox` · `ros-jazzy-nav2-bringup` · `ros-jazzy-teleop-twist-keyboard`

```bash
# clone & build
git clone https://github.com/KaveeshwaraBandara/rocker-bogie-rover.git
cd rocker-bogie-rover/rover_ws
colcon build --symlink-install
source install/setup.bash            # (or setup.zsh)
```

### Run it

| What | Command |
|---|---|
| 🔍 View the model + play with suspension | `ros2 launch rover_description display.launch.py` |
| 🎮 Simulate + drive with keyboard | `ros2 launch rover_gazebo sim.launch.py` + `ros2 run teleop_twist_keyboard teleop_twist_keyboard` |
| 🗺️ Build a map while driving | `ros2 launch rover_gazebo sim.launch.py rviz:=false` + `ros2 launch rover_navigation slam.launch.py` |
| 🤖 **Full autonomy** (SLAM + Nav2) | `ros2 launch rover_gazebo sim.launch.py rviz:=false` + `ros2 launch rover_navigation nav.launch.py` |
| 🔴 Autonomy on **Mars** | add `world:=mars_world.sdf` to the sim launch |
| ⛰️ **Suspension demo** — drive the rough-terrain test lane and watch the full rocker-bogie articulate | `ros2 launch rover_gazebo sim.launch.py world:=mars_rough_world.sdf` + teleop, drive along **+X** |
| 🏁 **Grand demo** — autonomous SLAM + Nav2 across rocky Mars terrain **while the suspension works live** | `ros2 launch rover_gazebo sim.launch.py rviz:=false world:=mars_demo_world.sdf` + `ros2 launch rover_navigation nav.launch.py`, then click goals on the mapped area |

Then in RViz: click **2D Goal Pose**, click anywhere on the map — the rover does the rest.

Send goals programmatically:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: -3.0}, orientation: {w: 1.0}}}}"
```

Save a finished map:

```bash
ros2 run nav2_map_server map_saver_cli -f my_map --ros-args -p use_sim_time:=true
```

Navigate on a saved map instead of live SLAM (AMCL):

```bash
ros2 launch rover_navigation nav.launch.py slam:=false
# then set the initial pose in RViz with "2D Pose Estimate"
```

## 🌍 Worlds

| `obstacle_world.sdf` *(default)* | `mars_world.sdf` | `mars_rough_world.sdf` | `mars_demo_world.sdf` |
|:---:|:---:|:---:|:---:|
| 10×10 m walled room, 4 obstacles — ideal for first mapping runs | 22×22 m Mars terrain — 34 boulders, crater, canyon, ramps, rocky ridge | 20×20 m rough-terrain suspension test lane — bump bars, staggered bars, log climbs, pebble field | 22×22 m **grand demo** — rocks + crater + ridge for SLAM/Nav2 **plus** 260 climbable pebbles and logs, so the bogies articulate while the rover navigates itself |

The Mars world is generated by [`rover_gazebo/scripts/gen_mars_world.py`](rover_ws/src/rover_gazebo/scripts/gen_mars_world.py) — tweak the rock density, crater position, or canyon width in the script, rerun it, rebuild, and you have a new (reproducible) planet. All obstacles are sized so the 2D lidar can see them, and rocks are placed with a guaranteed minimum gap so every region stays reachable.

## 🔧 Technical Notes

- **Suspension in simulation** — all four suspension joints (two rockers, two bogies) are passive revolutes in every world, and the **differential is genuinely simulated**. The real differential is a closed kinematic loop, which no Gazebo physics engine will build (a loop-closing joint is rejected outright, and the SDF `<mimic>` shorthand is ignored by DART), so it is enforced instead by a small gz-sim system plugin, [`RockerDifferential`](rover_ws/src/rover_gz_plugins/src/rocker_differential.cc): a stiff PD inside the physics loop that drives *left rocker + right rocker → 0*, exactly the constraint the tie rods apply, while leaving the differential mode completely free. Measured while driving rough terrain: constraint held to **0.17°**, left/right rocker travel mirrored exactly, bogies swinging −7°…+13°, chassis within **0.7° roll / 0.8° pitch**, and all six wheels staying loaded (max 0.3 mm clearance through a full in-place turn). Without the differential the suspension has a free internal degree of freedom and the chassis flops ~20° onto the joint limits, which is what makes ground contact and SLAM fall apart.
- **Skid-steer odometry calibration** — with six fixed wheels, in-place turns skid; raw wheel odometry over-reported rotation by ×1.39 (measured against Gazebo ground-truth pose). The drive plugin's effective wheel separation is calibrated accordingly — this single fix took the SLAM maps from unusable (rotated ghost rooms) to clean single-pass maps.
- **Drive torque is capped** at a realistic 0.6 N·m gearmotor stall, and wheel friction set to mu 0.8. This is not cosmetic: Gazebo's diff-drive is a *velocity* controller with unbounded torque, and a wheel's drive torque reacts on the arm it hangs from — both bogie wheels react on the same bogie, so the couple levers the front wheel clean off the ground and then pins the bogie at its travel limit. Capping the torque and lowering friction keeps every wheel loaded without costing traction (a wheel can only deliver ~0.4 N·m before it slips anyway).
- **Frames** follow REP-105: `map → odom → base_footprint → base_link → …`, lidar on `laser_frame`.

## 🗺️ Roadmap

- [x] URDF rocker-bogie model + RViz visualization
- [x] Gazebo simulation: skid-steer drive + lidar
- [x] SLAM mapping (slam_toolbox)
- [x] Nav2 autonomous navigation (SLAM + AMCL modes)
- [x] Mars demo world
- [ ] Waypoint-following missions
- [ ] Uneven-terrain worlds exercising the suspension
- [ ] Hardware build — this stack transfers to the real rover

## 🙏 Acknowledgments

- Rocker-bogie mechanism design tutorial: [Part 1 — parts design](https://youtu.be/X_NhGME49gQ) · [Part 2 — assembly & motion study](https://youtu.be/gKz1P_ilWDg)
- Built on [ROS 2](https://ros.org), [Gazebo Sim](https://gazebosim.org), [Nav2](https://docs.nav2.org), and [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)

## 📄 License

[MIT](LICENSE) © Kaveeshwara Bandara
