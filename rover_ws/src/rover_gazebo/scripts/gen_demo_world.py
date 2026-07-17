#!/usr/bin/env python3
"""Generate mars_demo_world.sdf: the final full-capability demo world.

Tall lidar-visible obstacles for SLAM + Nav2 autonomy (like mars_world) AND
climbable rounded micro-terrain that makes the bogie suspension articulate
while the rover navigates ITSELF. Runs on DART with the calibrated skid-steer
odometry; sim.launch.py auto-sets bogies_free:=true for this world (bogies
are independent joints, no closed loop — rockers stay locked because the
mimic differential needs bullet, which cannot skid-steer; see CLAUDE.md).

Design rules (from mars_rough_world testing — see CLAUDE.md gotchas):
- climbable terrain is ROUNDED shapes only (half-buried spheres/cylinders),
  exposure <= 0.035 m and SPARSE (>= 0.8 m apart) so the rover rolls over
  one bump at a time
- tall obstacles >= 0.5 m (lidar plane at z ~0.27, chassis tilts on bumps)
- min 1.4 m surface gap between tall obstacles (robot_radius 0.30 +
  inflation 0.45 -> keeps corridors Nav2-passable)

Deterministic (seeded). Edit this generator, rerun it, colcon build.
"""
import math
import os
import random

random.seed(21)

ROCK_COLORS = [
    (0.45, 0.26, 0.16),
    (0.52, 0.30, 0.18),
    (0.40, 0.22, 0.14),
    (0.58, 0.35, 0.22),
    (0.36, 0.20, 0.13),
]
GROUND = (0.71, 0.40, 0.24)
RIDGE = (0.48, 0.27, 0.16)
BUMP = (0.55, 0.32, 0.19)

HALF = 11.0          # 22 x 22 m playable
SPAWN_KEEPOUT = 2.0

models = []
placed_tall = []     # (x, y, footprint_r) tall obstacles
placed_bump = []     # (x, y, r) climbable bumps


def color_str(c):
    return f"{c[0]:.2f} {c[1]:.2f} {c[2]:.2f} 1"


def add_box(name, x, y, z, sx, sy, sz, yaw=0.0, color=None, pitch=0.0, roll=0.0):
    c = color_str(color or random.choice(ROCK_COLORS))
    models.append(f"""    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} {roll:.3f} {pitch:.3f} {yaw:.3f}</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
          <material><ambient>{c}</ambient><diffuse>{c}</diffuse></material>
        </visual>
      </link>
    </model>""")


def add_sphere(name, x, y, r, z, color=None):
    c = color_str(color or random.choice(ROCK_COLORS))
    models.append(f"""    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><sphere><radius>{r:.3f}</radius></sphere></geometry></collision>
        <visual name="visual">
          <geometry><sphere><radius>{r:.3f}</radius></sphere></geometry>
          <material><ambient>{c}</ambient><diffuse>{c}</diffuse></material>
        </visual>
      </link>
    </model>""")


def add_cylinder(name, x, y, r, h, color=None):
    c = color_str(color or random.choice(ROCK_COLORS))
    models.append(f"""    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {h / 2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><cylinder><radius>{r:.3f}</radius><length>{h:.3f}</length></cylinder></geometry></collision>
        <visual name="visual">
          <geometry><cylinder><radius>{r:.3f}</radius><length>{h:.3f}</length></cylinder></geometry>
          <material><ambient>{c}</ambient><diffuse>{c}</diffuse></material>
        </visual>
      </link>
    </model>""")


def add_bar(name, x, y, length, r, exposed, yaw=0.0):
    """Half-buried horizontal cylinder -> smooth climbable bump bar."""
    c = color_str(BUMP)
    z = exposed - r
    models.append(f"""    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 1.5708 0 {yaw:.3f}</pose>
      <link name="link">
        <collision name="collision"><geometry><cylinder><radius>{r:.3f}</radius><length>{length:.3f}</length></cylinder></geometry></collision>
        <visual name="visual">
          <geometry><cylinder><radius>{r:.3f}</radius><length>{length:.3f}</length></cylinder></geometry>
          <material><ambient>{c}</ambient><diffuse>{c}</diffuse></material>
        </visual>
      </link>
    </model>""")


def clear_tall(x, y, r, min_gap=1.4):
    """Navigable placement for a tall obstacle of footprint radius r."""
    if math.hypot(x, y) < SPAWN_KEEPOUT + r:
        return False
    for px, py, pr in placed_tall:
        if math.hypot(x - px, y - py) < r + pr + min_gap:
            return False
    return True


def clear_bump(x, y, r):
    if math.hypot(x, y) < SPAWN_KEEPOUT:
        return False
    for px, py, pr in placed_tall:          # don't wedge bumps against rocks
        if math.hypot(x - px, y - py) < r + pr + 0.5:
            return False
    for px, py, pr in placed_bump:
        # dense enough that most routes cross bumps constantly (DART has no
        # wheel torque cap, so grinding/beaching is not a concern here), but
        # never stacked so wheels take one bump at a time
        if math.hypot(x - px, y - py) < r + pr + 0.35:
            return False
    return True


# ---- perimeter ridge ----
n_rim = 40
for i in range(n_rim):
    ang = 2 * math.pi * i / n_rim
    rr = HALF + random.uniform(-0.2, 0.4)
    x, y = rr * math.cos(ang), rr * math.sin(ang)
    add_box(f"rim_{i}", x, y, random.uniform(0.5, 0.8),
            random.uniform(1.8, 2.8), random.uniform(1.0, 1.6),
            random.uniform(1.2, 2.0),
            yaw=ang + random.uniform(-0.4, 0.4),
            pitch=random.uniform(-0.08, 0.08), color=RIDGE)

# ---- crater rim: arc of boulders around (5.5, 5.5), entrance facing origin ----
crater_cx, crater_cy, crater_r = 5.5, 5.5, 2.6
n_cr = 14
for i in range(n_cr):
    ang = 2 * math.pi * i / n_cr
    if 3.6 < ang < 4.4:
        continue
    x = crater_cx + crater_r * math.cos(ang)
    y = crater_cy + crater_r * math.sin(ang)
    r = random.uniform(0.35, 0.55)
    add_sphere(f"crater_{i}", x, y, r, z=r * 0.55)
    placed_tall.append((x, y, r))

# ---- tall scattered obstacles: mixed sizes for the lidar/costmaps ----
n_tall = 22
count = 0
attempts = 0
while count < n_tall and attempts < 6000:
    attempts += 1
    x = random.uniform(-HALF + 1.8, HALF - 1.8)
    y = random.uniform(-HALF + 1.8, HALF - 1.8)
    kind = random.random()
    if kind < 0.45:                       # boulder (sphere)
        r = random.uniform(0.30, 0.60)
        if not clear_tall(x, y, r):
            continue
        add_sphere(f"rock_s{count}", x, y, r, z=r * 0.55)
        placed_tall.append((x, y, r))
    elif kind < 0.75:                     # block
        sx, sy = random.uniform(0.5, 1.2), random.uniform(0.4, 0.9)
        r = max(sx, sy) / 2
        if not clear_tall(x, y, r):
            continue
        add_box(f"rock_b{count}", x, y, random.uniform(0.3, 0.5),
                sx, sy, random.uniform(0.6, 1.3),
                yaw=random.uniform(0, 3.14),
                pitch=random.uniform(-0.12, 0.12),
                roll=random.uniform(-0.12, 0.12))
        placed_tall.append((x, y, r))
    else:                                 # pillar
        r = random.uniform(0.18, 0.38)
        if not clear_tall(x, y, r):
            continue
        add_cylinder(f"rock_c{count}", x, y, r, random.uniform(0.7, 1.4))
        placed_tall.append((x, y, r))
    count += 1
n_tall_placed = count

# ---- climbable micro-terrain: half-buried pebbles everywhere ----
n_bumps = 260
count = 0
attempts = 0
while count < n_bumps and attempts < 40000:
    attempts += 1
    x = random.uniform(-HALF + 1.6, HALF - 1.6)
    y = random.uniform(-HALF + 1.6, HALF - 1.6)
    r = random.uniform(0.05, 0.12)
    if not clear_bump(x, y, r):
        continue
    exposed = random.uniform(0.015, 0.040)
    add_sphere(f"bump_{count}", x, y, r, z=exposed - r, color=BUMP)
    placed_bump.append((x, y, r))
    count += 1
n_bumps_placed = count

# ---- a few short log bars in open areas (bogie articulation moments) ----
logs = ((-3.5, 2.5, 0.6), (2.5, -3.0, -0.4), (-2.0, -6.0, 0.2), (6.0, -1.0, 1.0),
        (0.0, 4.0, 0.1), (-6.0, -2.0, 0.9), (4.0, 2.0, -0.7), (-5.0, 5.5, 0.3))
n_logs = 0
for i, (x, y, yaw) in enumerate(logs):
    if n_logs >= 4:
        break
    if any(math.hypot(x - px, y - py) < pr + 1.1 for px, py, pr in placed_tall):
        continue
    add_bar(f"log_{i}", x, y, 1.6, 0.06, exposed=0.030, yaw=yaw)
    placed_bump.append((x, y, 0.8))
    n_logs += 1

g = color_str(GROUND)
world = f"""<?xml version="1.0"?>
<!-- Generated by gen_demo_world.py (seed 21) - edit the generator, not this file -->
<sdf version="1.8">
  <world name="rover_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <!-- DART (default engine): the calibrated SLAM/Nav2 stack runs here.
         Suspension: bogies_free mode (rockers locked, bogies live) — the
         full mimic differential would need bullet, which can't skid-steer -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <scene>
      <ambient>0.5 0.38 0.3 1</ambient>
      <background>0.85 0.6 0.42 1</background>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1.0 0.82 0.65 1</diffuse>
      <specular>0.3 0.25 0.2 1</specular>
      <direction>-0.4 0.2 -0.85</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <material><ambient>{g}</ambient><diffuse>{g}</diffuse></material>
        </visual>
      </link>
    </model>

{chr(10).join(models)}
  </world>
</sdf>
"""

out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "worlds", "mars_demo_world.sdf")
with open(out, "w") as f:
    f.write(world)
print(f"wrote {out}: {len(models)} models "
      f"({n_tall_placed} tall rocks, {n_bumps_placed} bumps, {n_logs} logs, "
      f"{n_rim} rim blocks)")
