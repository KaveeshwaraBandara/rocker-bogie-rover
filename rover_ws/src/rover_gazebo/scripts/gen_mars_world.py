#!/usr/bin/env python3
"""Generate mars_world.sdf: bounded rocky Mars terrain with many obstacles.

Lidar scans at z ~= 0.27 m, so every obstacle is >= 0.5 m tall to be visible.
Deterministic (seeded) so the world is reproducible.
"""
import math
import random

random.seed(42)

ROCK_COLORS = [
    (0.45, 0.26, 0.16),
    (0.52, 0.30, 0.18),
    (0.40, 0.22, 0.14),
    (0.58, 0.35, 0.22),
    (0.36, 0.20, 0.13),
]
GROUND = (0.71, 0.40, 0.24)
RIDGE = (0.48, 0.27, 0.16)

HALF = 11.0          # world half-size (22 x 22 m playable)
KEEPOUT = 1.6        # keep spawn area at origin clear

models = []


def color_str(c):
    return f"{c[0]:.2f} {c[1]:.2f} {c[2]:.2f} 1"


def add_box(name, x, y, z, sx, sy, sz, yaw=0.0, color=None, pitch=0.0, roll=0.0):
    c = color_str(color or random.choice(ROCK_COLORS))
    models.append(f"""    <model name="{name}">
      <static>true</static>
      <pose>{x:.2f} {y:.2f} {z:.2f} {roll:.3f} {pitch:.3f} {yaw:.3f}</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{sx:.2f} {sy:.2f} {sz:.2f}</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>{sx:.2f} {sy:.2f} {sz:.2f}</size></box></geometry>
          <material><ambient>{c}</ambient><diffuse>{c}</diffuse></material>
        </visual>
      </link>
    </model>""")


def add_sphere(name, x, y, r, color=None):
    c = color_str(color or random.choice(ROCK_COLORS))
    # bury the sphere a bit so it reads as a boulder
    models.append(f"""    <model name="{name}">
      <static>true</static>
      <pose>{x:.2f} {y:.2f} {r * 0.55:.2f} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><sphere><radius>{r:.2f}</radius></sphere></geometry></collision>
        <visual name="visual">
          <geometry><sphere><radius>{r:.2f}</radius></sphere></geometry>
          <material><ambient>{c}</ambient><diffuse>{c}</diffuse></material>
        </visual>
      </link>
    </model>""")


def add_cylinder(name, x, y, r, h, color=None):
    c = color_str(color or random.choice(ROCK_COLORS))
    models.append(f"""    <model name="{name}">
      <static>true</static>
      <pose>{x:.2f} {y:.2f} {h / 2:.2f} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><cylinder><radius>{r:.2f}</radius><length>{h:.2f}</length></cylinder></geometry></collision>
        <visual name="visual">
          <geometry><cylinder><radius>{r:.2f}</radius><length>{h:.2f}</length></cylinder></geometry>
          <material><ambient>{c}</ambient><diffuse>{c}</diffuse></material>
        </visual>
      </link>
    </model>""")


placed = []  # (x, y, radius) for overlap checks


def clear(x, y, r, min_gap=0.9):
    """True if a rock of footprint radius r at (x,y) leaves a navigable gap."""
    if math.hypot(x, y) < KEEPOUT + r:
        return False
    for px, py, pr in placed:
        if math.hypot(x - px, y - py) < r + pr + min_gap:
            return False
    return True


# ---- perimeter ridge: overlapping rotated blocks -> rocky rim ----
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

# ---- crater rim: arc of boulders around (5.5, 5.5), one gap to enter ----
crater_cx, crater_cy, crater_r = 5.5, 5.5, 2.6
n_cr = 14
for i in range(n_cr):
    ang = 2 * math.pi * i / n_cr
    if 3.6 < ang < 4.4:      # entrance gap facing the origin
        continue
    x = crater_cx + crater_r * math.cos(ang)
    y = crater_cy + crater_r * math.sin(ang)
    r = random.uniform(0.35, 0.55)
    add_sphere(f"crater_{i}", x, y, r)
    placed.append((x, y, r))

# ---- canyon: two parallel rock walls making a 1.6 m wide passage ----
for i, off in enumerate((-1.3, 1.3)):
    for j in range(4):
        x = -4.0 - j * 1.6
        y = -4.0 + off + random.uniform(-0.1, 0.1)
        add_box(f"canyon_{i}_{j}", x, y, random.uniform(0.5, 0.7),
                1.7, random.uniform(0.7, 1.0), random.uniform(1.0, 1.6),
                yaw=random.uniform(-0.15, 0.15))
        placed.append((x, y, 0.9))

# ---- two gentle ramps (12 deg) the rover can climb ----
for i, (rx, ry, ryaw) in enumerate(((0.0, 6.5, 0.0), (-6.5, 2.0, 1.2))):
    add_box(f"ramp_{i}", rx, ry, 0.10, 2.4, 1.6, 0.05,
            yaw=ryaw, pitch=-0.21, color=(0.55, 0.33, 0.20))
    placed.append((rx, ry, 1.5))

# ---- scattered boulders: mix of spheres, blocks, pillars ----
n_rocks = 34
attempts = 0
count = 0
while count < n_rocks and attempts < 4000:
    attempts += 1
    x = random.uniform(-HALF + 1.6, HALF - 1.6)
    y = random.uniform(-HALF + 1.6, HALF - 1.6)
    kind = random.random()
    if kind < 0.45:
        r = random.uniform(0.30, 0.60)
        if not clear(x, y, r):
            continue
        add_sphere(f"rock_s{count}", x, y, r)
        placed.append((x, y, r))
    elif kind < 0.8:
        sx, sy = random.uniform(0.5, 1.3), random.uniform(0.4, 1.0)
        r = max(sx, sy) / 2
        if not clear(x, y, r):
            continue
        add_box(f"rock_b{count}", x, y, random.uniform(0.3, 0.5),
                sx, sy, random.uniform(0.6, 1.4),
                yaw=random.uniform(0, 3.14),
                pitch=random.uniform(-0.15, 0.15),
                roll=random.uniform(-0.15, 0.15))
        placed.append((x, y, r))
    else:
        r = random.uniform(0.2, 0.4)
        if not clear(x, y, r):
            continue
        add_cylinder(f"rock_c{count}", x, y, r, random.uniform(0.7, 1.5))
        placed.append((x, y, r))
    count += 1

g = color_str(GROUND)
world = f"""<?xml version="1.0"?>
<!-- Generated by gen_mars_world.py (seed 42) - edit the generator, not this file -->
<sdf version="1.8">
  <world name="rover_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
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

import os
out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "worlds", "mars_world.sdf")
with open(out, "w") as f:
    f.write(world)
print(f"wrote {out}: {len(models)} models ({count} scattered rocks, {n_rim} rim blocks)")
