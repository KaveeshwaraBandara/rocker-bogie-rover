#!/usr/bin/env python3
"""Generate mars_rough_world.sdf: a rocker-bogie suspension test track on Mars.

Unlike mars_world.sdf (tall lidar obstacles on flat ground), everything on the
track here is CLIMBABLE — bump heights are sized against the 0.05 m wheel
radius so the rover drives OVER them and the passive rocker/bogie joints
visibly articulate. Test lane runs along +X from the spawn at the origin:

  Zone A  x 1.2..3.0   full-width bump bars (both sides hit together -> bogies)
  Zone B  x 3.6..5.6   staggered left/right half-bars (one side at a time ->
                       rocker differential: chassis averages left/right pitch)
  Zone C  x 6.4..7.0   log crossing (two full-width logs, 3 and 4 cm)
  Zone D  x 7.6..9.6   random half-buried boulder field (slow torque-limited
                       rock crawl -- the toughest part, so it goes last)

Tall scenery rocks sit OUTSIDE the lane (|y| > 1.8) for looks and lidar.
Deterministic (seeded). Edit this generator, then rerun it + colcon build.
"""
import math
import os
import random

random.seed(7)

ROCK_COLORS = [
    (0.45, 0.26, 0.16),
    (0.52, 0.30, 0.18),
    (0.40, 0.22, 0.14),
    (0.58, 0.35, 0.22),
    (0.36, 0.20, 0.13),
]
GROUND = (0.71, 0.40, 0.24)
RIDGE = (0.48, 0.27, 0.16)
BAR = (0.50, 0.29, 0.17)
SLAB = (0.55, 0.33, 0.20)

WHEEL_R = 0.05       # rover wheel radius the bump sizes are tuned against
HALF = 10.0          # world half-size (20 x 20 m playable)

models = []


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
    """Sphere centred at height z (negative = buried)."""
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


def add_bar(name, x, y_center, length, r, exposed):
    """Transverse half-buried cylinder (axis along Y) -> a smooth bump bar."""
    c = color_str(BAR)
    z = exposed - r
    models.append(f"""    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y_center:.3f} {z:.3f} 1.5708 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><cylinder><radius>{r:.3f}</radius><length>{length:.3f}</length></cylinder></geometry></collision>
        <visual name="visual">
          <geometry><cylinder><radius>{r:.3f}</radius><length>{length:.3f}</length></cylinder></geometry>
          <material><ambient>{c}</ambient><diffuse>{c}</diffuse></material>
        </visual>
      </link>
    </model>""")


# ---- Zone A: full-width bump bars (both wheels of a side hit together) ----
for i, x in enumerate((1.2, 1.8, 2.4, 3.0)):
    add_bar(f"bumpA_{i}", x, 0.0, 1.2, 0.045, exposed=0.030)

# ---- Zone B: staggered one-side half-bars (rocker differential demo) ----
# Left wheels run at y = +0.19, right wheels at y = -0.19. Alternate which
# side hits first per pair — same-side-first pairs each kick yaw the same
# way and the accumulated drift walks the rover off the lane.
for i, x in enumerate((3.6, 4.8, 5.2)):
    add_bar(f"bumpB_L{i}", x, 0.30, 0.62, 0.05, exposed=0.035)
for i, x in enumerate((4.0, 4.4, 5.6)):
    add_bar(f"bumpB_R{i}", x, -0.30, 0.62, 0.05, exposed=0.035)

# ---- Zone C: log crossing — half-buried cylinders of growing exposure ----
# ROUNDED shapes only: bullet-featherstone launches this light rover off
# thin box edges (step slabs, pitched ramps all caused backflips in testing)
# but half-buried cylinders/spheres are contact-stable and climb reliably.
# Full-width logs can't be dodged, so they stay a notch below the pebble
# ceiling (a 0.045 full-width log needed more than the capped stall torque).
for i, (x, r, exp) in enumerate(((6.4, 0.07, 0.030),
                                 (7.0, 0.09, 0.040))):
    add_bar(f"log_{i}", x, 0.0, 3.0, r, exposed=exp)

# ---- Zone D: half-buried boulder field across the lane ----
# exposures tuned against the 0.65 N.m stall-torque cap: at 0.045 the dense
# field reliably beached the rover mid-crossing; <=0.038 crosses blind
for i in range(16):
    x = random.uniform(7.6, 9.6)
    y = random.uniform(-0.85, 0.85)
    r = random.uniform(0.05, 0.10)
    exposed = random.uniform(0.015, 0.038)
    add_sphere(f"pebble_{i}", x, y, r, z=exposed - r)

# ---- tall scenery rocks OUTSIDE the lane (lidar features) ----
placed = []
n_scenery = 20
count = 0
attempts = 0
while count < n_scenery and attempts < 3000:
    attempts += 1
    x = random.uniform(-HALF + 1.5, HALF - 1.5)
    y = random.uniform(-HALF + 1.5, HALF - 1.5)
    # lane keepout: open-loop skid-steer drifts ~1.5 m laterally by the lane
    # end, and a big sphere's base spreads well beyond its centre
    if abs(y) < 3.0 and -1.5 < x < HALF:
        continue
    if math.hypot(x, y) < 1.6:                 # spawn keepout
        continue
    r = random.uniform(0.30, 0.55)
    if any(math.hypot(x - px, y - py) < r + pr + 0.8 for px, py, pr in placed):
        continue
    add_sphere(f"scenery_{count}", x, y, r, z=r * 0.55)
    placed.append((x, y, r))
    count += 1

# ---- perimeter ridge ----
n_rim = 36
for i in range(n_rim):
    ang = 2 * math.pi * i / n_rim
    rr = HALF + random.uniform(-0.2, 0.4)
    x, y = rr * math.cos(ang), rr * math.sin(ang)
    add_box(f"rim_{i}", x, y, random.uniform(0.5, 0.8),
            random.uniform(1.8, 2.8), random.uniform(1.0, 1.6),
            random.uniform(1.2, 2.0),
            yaw=ang + random.uniform(-0.4, 0.4),
            pitch=random.uniform(-0.08, 0.08), color=RIDGE)

g = color_str(GROUND)
world = f"""<?xml version="1.0"?>
<!-- Generated by gen_rough_world.py (seed 7) - edit the generator, not this file -->
<sdf version="1.8">
  <world name="rover_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <!-- DART (default). The rocker differential is enforced by the
         RockerDifferential system plugin, so bullet-featherstone (which
         cannot do controllable skid-steer) is no longer needed. -->
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
                   "worlds", "mars_rough_world.sdf")
with open(out, "w") as f:
    f.write(world)
print(f"wrote {out}: {len(models)} models ({count} scenery rocks, {n_rim} rim blocks)")
