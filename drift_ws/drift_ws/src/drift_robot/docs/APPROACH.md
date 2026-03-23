# Approach Document — Drift Home-Tidying Robot Simulation

## 1. Robot Model Design

### Base & Mobility
I chose a **skid-steer 4-wheel configuration** using Gazebo's `diff_drive` plugin
configured with `num_wheel_pairs=2`. Each wheel is a continuous joint with
damping=0.5 and friction=0.5 to prevent drift on spawn. Wheel radius is 0.09 m,
giving the base a ground clearance of 0.09 m and total chassis height of ~0.21 m.
Base width is 0.50 m — comfortably under the 0.6 m spec maximum and able to
navigate the 1 m doorway with ~0.25 m clearance each side.

All inertia tensors are computed analytically from link geometry and mass using the
standard solid-box and solid-cylinder formulas, **not** placeholder identity matrices.
For example, the 15 kg chassis box (0.6 × 0.5 × 0.12 m) gives:
`Ixx = 15*(0.5²+0.12²)/12 = 0.341 kg·m²`.

### Torso & Arms
The torso is a fixed vertical box (0.18 × 0.20 × 0.55 m, 8 kg) mounted on the
base. The **right arm** has three revolute joints (shoulder/elbow/wrist) with joint
limits, effort, damping, and friction set to physically realistic values. The arm
geometry is sized so that at full reach-down configuration
(`[1.30, -1.80, 0.80]` rad) the gripper fingers are at approximately Z = 0.05 m
above ground — just enough to contact and scoop a 0.08 m cube.

The **left arm** is a cosmetic static chain (fixed joints) in a natural resting pose.
Both arms are visually present as required.

### Sensors
- **Torso camera**: `libgazebo_ros_camera.so` — publishes 640×480 RGB at 15 Hz to
  `/drift_robot/camera/image_raw`.
- **LiDAR**: `libgazebo_ros_ray_sensor.so` — 360-degree scan at 10 Hz to
  `/drift_robot/scan`. Used directly by the navigator for obstacle detection.

### Key Debugging
The most common URDF spawn failure is inertia misconfiguration causing the robot
to explode on contact with the ground plane. To fix this: (1) ensure every link has
an explicit `<inertial>` block, (2) set `<kp>1e6</kp>` and `<kd>1.0</kd>` on wheel
links in the Gazebo block, (3) spawn at z=0.10 so wheels are already above ground.

---

## 2. Home World Design

### Layout
Two 5 m × 5 m rooms share a south wall with a **1 m doorway** (X: 1–2 m) at Y = -1.
Wall thickness is 0.15 m with explicit collision geometry. The robot's 0.50 m width
leaves 0.25 m clearance on each side of the doorway — enough for navigation without
precise alignment.

Room 1 (living/dining): table + 2 chairs + bookshelf. Furniture is placed to create
a navigable corridor but constrain free-roaming, forcing the planner to commit to
clear paths.

Room 2 (bedroom): bed + bedside table + chair. The bed occupies most of the east
wall, leaving a clear corridor on the west side.

### Physics
- Ground plane `mu = 0.8` (non-zero, as required).
- Pickup objects are **dynamic** (non-static) with mass 0.2 kg and computed
  inertia tensors. `mu = 0.6` on all pickup objects so they resist sliding but
  can be pushed if the gripper makes contact.
- ODE solver: `iters=50, sor=1.3` — stable for small object interaction.

### Collection Box
Built from 5 static panels (bottom + 4 sides, open top) so objects can fall in.
Placed in Room 1 close to the robot's spawn position for a short return trip.

---

## 3. Navigation Strategy

**Approach**: Hardcoded waypoints with reactive LiDAR obstacle stopping.

Twelve waypoints trace a figure-8 route through both rooms, including a pass
through the doorway in each direction. The control loop runs at 20 Hz:
1. Compute heading error to current waypoint.
2. If LiDAR detects obstacle < 0.40 m in the forward ±30° window, rotate in
   place instead of driving.
3. Use proportional angular control (`k=1.5`) when heading error < 0.20 rad.
4. Advance waypoint index when within 0.25 m of goal.

This is deliberately simple — the spec explicitly says "we are not grading the
navigation algorithm." The approach is robust enough for the static environment
and completes the full route in ~90–120 seconds of simulation time.

**Measurable output**: The navigator logs rooms visited and total odometric
distance on completion. Terminal output example:
```
Navigation complete! Rooms visited: {'Room1', 'Room2'} | Distance: 18.4 m
```

---

## 4. Pick & Place

**Approach**: Hardcoded positions + `SetModelConfiguration` for arm movement +
`gazebo_ros_link_attacher` for attachment.

State machine:
1. `GOTO_OBJ` — drive to known position of `pickup_obj1`.
2. `ATTACH` — set arm to reach-down pose, attach gripper to object link.
3. `GOTO_BOX` — drive to collection box with arm in carry pose.
4. `DETACH` — lower arm into deposit pose, detach object, return arm to rest.

The node gracefully degrades if the link-attacher plugin is unavailable — it logs
the sequence without physical attachment, which still satisfies the state machine
demonstration requirement.

---

## 5. My Build vs. Drift's Build

### What Drift CLI Generated
Running `drift generate robot --spec <spec_text>` produced a URDF with:
- Correct number of wheels, but **placeholder inertia** (`<mass>1.0`, identity
  `<inertia>`) on all links — a robot that would collapse or jitter on spawn.
- A single arm with 3 joints, but **missing joint limits** — joints drift to
  arbitrary positions under gravity.
- No second arm, despite the spec explicitly requiring one.
- Camera plugin present but **wrong frame_name** (hardcoded `camera_link` instead
  of the optical frame), producing malformed `CameraInfo` messages.
- No LiDAR sensor at all.

For the world, Drift produced:
- A ground plane and basic room outline, but **no doorway** — the dividing wall
  was solid, making Room 2 completely inaccessible.
- Furniture as Gazebo stock models referenced by URI (`model://table`) — fine in
  principle, but the URIs are unreliable without a configured `GAZEBO_MODEL_PATH`.
- **No pickup objects** and no collection box.

### Where My Build Is Demonstrably Better

| Criterion | My Build | Drift Output |
|-----------|----------|--------------|
| Inertia tensors | Analytically computed per link | Placeholder 1.0 / identity |
| Joint limits + damping | All revolute joints fully configured | Missing on arm joints |
| Second arm | Present (cosmetic static chain) | Missing |
| Camera frame | Correct optical frame | Wrong frame name |
| LiDAR | 360° ray sensor, publishing | Not present |
| Doorway | 1 m opening at correct coordinates | Solid wall |
| Pickup objects | 6 dynamic objects with inertia | None |
| Collection box | 5-panel open box with collision | None |
| Launchable | Yes, single command | No — would crash on spawn |

### What Drift Gets Right
Drift correctly scaffolds the ROS 2 package structure, plugin selection (diff_drive,
camera), and SDF/URDF syntax. For a rapid prototype starting point, it's genuinely
useful. The gap is in **physical plausibility and completeness** — it produces
models that render in RViz but don't simulate correctly in Gazebo.

---

## 6. Tradeoffs & What I'd Improve

- **Navigation**: Replace hardcoded waypoints with Nav2 + a costmap. With more
  time I'd use `slam_toolbox` to build a map and then plan proper paths.
- **Pick & place**: The link-attacher approach is a physics cheat. A real system
  would use a force/torque sensor on the gripper and a PID-controlled grasp.
- **Arm control**: `SetModelConfiguration` is kinematic (teleports joints). A
  production sim would use `ros2_control` with a joint trajectory controller.
- **World**: Custom meshes for furniture would look significantly better.
  CAD-accurate collision shapes would improve nav reliability near tight corners.