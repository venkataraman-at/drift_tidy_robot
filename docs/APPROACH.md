# Approach Document — Drift Home-Tidying Robot (Gazebo + ROS 2)

This note describes how the simulated mobile manipulator and home world were built, how navigation and pick-and-place work, tradeoffs, debugging lessons, and what we would extend with more time.

---

## 1. Robot model (`drift_robot.urdf.xacro`)

**Geometry and dynamics**  
The model is authored by hand in Xacro using primitive shapes (boxes, cylinders, spheres) only—no external mesh files. Mass and inertia use macros (`box_inertia`, `cylinder_inertia`) with standard formulas so every link has physically consistent tensors rather than placeholder values.

**Mobility**  
The platform is a four-wheel layout with continuous wheel joints (radius 0.08 m, separation 0.48 m). ROS 2 Humble does not ship `libgazebo_ros_skid_steer_drive.so`, so **two** `libgazebo_ros_diff_drive.so` plugins are used (front and rear axle), both driven by `/cmd_vel`. Only the front plugin publishes `/odom` and the `odom` → `base_footprint` transform to avoid duplicates. Wheel contacts use high stiffness/damping (`kp`, `kd`) and friction for stable ground contact.

**Manipulator**  
The right arm has three revolute joints (shoulder, elbow, wrist) plus two prismatic finger joints. Gazebo `implicitSpringDamper` blocks define a rest “V” pose at startup. The left arm is a fixed cosmetic chain so the robot meets a two-arm visual requirement without extra controllers.

**Sensors**  
- RGB camera on the torso: `libgazebo_ros_camera.so`, remapped to `/camera/image_raw` and `/camera/camera_info`, with `frame_name` set to `camera_optical_link` for valid `CameraInfo`.  
- 2D LiDAR: `libgazebo_ros_ray_sensor.so` on `lidar_link`, 720 samples over ±90°, publishing `sensor_msgs/LaserScan` on `/scan`.

**Arm command path**  
Joint targets are sent as `trajectory_msgs/JointTrajectory` to **`/arm_trajectory`**, which matches the Gazebo plugin remapping (`~/in:=arm_trajectory`) for `libgazebo_ros_joint_pose_trajectory.so`.

*(Third-party components and attribution: **§8**.)*

---

## 2. Home world (`home.world`)

**Layout**  
Two adjacent rooms are defined as **8 m × 7 m** floor regions: Room 1 centered at (−2, 0) and Room 2 at (6, 0), sharing a partition at **x = 2** with a **1.2 m wide doorway** at **y = 0** (split wall segments plus lintel and door-frame visuals). Walls use explicit collision boxes; floors use separate warm (Room 1) and cool (Room 2) materials. Lighting mixes directional “sun” with warm point lights in Room 1 and cooler tones in Room 2 for readable contrast.

**Objects and collection**  
Pickup items are **dynamic** SDF models (cylinders, spheres, boxes) with explicit mass and inertia comments in-file. A static **collection region** is placed in Room 1 (south-west area; coordinates aligned with mission waypoints). The world loads `libgazebo_ros_state.so` and **`libgazebo_link_attacher.so`** so the mission node can weld/release objects to the gripper fingers via services.

**Physics**  
ODE is configured with moderate solver iterations and contact parameters; the ground plane uses non-zero friction (`mu` 0.8) as required for credible wheeled motion.

---

## 3. Navigation and mission strategy (`navigator.py`)

**High-level behavior**  
The mission node combines **hand-tuned waypoint lists** with **reactive LiDAR**: outbound waypoints drive from spawn toward the configured object in Room 2; a return path brings the robot back to the drop pose near the box.

**Control loop**  
At 10 Hz it steers toward the current waypoint using heading error: turn in place when error is large, otherwise drive forward with a simple proportional yaw correction. For the final approach with the arm down, a **creep** mode uses low linear speed. Room visitation is inferred from odometry (`x < 2` vs `≥ 2`).

**LiDAR**  
The minimum range in a **narrow forward window** around the scan center is tracked. If something is closer than ~0.35 m while heading is roughly aligned, the robot rotates instead of driving forward—enough to avoid head-on collisions with walls or the box on the last segment.

**Pick and place**  
Phases: reach down (trajectory) → creep using **`/link_states`** finger positions vs known object XY → close fingers → **`AttachLink`** to **`right_forearm_link`** (gripper centre after Gazebo merges fixed gripper geometry into the forearm; see **§5.4**) → carry pose → return waypoints → drop pose → **`DetachLink`** → stow. Completion logs distance, rooms, and pick/drop lists and publishes `/nav_done`.

---

## 4. Tradeoffs

| Choice | Why | Cost |
|--------|-----|------|
| Waypoints + LiDAR gating | Fast to tune, deterministic in a static world | No global optimality, no replanning around arbitrary clutter |
| Two diff-drive plugins | Works on Humble without skid-steer plugin | Must carefully disable duplicate odom/TF on one axle |
| Link attacher for grasp | Reliable in sim for “carry object” demos | Not a physical grasp model |
| Implicit springs on arm | Natural idle pose, simple | Trajectories must be **re-sent** periodically or springs pull the arm away from hold poses |
| Primitives-only URDF | No asset pipeline, portable | Less realistic appearance than meshes |

---

## 5. Debugging challenges (simulation experience)

This section is intentionally concrete: what broke, how it showed up in sim or ROS, and what changed in code or URDF.

### 5.1 Arm trajectory never reached the controller

**Symptom:** The arm stayed in its default “V” rest pose no matter what the node published; RViz / topic echo showed messages, but Gazebo joints did not track.

**Diagnosis:** The URDF uses `libgazebo_ros_joint_pose_trajectory.so` with `<remapping>~/in:=arm_trajectory</remapping>`, so the plugin subscribes to **`/arm_trajectory`**. The node had been publishing to a different topic (e.g. `/set_joint_trajectory`), which nothing in Gazebo was subscribed to.

**Resolution:** Publish `trajectory_msgs/JointTrajectory` only to **`/arm_trajectory`**, matching the plugin remapping in `drift_robot.urdf.xacro`.

### 5.2 Grasp / attach happened in mid-air

**Symptom:** `AttachLink` ran while the gripper was visibly above or beside the object; the object jumped or welded in the wrong place.

**Diagnosis:** The “grip” step reused a trajectory that also retargeted shoulder, elbow, and wrist. That moved the TCP away from the creep pose **before** the attach service fired.

**Resolution:** Split behaviors: **reach-down** sets arm pose; **grip** closes prismatic fingers **only**, keeping shoulder/elbow/wrist identical to reach-down; **carry** runs only after attach.

### 5.3 Implicit arm springs vs hold poses

**Symptom:** After a single trajectory point, the arm slowly drifted back toward the rest pose during long waits (creep, pause before detach).

**Diagnosis:** Gazebo `implicitSpringDamper` + `springReference` on the arm joints pulls joints toward a default configuration; one-shot trajectories do not permanently “win” against that bias over many seconds.

**Resolution:** On a timer during CREEP, GRAB_PAUSE, LIFT, and DROP_PAUSE, **re-send** the same joint trajectory so the commanded pose is refreshed and the springs do not dominate.

### 5.4 Which link to attach (fixed joints and Gazebo lumping)

**Symptom:** Attach service calls to `right_gripper_link` failed or behaved inconsistently.

**Diagnosis:** In URDF, `right_gripper_link` is connected with a **fixed** joint to the forearm. In Gazebo Classic, fixed children are often **merged** into the parent rigid body, so `right_gripper_link` may not exist as a distinct physics link.

**Resolution:** Attach to **`right_forearm_link`**. After merging, the forearm body represents the palm region centred between the fingers; a single `AttachLink` welds the object to that rigid body. Finger link names are still used for **`/link_states`** to estimate gripper centre for the creep phase.

### 5.5 Final approach to the collection box

**Symptom:** Robot drove into the box front wall or oscillated on the last waypoint.

**Diagnosis:** Waypoints are coarse; odometry tolerance can leave the nose pointed at the box; forward motion then violates clearance.

**Resolution:** On the **last** return waypoint, if forward LiDAR range drops below ~0.40 m, **stop**, command drop pose, `DetachLink`, and stow—treating the scan as a virtual “bump” into the box region.

### 5.6 Robot explode / jitter on spawn or first contact

**Symptom:** Violent motion, interpenetration, or flying links at startup or when wheels touch the ground.

**Diagnosis:** Missing or inconsistent `<inertial>` blocks (e.g. identity inertia with wrong mass), weak wheel contact (`kp`/`kd`), or spawn height jamming wheels into the plane.

**Resolution:** Analytic box/cylinder inertia for every link; wheel contacts with high `kp` and `kd`; spawn pose with a small positive **z** in `simulation.launch.py` so the model settles onto the plane instead of tunneling.

### 5.7 Duplicate odometry / TF from four driven wheels

**Symptom:** Double transforms, conflicting `/odom`, or unstable TF trees when using two diff-drive plugins.

**Diagnosis:** Two plugins both publishing odometry and `odom` → `base` would duplicate the same transform.

**Resolution:** Front axle plugin: `publish_odom` and `publish_odom_tf` **true**. Rear axle plugin: both **false**, same `cmd_vel` and wheel parameters so both axles still apply torque.

---

## 6. What we would improve with more time

- **Nav2** (or similar) with a costmap and planner for generic goals instead of fixed polylines.  
- **`ros2_control`** and a proper trajectory controller instead of Gazebo-only joint pose trajectory + springs.  
- **Perception-driven grasp**: segment object pose from `/camera/image_raw` instead of fixed world coordinates.  
- **Richer world**: mesh furniture, tighter collision aligned to visuals, and multiple objects in `OBJECTS` with scheduling.  
- **Skid-steer or mecanum** plugin if/when available for the target distro, or a single plugin that officially supports four driven wheels with one odom source.

---

## 7. My Build vs. Drift AI’s Build — Side-by-Side Comparison

**Drift AI** = representative incomplete scaffold (Ignition-oriented plugins, coordinate mismatches, etc.); product may change. **My build** = this repo (`home.world`, `drift_robot.urdf.xacro`, `navigator.py`). *World has 12 pickup models; `OBJECTS` can list one or more for multi-object missions.*

### 1. World SDF

| Drift AI | My build |
|----------|----------|
| **1.1 Room geometry & wall gap** — Room 1 (−2, 0), x ∈ [−6, 2]. Room 2 **(7, 0)**, x ∈ [3, 11]. Shared wall x = 2 → **1 m gap** (2–3). N/S walls **8 m**, corners don’t seal. Wall height **2.5 m**. | **1.1** — Room 1 unchanged. Room 2 **(6, 0)**, x ∈ **[2, 10]**, **zero gap** at x = 2. Walls **8.4 m** with corner overlap. Height **2.6 m**. Door frame: left post, right post, top beam (visuals). |
| **1.2 Lighting** — 1 sun + 1 fill; both rooms flat grey, no mood split. | **1.2** — Warm-shift sun; R1 warm overhead + accent; R2 cool overhead + accent + warm bedside; **5 lights** total. |
| **1.3 Materials** — Walls ambient+diffuse only; flat floors; cream everywhere; no emissive; no baseboards/trim. | **1.3** — ambient+diffuse+**specular**+**emissive**; R1 oak floor sheen, R2 concrete specular; terracotta / slate-blue accents; **6** baseboards; **3-piece** door trim. |
| **1.4 Furniture** — R1 table+chair+shelf boxes; R2 table+chair+couch; chairs = seat+back+4 legs; **6** models. | **1.4** — R1: dining set (aprons, legs), 2 detailed chairs, sofa, coffee table, side table, TV cabinet; R2: L-desk, rolling chair, bed+mattress+duvet+pillows+headboard, bedside cabinet; **10** assemblies, **80+** sub-visuals. |
| **1.5 Collection box** — SDF ~(−4, 0) vs nav **(0.5, 1.5)**; ~0.6³ m; plain yellow. | **1.5** — SDF **(−5.0, −2.5)**; `BOX_POS` / `DROP_POS` match; ~**1.2×1.0×0.28 m** class; corner caps, label band, commented clearance. |
| **1.6 Pickups** — **6** objects; weak inertia; little bounce; random-ish placement. | **1.6** — **12** objects; inertia notes in SDF; sphere restitution ~0.25–0.75; tight/corridor/behind-furniture placement; some yaw. |
| **1.7 Physics** — ODE quick 50 iters; no SOR; sparse constraints/contacts. | **1.7** — ODE quick **50**, **SOR 1.3**, dynamic MoI rescaling; CFM/ERP/contact layers/min_depth tuned; magnetic field in world. |

### 2. Robot URDF

| Drift AI | My build |
|----------|----------|
| **2.1 Arm embed** — All joints 0 → arms through torso/base; tips below ground. | **2.1** — V-pose springs: shoulder **0.8**, elbow **−1.4**, wrist **0.6** rad; **`self_collide=false`** where needed. |
| **2.2 Camera / face** — Panel and camera ~**2.75 cm** apart on torso front. | **2.2** — Camera z **0.30**, LiDAR **0.47**, head panel **0.58** (torso frame); ~**28 cm** camera–display gap. |
| **2.3 Arms** — Two mirrored Xacro arms, “actuated” but **no** trajectory plugin; JSP often skips arm joints. | **2.3** — Right: 3-DOF + 2 fingers, blue joint markers, palm yaw for outward jaws; left cosmetic fixed chain; **`libgazebo_ros_joint_pose_trajectory.so`** → **`/arm_trajectory`**; JSP lists **9** joints. |
| **2.4 Drive** — Ignition-style 4-wheel diff; geometry for Fortress, not Classic. | **2.4** — **`libgazebo_ros_diff_drive.so` ×2** (front/rear), **`/cmd_vel`**; front only **/odom**+TF; track **0.48 m**, dia **0.16 m**, wheels z=0 in `base_link`; notes on missing skid-steer on Humble. |
| **2.5 Inertia** — `box_inertia` without CoM offset; visual/inertial mismatch risk. | **2.5** — `box_inertia` **cx,cy,cz**; CoM matches visuals (e.g. torso **cz=0.26**). |
| **2.6 Sensors** — e.g. 30 Hz cam, Ignition; **360°** GPU LiDAR + noise. | **2.6** — Cam **640×480**, **15 Hz**, ~**70°** HFOV, `libgazebo_ros_camera.so` + remaps; LiDAR **±90°**, **720** samples, **12 Hz**, `libgazebo_ros_ray_sensor.so` → **`/scan`**. |

### 3. Navigation code

| Drift AI | My build |
|----------|----------|
| **3.1 Architecture** — Large **C++** node, `NavState` enum, one `control_loop()`, pixel-style scan, slow iterate-build. | **3.1** — **Python** `navigator.py` ~**560** lines; phases **TO_OBJ→…→DONE**; link-attacher services; **`/link_states`**; service check every **5 s**. |
| **3.2 Joint names** — e.g. `shoulder_joint` vs `right_shoulder_joint` → silent ignore. | **3.2** — `right_shoulder_joint`, `right_elbow_joint`, `right_wrist_joint`, `right_finger_l_joint`, `right_finger_r_joint`. |
| **3.3 Topics** — e.g. `/drift_robot/set_joint_trajectory` with **no** URDF subscriber. | **3.3** — Publish **`/arm_trajectory`** ↔ `~/in:=arm_trajectory`. |
| **3.4 Waypoints** — e.g. “R2 centre” **(3,0)** vs SDF centre **(7,0)**; home in wall. | **3.4** — Out **(1.4,0)→(2.6,0)→(3.2,−1.5)**; return through door + R1 to SW box; object coords match SDF. |
| **3.5 Box coords** — SDF vs nav **~4.7 m** error. | **3.5** — `BOX_POS`/`DROP_POS` match SDF; LiDAR **~0.40 m** box-wall early stop. |
| **3.6 Pick/place** — Wrong topic/names; `holding_object_` on timer; **no** physics attach. | **3.6** — `AttachLink`/`DetachLink` (IFRA); creep to **0.20 m** (finger midpoints via `/link_states`); re-send traj every **10** ticks (~**1 s**); attach **`right_forearm_link`**. |
| **3.7 Springs** — No URDF springs; one-shot arm cmds. | **3.7** — Spring **stiffness 50**; periodic reach/grip/carry/drop resend; grip = **fingers only**, same arm angles as reach-down. |
| **3.8 Obstacles** — e.g. ±**30°**, **0.55 m**, **0.6 rad/s** turn, L/R compare. | **3.8** — Forward ~**±1/12** of scan (~**±15°**); **0.35 m**; **TO_BOX** **0.40 m** stop; active out+return. |

### Summary (side by side)

| Drift AI (issues) | My build (fixes) |
|-------------------|------------------|
| **World:** 1 m wall gap; 8 m walls; 6 pickups; box/nav mismatch; flat materials; minimal lights; basic furniture. | **World:** R2 x∈[2,10]; 8.4 m walls; **12** pickups; box **SDF↔nav**; full material channels; 5-lamp mood; 10 detailed furniture sets. |
| **URDF:** arms at 0 embed; camera/face clash; dual non-functional arms; Ignition drive; no CoM in inertia; Ignition sensors. | **URDF:** V-pose + self_collide; camera/LiDAR/display stack; right arm + `/arm_trajectory`; Classic dual diff_drive + one odom; CoM inertia; Classic cam + front LiDAR. |
| **Nav:** C++ monolith; wrong joint/topic names; bad waypoints/box; fake grasp; no spring fight; coarse LiDAR avoid. | **Nav:** Python phases; matching names/`/arm_trajectory`; waypoints+box aligned SDF; IFRA attach + creep + **`right_forearm_link`**; ~1 s traj refresh; tighter cone + box stop. |

| Drift AI still useful for | My build caveat |
|---------------------------|-----------------|
| Package skeleton, first link names, plugin *ideas*. | Must re-validate **Classic vs Ignition**, **world↔nav**, **remaps**, and **sim-ready URDF** by hand; authoritative files live in `drift_robot`. |

---

## 8. Sources and third-party assets (explicit attribution)

| What | Source | Notes |
|------|--------|--------|
| **Robot link geometry** | **Original** | Built from URDF/Xacro primitives (boxes, cylinders, spheres). **No** downloaded robot models, **no** CAD/mesh imports for the base description. |
| **Gazebo + ROS 2 plugins** | **[ros-simulation/gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs)** (and OS packages on Ubuntu for ROS 2 Humble) | `libgazebo_ros_diff_drive.so`, `libgazebo_ros_camera.so`, `libgazebo_ros_ray_sensor.so`, `libgazebo_ros_joint_state_publisher.so`, `libgazebo_ros_joint_pose_trajectory.so`, `libgazebo_ros_state.so` — standard bridge plugins, not custom binaries. |
| **Link attacher** | **[IFRA LinkAttacher](https://github.com/ifra-cranfield/ifra_linkattacher)** — ROS 2 packages `ros2_linkattacher` and `linkattacher_msgs` (IFRA-Cranfield / Cranfield University; author Mikel Bueno Viso; **Apache-2.0**). Vendored under `drift_ws/src/IFRA_LinkAttacher/` (C++ plugin sources live in folder **`ros2_LinkAttacher/`**; ROS package name remains **`ros2_linkattacher`**). | World file loads `libgazebo_link_attacher.so`; `navigator.py` calls `AttachLink` / `DetachLink` from `linkattacher_msgs`. This is **not** original code; it is third-party simulation glue for rigid “welding” in Gazebo Classic. |
| **Materials in Gazebo** | **Built-in Gazebo material scripts** | References such as `Gazebo/White`, `Gazebo/Blue`, `Gazebo/Black`, `Gazebo/Grey`, `Gazebo/FlatBlack` ship with Gazebo’s media/materials. |
| **Home world layout** | **Original SDF** | Walls, floors, furniture blocks, lights, and pickup models are hand-written in `home.world` (primitive geometries). **No** `model://` dependency on the Gazebo model database for core layout. |
| **Optional scaffolding** | **Drift CLI** (if used) | Any generated starter URDF/world was **replaced or heavily edited**; this submission’s model and world are defined by the `drift_robot` package files, not a verbatim Drift export. |

If graders need a one-line statement: **all robot and environment geometry in this submission is original primitive-based SDF/URDF; third-party components are (1) upstream `gazebo_ros` plugins and built-in Gazebo materials, and (2) the IFRA LinkAttacher stack (`ros2_linkattacher`, `linkattacher_msgs`) for attach/detach in simulation.**