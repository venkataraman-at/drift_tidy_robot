#!/usr/bin/env python3
"""
mission_node.py — Single-object mission: pick up cyan cylinder and drop in collection box
==========================================================================================
Task  : Enter Room 2, collect pickup_cylinder_cyan_tall, return to collection box in Room 1.

World layout  (home.world)
---------------------------------------
  Room 1 : x=[-6,  2], y=[-3.5, 3.5]   warm oak floor
  Room 2 : x=[ 2, 10], y=[-3.5, 3.5]   cool concrete floor
  Doorway: x=2,  y=[-0.70, +0.70]  (1.4 m wide collision gap, centred y=0)
  Robot spawn : ~(0, 0)

Obstacle edges (exact from SDF, with robot radius 0.25 m noted):
  shared_wall_south : x=2.0, spans y=[-3.5,-0.70]  ← wall inner face x=1.9
  shared_wall_north : x=2.0, spans y=[+0.70,+3.5]  ← wall inner face x=1.9
  room1_south wall  : inner face y=-3.4
  room1_north wall  : inner face y=+3.4
  room1_west wall   : inner face x=-5.9
  room2_east wall   : inner face x=+9.9
  room1_chair1      : x=[-2.22,-1.78]  y=[0.33,0.77]
  room1_table       : x=[-2.70,-1.30]  y=[0.825,1.575]
  room1_chair2      : x=[-3.72,-3.28]  y=[0.98,1.42]   (rotated 90°)
  room1_sofa        : x=[-2.90,-1.10]  y=[-3.20,-2.48]
  room1_coffee_table: x=[-2.45,-1.55]  y=[-1.80,-1.30]
  room1_side_table  : x=[-1.10,-0.70]  y=[-3.04,-2.64]
  room1_tv_cabinet  : x=[-2.60,-1.40]  y=[3.12,3.50]
  room2_bed         : x=[7.80, 9.80]   y=[-0.80,+0.80]
  room2_desk (main) : x=[7.80, 9.20]   y=[2.50, 3.10]
  room2_rolling_chair: x=[8.26,8.74]   y=[1.61, 2.09]
  collection_box    : outer x=[-5.58,-4.42]  outer y=[-3.00,-2.02]

Cyan object pose: (3.2, -1.5)
Collection box centre: (-5.0, -2.5)

=== NAVIGATION PATH (hardcoded, obstacle-free) ===

OUTBOUND  spawn(0,0) → cyan(3.2,-1.5)
──────────────────────────────────────────────────
WP1  ( 1.4,  0.0)   Approach doorway dead-centre.  0.5 m west of shared-wall inner face.
                    y=0.0 is geometric centre of doorway gap [-0.70,+0.70]: ±0.45 m clearance.
WP2  ( 2.6,  0.0)   0.5 m east of shared-wall inner face — safely inside Room 2.
WP3  ( 3.2, -1.5)   Object location — open floor, no furniture within 1 m.

RETURN  cyan(3.2,-1.5) → collection_box(-5.0,-2.5)
──────────────────────────────────────────────────
WP1  ( 2.6,  0.0)   Mirror of outbound WP2 — start return through doorway.
WP2  ( 1.4,  0.0)   Back in Room 1, clear of shared wall.
WP3  ( 0.0,  2.2)   North corridor.  Clears:
                      dining table north face (y=1.575): +0.38 m gap ✓
                      chair2 north face       (y=1.42):  +0.53 m gap ✓
                      chair1 (east face x=-1.78): +1.53 m to east ✓
WP4  (-3.5,  2.2)   West along north corridor.  Clears:
                      TV cabinet south face (y=3.12): +0.67 m gap ✓
                      chair2 fully cleared to the south ✓
WP5  (-5.0,  2.2)   Far west at x=-5.0.
                      West wall inner face (x=-5.9): +0.65 m gap ✓
WP6  (-5.0, -2.0)   Descend south at x=-5.0 — entirely west of ALL Room 1 furniture.
                      Sofa west face (x=-2.90): +1.85 m clearance ✓
                      Coffee table west face (x=-2.45): +2.30 m clearance ✓
WP7  (-4.5, -2.3)   Drop position — inside collection box footprint.
                      Box east outer wall (x=-4.42): robot centre at -4.5 → east face -4.25 (inside) ✓
                      Box north outer wall (y=-2.02): robot centre at -2.3 → north face -2.05 (inside) ✓
                      West wall inner (x=-5.9): +1.15 m clearance ✓
                      South wall inner (y=-3.4): +0.85 m clearance ✓

ROS 2 : Humble
Gazebo: Ignition/Fortress — SetEntityState  (falls back to Classic SetModelState)
"""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from linkattacher_msgs.srv import AttachLink, DetachLink
from gazebo_msgs.msg import LinkStates

# ===========================================================================
# OBSTACLE BOUNDARIES  (centre ± half-extent, from home.world SDF)
# ===========================================================================
# Shared wall: x=2.0 ±0.1; collision gap y=[-0.70,+0.70] (doorway)
# room1_south wall inner face : y = -3.40
# room1_north wall inner face : y = +3.40
# room1_west  wall inner face : x = -5.90
# room2_east  wall inner face : x = +9.90
# room1_chair1      : x=[-2.22,-1.78]  y=[+0.33,+0.77]
# room1_table       : x=[-2.70,-1.30]  y=[+0.825,+1.575]
# room1_chair2(90°) : x=[-3.72,-3.28]  y=[+0.98,+1.42]
# room1_sofa        : x=[-2.90,-1.10]  y=[-3.20,-2.48]
# room1_coffee_table: x=[-2.45,-1.55]  y=[-1.80,-1.30]
# room1_side_table  : x=[-1.10,-0.70]  y=[-3.04,-2.64]
# room1_tv_cabinet  : x=[-2.60,-1.40]  y=[+3.12,+3.50]
# room2_bed         : x=[+7.80,+9.80]  y=[-0.80,+0.80]
# room2_desk(main)  : x=[+7.80,+9.20]  y=[+2.50,+3.10]
# room2_rolling_chair:x=[+8.26,+8.74]  y=[+1.61,+2.09]
# collection_box    : outer x=[-5.58,-4.42]  outer y=[-3.00,-2.02]

# ===========================================================================
# WAYPOINTS — verified against every obstacle listed above
# ===========================================================================

# ── OUTBOUND: spawn(0,0) → cyan object(3.2,-1.5) ──────────────────────────
#
# Doorway centre y=0.0  →  ±0.45 m from collision wall edge, ±0.70 m gap half-width
# No furniture in the east-side R1 corridor at y=0.0 and x in [−0.5, 2.0]
#   chair1 south face y=+0.33 → at y=0.0 clearance = 0.33 − 0.25(radius) = 0.08 m (marginal)
#   To be safe we go straight east at y=0.0 which is BELOW chair1 (chair1 y=[0.33,0.77]) ✓
#   side_table at y=[-3.04,-2.64] → y=0.0 is 2.64 m north of it ✓
_OUTBOUND_PATH = [
    ( 1.4,  0.0),   # WP1 — doorway approach: 0.5 m west of wall, y=0 centre
    ( 2.6,  0.0),   # WP2 — 0.5 m inside Room 2, clear of wall
    ( 3.2, -1.5),   # WP3 — cyan object (exact world pose)
]

# ── RETURN: cyan(3.2,-1.5) → collection_box(-5.0,-2.5) ───────────────────
#
# Strategy: reverse through doorway → arc to north corridor above ALL R1 furniture →
#           descend along the west wall (x=-5.0) → approach box from north/east
#
# y=2.2 north corridor clearances:
#   dining table north face y=1.575 → gap = 2.2−1.575−0.25 = +0.375 m ✓
#   chair2 north face       y=1.42  → gap = 2.2−1.42 −0.25 = +0.530 m ✓
#   TV cabinet south face   y=3.12  → gap = 3.12−2.2 −0.25 = +0.670 m ✓
#
# x=−5.0 south descent clearances:
#   sofa west face       x=−2.90 → gap = −2.90−(−5.0)−0.25 = +1.85 m ✓
#   coffee table west    x=−2.45 → gap = −2.45−(−5.0)−0.25 = +2.30 m ✓
#   west wall inner face x=−5.90 → gap = −5.0−(−5.9)−0.25  = +0.65 m ✓
#
# Drop position (−4.5, −2.3):
#   box east outer wall  x=−4.42 → robot east face −4.25, inside box footprint ✓
#   box north outer wall y=−2.02 → robot north face −2.05, inside box footprint ✓
#   west wall inner      x=−5.90 → gap = −4.5−(−5.9)−0.25  = +1.15 m ✓
#   south wall inner     y=−3.40 → gap = −2.3−(−3.4)−0.25  = +0.85 m ✓
_RETURN_PATH = [
    ( 2.6,  0.0),   # WP1 — mirror of outbound WP2, heading back to doorway
    ( 1.4,  0.0),   # WP2 — clear of shared wall, back in Room 1
    ( 0.0,  2.2),   # WP3 — northeast arc to clear ALL room-centre furniture
    (-3.5,  2.2),   # WP4 — west along north corridor, above chair2 & TV cabinet
    (-5.0,  2.2),   # WP5 — far-west staging, hugging west corridor
    (-5.0, -2.0),   # WP6 — south descent, west of every piece of furniture
    (-4.5, -2.3),   # WP7 — drop position inside collection box
]

# ===========================================================================
# COLLECTION BOX & OBJECT REGISTRY
# ===========================================================================
BOX_POS  = (-5.0, -2.5)   # box centre (reference)
DROP_POS = (-4.5, -2.3)   # robot stop position for drop — inside box footprint

# Single-object mission
OBJECTS = [
    (
        "pickup_cylinder_cyan_tall",
        (3.2, -1.5),        # exact world pose from SDF
        list(_OUTBOUND_PATH),
        True,               # is_room2
    ),
]

# ===========================================================================
# TUNING
# ===========================================================================
GOAL_TOL       = 0.40   # m — waypoint acceptance radius
GRAB_DIST      = 0.25   # m — tighter grab distance so robot is closer
LIN_SPEED      = 0.28   # m/s
ANG_SPEED      = 0.50   # rad/s
HEADING_THRESH = 0.25   # rad — rotate-in-place threshold
GRAB_PAUSE     = 25     # 10 Hz ticks = 2.5 s for arm to sweep down and grip
REACH_TICKS    = 25     # 10 Hz ticks = 2.5 s for arm to point up


# ===========================================================================
# MISSION NODE
# ===========================================================================
class MissionNode(Node):

    def __init__(self):
        super().__init__("mission_node")

        self.create_subscription(
            LinkStates,
            "/link_states",
            self._link_states_cb,
            10
        )

        self.gripper_pos = None
        self._arm_sent = False
        self.CENTER_TOL = 0.05

        # ── ROS interfaces ──────────────────────────────────────────────
        self.cmd_pub  = self.create_publisher(Twist, "/cmd_vel", 10)
        self.done_pub = self.create_publisher(Bool,  "/nav_done", 10)
        self.arm_pub  = self.create_publisher(
            JointTrajectory, "/set_joint_trajectory", 10)

        self.create_subscription(
            Odometry,   "/odom",              self._odom_cb,  10)
        self.create_subscription(
            LaserScan,  "/scan",               self._scan_cb,  10)

        # IFRA LinkAttacher services
        self._attach_cli = self.create_client(AttachLink, "/ATTACHLINK")
        self._detach_cli = self.create_client(DetachLink, "/DETACHLINK")
        self.create_timer(5.0, self._check_services)

        # ── Odometry / sensor state ──────────────────────────────────────
        self.x = self.y = self.yaw = 0.0
        self._odom_ready  = False
        self._odom_count  = 0
        self._prev_x = self._prev_y = None
        self.dist_total   = 0.0
        self.rooms_visited = set()

        # LiDAR for emergency brake
        self._front_min = float("inf")

        # ── Mission state ────────────────────────────────────────────────
        self._obj_idx     = 0
        self._wp_list     = []
        self._wp_idx      = 0
        # Phases: TO_OBJ → REACH_DOWN → GRAB_PAUSE → LIFT → TO_BOX → DROP_PAUSE → DONE
        self._phase       = "TO_OBJ"
        self._pause_ticks = 0
        self._picked      = []
        self._dropped     = []
        self._next_idx    = 0

        # Start the first object's outbound path
        self._begin_outbound(0)

        self.create_timer(0.10,  self._loop)
        self.create_timer(10.0,  self._status_log)

        self.get_logger().info(
            "MissionNode ready — waiting for odometry "
            f"({len(OBJECTS)} objects to collect)..."
        )

    # ===================================================================
    # Sensor callbacks
    # ===================================================================

    def _odom_cb(self, msg: Odometry):
        self.x   = msg.pose.pose.position.x
        self.y   = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        if self._prev_x is not None:
            self.dist_total += math.hypot(
                self.x - self._prev_x, self.y - self._prev_y)
        self._prev_x, self._prev_y = self.x, self.y

        self.rooms_visited.add("Room1" if self.x < 2.0 else "Room2")

        if not self._odom_ready:
            self._odom_count += 1
            if self._odom_count >= 20:
                self._odom_ready = True
                self.get_logger().info(
                    f"Odom live — start pos ({self.x:.2f},{self.y:.2f}). "
                    "Navigation enabled."
                )

    def _link_states_cb(self, msg):
        left = None
        right = None
        for i, name in enumerate(msg.name):
            if "right_finger_l_link" in name:
                pos = msg.pose[i].position
                left = (pos.x, pos.y, pos.z)
            elif "right_finger_r_link" in name:
                pos = msg.pose[i].position
                right = (pos.x, pos.y, pos.z)

        if left and right:
            gx = (left[0] + right[0]) / 2
            gy = (left[1] + right[1]) / 2
            gz = (left[2] + right[2]) / 2
            self.gripper_pos = (gx, gy, gz - 0.05)  # push DOWN toward jaw tips


    def _scan_cb(self, msg: LaserScan):
        ranges = [r for r in msg.ranges if math.isfinite(r) and r > 0.05]
        n = len(ranges)
        if not n:
            self._front_min = float("inf")
            return
        c  = n // 2
        hw = max(1, n // 12)
        window = ranges[max(0, c - hw): c + hw]
        self._front_min = min(window) if window else float("inf")

    # ===================================================================
    # Mission sequencing helpers
    # ===================================================================

    def _begin_outbound(self, idx: int):
        """Set up waypoint list to travel from current position to object[idx]."""
        self._obj_idx = idx
        name, pos, path, _ = OBJECTS[idx]
        self._wp_list = list(path)   # path already ends AT the object position
        self._wp_idx  = 0
        self._phase   = "TO_OBJ"
        self.get_logger().info(
            f"[MISSION] Starting outbound trip → "
            f"obj {idx+1}/{len(OBJECTS)}: '{name}' at {pos}"
        )

    def _begin_return(self):
        """Set up waypoint list to return to collection box after pickup."""
        self._wp_list = list(_RETURN_PATH)
        self._wp_idx = 0
        self._phase  = "TO_BOX"
        self.get_logger().info(
            f"[MISSION] Returning to collection box with "
            f"'{OBJECTS[self._obj_idx][0]}'"
        )

    # ===================================================================
    # Drive-to-waypoint
    # ===================================================================

    def _drive_to(self, gx: float, gy: float) -> bool:
        """Proportional heading + forward drive. Returns True when at goal."""
        dx   = gx - self.x
        dy   = gy - self.y
        dist = math.hypot(dx, dy)

        if dist < GOAL_TOL:
            self._stop()
            return True

        target_hdg = math.atan2(dy, dx)
        err = math.atan2(
            math.sin(target_hdg - self.yaw),
            math.cos(target_hdg - self.yaw),
        )

        cmd = Twist()

        # Emergency brake if LiDAR detects imminent obstacle
        if self._front_min < 0.35 and abs(err) < 0.5:
            # Rotate toward open space instead of braking completely
            cmd.angular.z = ANG_SPEED * (1.0 if err >= 0 else -1.0)
            self.cmd_pub.publish(cmd)
            return False

        if abs(err) > HEADING_THRESH:
            # Rotate in place to face waypoint
            cmd.angular.z = ANG_SPEED * (1.0 if err > 0 else -1.0)
        else:
            if dist < 1.0:
                cmd.linear.x = 0.12   # slow near target (fix jitter)
            else:
                cmd.linear.x = 0.28
            cmd.angular.z = max(-ANG_SPEED, min(ANG_SPEED, 1.6 * err))

        self.cmd_pub.publish(cmd)
        return False

    def _stop(self):
        self.cmd_pub.publish(Twist())

    # ===================================================================
    # Gazebo teleport  (Ignition-first, Classic fallback)
    # ===================================================================

    def _check_services(self):
        self.get_logger().info(
            f"ATTACHLINK ready: {self._attach_cli.service_is_ready()}, "
            f"DETACHLINK ready: {self._detach_cli.service_is_ready()}")

    def _attach(self, obj_name: str):
        """Weld object link to right_gripper_link."""
        req = AttachLink.Request()
        req.model1_name = "drift_robot"
        req.link1_name = "right_finger_l_link"
        req.model2_name = obj_name
        req.link2_name  = "link"
        self._attach_cli.call_async(req)
        self.get_logger().info(f"[ATTACH] Welding '{obj_name}' to gripper.")

    def _snap_object_to_gripper(self, obj_name):
        if self.gripper_pos is None:
            return
        gx, gy, gz = self.gripper_pos

        from gazebo_msgs.srv import SetEntityState
        from gazebo_msgs.msg import EntityState

        client = self.create_client(SetEntityState, "/set_entity_state")
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("SetEntityState not available")
            return

        req = SetEntityState.Request()
        state = EntityState()
        state.name = obj_name
        state.pose.position.x = gx
        state.pose.position.y = gy
        state.pose.position.z = gz          # no extra lift — sit between the jaws
        state.pose.orientation.w = 1.0
        req.state = state
        client.call_async(req)

    def _detach(self, obj_name: str):
        """Release object from gripper."""
        req = DetachLink.Request()
        req.model1_name = "drift_robot"
        req.link1_name = "right_finger_l_link"
        req.model2_name = obj_name
        req.link2_name  = "link"
        self._detach_cli.call_async(req)
        self.get_logger().info(f"[DETACH] Releasing '{obj_name}' from gripper.")

    # ===================================================================
    # Arm control  — sends JointTrajectory to /arm_trajectory
    # Joints: right_shoulder_joint, right_elbow_joint, right_wrist_joint,
    #         right_finger_l_joint, right_finger_r_joint
    # ===================================================================

    def _arm_pose(self, shoulder, elbow, wrist, finger_l, finger_r, secs=1):
        """Send a single joint trajectory point to the arm."""
        traj = JointTrajectory()
        
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.header.frame_id = "base_footprint"
        
        traj.joint_names = [
            "right_shoulder_joint",
            "right_elbow_joint",
            "right_wrist_joint",
            "right_finger_l_joint",
            "right_finger_r_joint",
        ]
        pt = JointTrajectoryPoint()
        pt.positions = [
            float(shoulder),
            float(elbow),
            float(wrist),
            float(finger_l),
            float(finger_r),
        ]
        pt.time_from_start = Duration(sec=secs, nanosec=0)
        traj.points = [pt]
        self.arm_pub.publish(traj)

    def _arm_reach_down(self):
        """Reach deeply toward object"""
        self._arm_pose(
            0.7,    # more forward
            1.0,    # elbow DOWN (very important)
            1.3,    # wrist DOWN
            0.05, 0.05,
            secs=2
        )

    def _arm_grip(self):
        """Close gripper while slightly dipping."""
        self._arm_pose(
            0.5,
            -0.6,
            0.3,
            0.0, 0.0,
            secs=2
        )

    def _arm_carry(self):
        """Lift object upward (boxing-style elbow up carry)."""
        self._arm_pose(
            0.9,    # shoulder forward/up
            -1.4,   # strong elbow bend (THIS gives "elbow up")
            0.7,    # wrist support
            0.0, 0.0,
            secs=2
        )

    def _arm_drop(self):
        """Sweep arm down over box and open fingers to release."""
        self._arm_pose(0.0, 1.2, 1.5, 0.05, 0.05, secs=2)

    def _arm_stow(self):
        """Return to spring rest pose — arm horizontal at side."""
        self._arm_pose(0.0, 0.0, 0.0, 0.0, 0.0, secs=2)

    # ===================================================================
    # Status logger
    # ===================================================================

    def _status_log(self):
        if self._phase == "DONE":
            return
        if self._obj_idx < len(OBJECTS):
            obj_name = OBJECTS[self._obj_idx][0]
        else:
            obj_name = "—"
        self.get_logger().info(
            f"[STATUS] phase={self._phase}  obj={self._obj_idx+1}/{len(OBJECTS)} "
            f"({obj_name})  wp={self._wp_idx}/{len(self._wp_list)}  "
            f"pos=({self.x:.2f},{self.y:.2f})  yaw={math.degrees(self.yaw):.1f}°  "
            f"dist={self.dist_total:.1f}m  rooms={self.rooms_visited}  "
            f"front_lidar={self._front_min:.2f}m  "
            f"picked={len(self._picked)}  dropped={len(self._dropped)}"
        )

    # ===================================================================
    # Main control loop  (10 Hz)
    # ===================================================================

    def _loop(self):
        if not self._odom_ready or self._phase == "DONE":
            return

        # ── TO_OBJ — follow waypoints toward current object ────────────
        if self._phase == "TO_OBJ":
            if self._wp_idx >= len(self._wp_list):
                self._stop()
                self.get_logger().info("[TO_OBJ] Waypoints complete — reaching down.")
                
                self._phase       = "REACH_DOWN"
                self._pause_ticks = REACH_TICKS
                return

            gx, gy = self._wp_list[self._wp_idx]

            _, obj_pos, _, _ = OBJECTS[self._obj_idx]

            if self.gripper_pos is not None:
                gxg, gyg, _ = self.gripper_pos
                ox, oy = obj_pos

                grip_dist = math.hypot(ox - gxg, oy - gyg)

                self.get_logger().info(f"[MAGNET] dist={grip_dist:.3f}")

                if grip_dist < 0.18:

                    # Transform error into robot frame
                    dx_world = ox - gxg
                    dy_world = oy - gyg

                    dx = math.cos(-self.yaw) * dx_world - math.sin(-self.yaw) * dy_world
                    dy = math.sin(-self.yaw) * dx_world + math.cos(-self.yaw) * dy_world

                    # --- ALIGNMENT BEFORE ATTACH ---
                    if abs(dx) > self.CENTER_TOL:
                        cmd = Twist()
                        cmd.angular.z = max(-0.3, min(0.3, 2.0 * dx))
                        self.cmd_pub.publish(cmd)
                        return

                    if abs(dy) > self.CENTER_TOL:
                        cmd = Twist()
                        cmd.linear.x = max(-0.05, min(0.05, 0.8 * dy))
                        self.cmd_pub.publish(cmd)
                        return

                    # --- ALIGNED → ATTACH ---
                    self._stop()

                    name = OBJECTS[self._obj_idx][0]
    
                    self.get_logger().info(f"[MAGNET] Centered → attaching '{name}'")
    
                    self._arm_reach_down()

                    # SNAP object into gripper BEFORE attach
                    self._snap_object_to_gripper(name)

                    self._attach(name)
    
                    self._picked.append(name)

                    self._arm_carry()

                    self._phase = "LIFT"
                    self._pause_ticks = REACH_TICKS
                    return
                
            if self._drive_to(gx, gy):
                self.get_logger().info(
                    f"[TO_OBJ] WP{self._wp_idx} ({gx:.1f},{gy:.1f}) reached.")

                self._wp_idx += 1
            
        # ── REACH_DOWN — wait for arm to extend before gripping ────────
        elif self._phase == "REACH_DOWN":

            if not self._arm_sent:
                self._arm_reach_down()
                self._arm_sent = True

            self._pause_ticks -= 1

            if self._pause_ticks <= 0:
                self._arm_sent = False

                self.get_logger().info("[REACH_DOWN] Arm extended — closing gripper.")
                self._arm_grip()

                self._phase       = "GRAB_PAUSE"
                self._pause_ticks = GRAB_PAUSE

        # ── GRAB_PAUSE — fingers closing, then attach ──────────────────
        elif self._phase == "GRAB_PAUSE":
            self._pause_ticks -= 1

            if self._pause_ticks <= 0:
                name, obj_pos, _, _ = OBJECTS[self._obj_idx]

                # Check if robot is close enough
                if self.gripper_pos is None:
                    self.get_logger().warn("No gripper position yet")
                    return

                gx, gy, gz = self.gripper_pos
                ox, oy = obj_pos

                dist = math.sqrt((gx - ox)**2 + (gy - oy)**2)
                self.get_logger().info(f"[GRAB] dist={dist:.3f}")

                if dist < 0.15:
                    self._attach(name)
                    self._picked.append(name)

                    # Send carry pose ONCE
                    self._arm_carry()

                    self._phase = "LIFT"
                    self._pause_ticks = REACH_TICKS

                else:
                    self.get_logger().warn(
                        f"[GRAB] Too far ({dist:.2f}m). Retrying reach.")

                    # Go back and try again instead of failing silently
                    self._arm_sent = False
                    self._phase = "REACH_DOWN"
                    self._pause_ticks = REACH_TICKS

        # ── LIFT — wait for arm to raise before driving ─────────────────
        elif self._phase == "LIFT":
            self._pause_ticks -= 1
            if self._pause_ticks <= 0:
                self.get_logger().info(
                    f"[LIFT] Arm raised — heading to collection box.")
                self._begin_return()

        # ── TO_BOX — carry object back to collection box ───────────────
        elif self._phase == "TO_BOX":
            if self._wp_idx >= len(self._wp_list):
                self._stop()
                self.get_logger().info("[TO_BOX] At box — lowering arm to drop.")
                self._arm_drop()
                self._phase       = "DROP_PAUSE"
                self._pause_ticks = 25   # 2.5 s for arm to lower
                return

            gx, gy = self._wp_list[self._wp_idx]
            if self._drive_to(gx, gy):
                self.get_logger().info(
                    f"[TO_BOX] WP{self._wp_idx} ({gx:.1f},{gy:.1f}) reached.")
                self._wp_idx += 1

        # ── DROP_PAUSE — arm lowered, release object ────────────────────
        elif self._phase == "DROP_PAUSE":
            self._pause_ticks -= 1
            if self._pause_ticks <= 0:
                name = OBJECTS[self._obj_idx][0]
                self._detach(name)
                self._dropped.append(name)
                self.get_logger().info(
                    f"[DROP] '{name}' released at collection box. "
                    f"Dropped {len(self._dropped)}/{len(OBJECTS)}.")
                self._arm_stow()

                next_idx = self._obj_idx + 1
                if next_idx < len(OBJECTS):
                    # Brief pause for arm to stow before next outbound
                    self._phase       = "STOW_PAUSE"
                    self._pause_ticks = 20
                    self._next_idx    = next_idx
                else:
                    self._mission_complete()

        # ── STOW_PAUSE — wait for arm to return to rest ─────────────────
        elif self._phase == "STOW_PAUSE":
            self._pause_ticks -= 1
            if self._pause_ticks <= 0:
                self._begin_outbound(self._next_idx)

    # ===================================================================
    # Mission complete
    # ===================================================================

    def _mission_complete(self):
        self._stop()
        self._phase = "DONE"
        sep = "=" * 60
        self.get_logger().info(
            f"\n{sep}\n"
            f"  MISSION COMPLETE\n"
            f"  Objects collected : {len(self._dropped)}/{len(OBJECTS)}\n"
            f"  Rooms visited     : {self.rooms_visited}\n"
            f"  Distance driven   : {self.dist_total:.1f} m\n"
            f"  Picked   : {self._picked}\n"
            f"  Dropped  : {self._dropped}\n"
            f"{sep}"
        )
        msg = Bool()
        msg.data = True
        self.done_pub.publish(msg)


# ===========================================================================
# ENTRY POINT
# ===========================================================================

def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()