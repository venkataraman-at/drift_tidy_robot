#!/usr/bin/env python3
"""
Reliable pick and place.
- Waits for nav_done
- Drives to object in Room 2
- Snaps object to robot (carry loop keeps it glued)
- Drives to box in Room 1
- Drops object in box
- Moves arm visually during each phase
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

OBJ_NAME = "pickup_obj1"
OBJ_POS  = (1.5, -2.5)
BOX_POS  = (0.5,  3.0)
GOAL_TOL = 0.35
SPEED    = 0.28

ARM_JOINTS = ["r_shoulder_joint", "r_elbow_joint", "r_wrist_joint"]
ARM_REST   = [0.40, -1.20, 0.80]   # V shape
ARM_REACH  = [1.10, -1.90, 0.90]   # reaching down
ARM_CARRY  = [0.30, -0.90, 0.50]   # carrying up


class PickPlace(Node):
    def __init__(self):
        super().__init__("pick_place")
        self.cmd_pub = self.create_publisher(Twist, "/drift_robot/cmd_vel", 10)
        self.arm_pub = self.create_publisher(
            JointTrajectory, "/drift_robot/arm_trajectory", 10)
        self.create_subscription(Odometry, "/drift_robot/odom", self._odom, 10)
        self.create_subscription(Bool, "/drift_robot/nav_done", self._nav_done, 10)
        self.sms = self.create_client(SetModelState, "/gazebo/set_model_state")

        self.x = self.y = self.yaw = 0.0
        self.active  = False
        self.state   = "WAIT"
        self.carrying = False
        self._carry_t = None

        self.create_timer(0.05, self._loop)
        self.create_timer(3.0, lambda: self._arm(ARM_REST, 2))
        self.get_logger().info("PickPlace waiting...")

    def _odom(self, m):
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def _nav_done(self, m):
        if m.data and not self.active:
            self.active = True
            self.state  = "GOTO_OBJ"
            self.get_logger().info("Nav done — starting pick and place!")

    def _arm(self, pos, secs=1):
        t = JointTrajectory()
        t.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in pos]
        pt.velocities = [0.0]*3
        pt.time_from_start = Duration(sec=secs, nanosec=0)
        t.points = [pt]
        self.arm_pub.publish(t)

    def _move(self, name, x, y, z):
        if not self.sms.wait_for_service(timeout_sec=0.5):
            return
        req = SetModelState.Request()
        ms  = ModelState()
        ms.model_name = name
        ms.pose.position.x = float(x)
        ms.pose.position.y = float(y)
        ms.pose.position.z = float(z)
        ms.pose.orientation.w = 1.0
        ms.reference_frame = "world"
        req.model_state = ms
        self.sms.call_async(req)

    def _drive_to(self, gx, gy):
        dx, dy = gx-self.x, gy-self.y
        if math.hypot(dx, dy) < GOAL_TOL:
            self._stop(); return True
        desired = math.atan2(dy, dx)
        err = math.atan2(math.sin(desired-self.yaw), math.cos(desired-self.yaw))
        cmd = Twist()
        if abs(err) > 0.3:
            cmd.angular.z = 0.6*(1.0 if err>0 else -1.0)
        else:
            cmd.linear.x  = SPEED
            cmd.angular.z = 1.5*err
        self.cmd_pub.publish(cmd)
        return False

    def _stop(self): self.cmd_pub.publish(Twist())

    def _start_carry(self):
        self.carrying = True
        if self._carry_t: self._carry_t.cancel()
        self._carry_t = self.create_timer(0.05, self._carry_tick)

    def _stop_carry(self):
        self.carrying = False
        if self._carry_t:
            self._carry_t.cancel()
            self._carry_t = None

    def _carry_tick(self):
        # Keep object glued just in front of robot
        cx = math.cos(self.yaw); sx = math.sin(self.yaw)
        ox = self.x + 0.15*cx
        oy = self.y + 0.15*sx
        self._move(OBJ_NAME, ox, oy, 0.32)

    def _delay_then(self, nxt, sec):
        self.state = "_WAIT"
        def cb(): self.state = nxt; t.cancel()
        t = self.create_timer(sec, cb)

    def _loop(self):
        if not self.active: return

        if self.state == "GOTO_OBJ":
            if self._drive_to(*OBJ_POS):
                self.get_logger().info("At object — grabbing!")
                self._arm(ARM_REACH, 2)
                self._delay_then("GRAB", 2.0)

        elif self.state == "GRAB":
            # Snap to robot
            cx = math.cos(self.yaw); sx = math.sin(self.yaw)
            self._move(OBJ_NAME, self.x+0.15*cx, self.y+0.15*sx, 0.32)
            self._arm(ARM_CARRY, 2)
            self._start_carry()
            self.get_logger().info("Object grabbed — heading to box!")
            self._delay_then("GOTO_BOX", 2.0)

        elif self.state == "GOTO_BOX":
            if self._drive_to(*BOX_POS):
                self.get_logger().info("At box — dropping!")
                self._stop_carry()
                self._move(OBJ_NAME, BOX_POS[0], BOX_POS[1], 0.08)
                self._arm(ARM_REST, 2)
                self._delay_then("DONE", 1.0)

        elif self.state == "DONE":
            self.get_logger().info("Done! Object delivered to box.")
            self._stop()


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PickPlace())

if __name__ == "__main__":
    main()