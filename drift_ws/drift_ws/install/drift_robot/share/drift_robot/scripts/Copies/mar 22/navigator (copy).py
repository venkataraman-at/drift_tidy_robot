#!/usr/bin/env python3
"""
Simple navigator. Robot spawns at (1.5, 2.5) facing south (-Y).
Goes straight down X=1.5 through doorway into Room 2, tours it, comes back.
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

# All waypoints at X=1.5 (centre of doorway).
# Robot just drives straight south then straight north.
WAYPOINTS = [
    (1.5,  1.5),   # south through Room 1
    (1.5,  0.3),   # approach doorway
    (1.5, -0.5),   # through doorway into Room 2
    (1.5, -2.5),   # Room 2 centre — near obj1
    (1.5, -4.5),   # Room 2 south
    (1.5, -0.5),   # back through doorway
    (1.5,  1.5),   # back in Room 1
    (0.5,  3.0),   # collection box
]

GOAL_TOL = 0.55
LIN      = 0.28
ANG      = 0.55


class Navigator(Node):
    def __init__(self):
        super().__init__("navigator")
        self.cmd  = self.create_publisher(Twist, "/drift_robot/cmd_vel", 10)
        self.done = self.create_publisher(Bool,  "/drift_robot/nav_done", 10)
        self.create_subscription(Odometry, "/drift_robot/odom", self._odom, 10)

        self.x = self.y = self.yaw = 0.0
        self.wp = 0
        self.finished = False
        self.dist = 0.0
        self._px = self._py = None
        self.rooms = set()

        self.create_timer(0.1, self._loop)
        self.get_logger().info("Navigator ready.")

    def _odom(self, m):
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y
        q = m.pose.pose.orientation
        self.yaw = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z))
        if self._px is not None:
            self.dist += math.hypot(self.x-self._px, self.y-self._py)
        self._px, self._py = self.x, self.y
        self.rooms.add("Room1" if self.y > -1.0 else "Room2")

    def _loop(self):
        if self.finished:
            return

        if self.wp >= len(WAYPOINTS):
            self._stop()
            self.finished = True
            m = Bool(); m.data = True
            self.done.publish(m)
            self.get_logger().info(
                "NAVIGATION COMPLETE! Rooms=%s Dist=%.1fm" % (self.rooms, self.dist))
            return

        gx, gy = WAYPOINTS[self.wp]
        dx = gx - self.x
        dy = gy - self.y
        dist = math.hypot(dx, dy)

        if dist < GOAL_TOL:
            self.get_logger().info("WP%d (%.1f,%.1f) reached" % (self.wp, gx, gy))
            self.wp += 1
            return

        target = math.atan2(dy, dx)
        err = math.atan2(math.sin(target - self.yaw), math.cos(target - self.yaw))

        cmd = Twist()
        if abs(err) > 0.25:
            cmd.angular.z = ANG * (1.0 if err > 0 else -1.0)
        else:
            cmd.linear.x  = LIN
            cmd.angular.z = 0.8 * err
        self.cmd.publish(cmd)

    def _stop(self):
        self.cmd.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Navigator())

if __name__ == "__main__":
    main()