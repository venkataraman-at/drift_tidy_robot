#!/usr/bin/env python3
"""Pick place is now handled by navigator.py mission node."""
import rclpy

def main(args=None):
    rclpy.init(args=args)
    # No-op — mission handled by navigator.py
    rclpy.shutdown()

if __name__ == "__main__":
    main()