#!/usr/bin/env python3
"""Abstract trajectory publisher for the ambulance.

Publishes geometry_msgs/PoseStamped on /ambulance/trajectory.  The pose is a
displacement from the ambulance's starting position; the Unity subscriber
applies it relative to the ambulance's spawn transform.

Trajectory logic and parameters live in trajectories.py.  Edit that file to
tune speeds, radii, etc.  This node only controls publishing mechanics.

Usage
-----
    python3 scripts/trajectory_publisher.py
    python3 scripts/trajectory_publisher.py --ros-args -p scenario:=circle
    python3 scripts/trajectory_publisher.py --ros-args -p scenario:=accel_decel -p publish_rate:=50.0

Parameters
----------
scenario     : str   – key from trajectories.SCENARIOS  (default: straight)
topic        : str   – ROS topic to publish on          (default: /ambulance/trajectory)
frame_id     : str   – header frame                     (default: map)
publish_rate : float – Hz                               (default: 20.0)
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from trajectories import SCENARIOS

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__("trajectory_publisher")

        self.declare_parameter("scenario", "straight")
        self.declare_parameter("topic", "/ambulance/trajectory")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_rate", 20.0)

        scenario_name = self.get_parameter("scenario").value
        topic = self.get_parameter("topic").value
        self.frame_id = self.get_parameter("frame_id").value
        publish_rate = self.get_parameter("publish_rate").value

        if scenario_name not in SCENARIOS:
            self.get_logger().fatal(
                f"Unknown scenario '{scenario_name}'. "
                f"Valid choices: {list(SCENARIOS.keys())}"
            )
            raise SystemExit(1)

        self.trajectory = SCENARIOS[scenario_name]

        self.publisher = self.create_publisher(PoseStamped, topic, 10)
        self.start_time = self.get_clock().now()
        self.create_timer(1.0 / publish_rate, self.timer_callback)

        self.get_logger().info(
            f"TrajectoryPublisher ready: scenario='{scenario_name}' "
            f"on '{topic}' at {publish_rate} Hz"
        )

    def timer_callback(self):
        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds * 1e-9

        x, y, yaw = self.trajectory(t)

        msg = PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
