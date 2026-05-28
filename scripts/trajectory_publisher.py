#!/usr/bin/env python3
"""Publishes a constant-velocity-forward trajectory for the ambulance.

Standalone ROS2 node (no colcon package needed). Run after sourcing your ROS2
workspace:

    python3 scripts/trajectory_publisher.py
    python3 scripts/trajectory_publisher.py --ros-args -p velocity:=2.0

It emits a geometry_msgs/PoseStamped whose position.x grows as velocity * t
(+x is forward in the ROS FLU convention) and whose yaw grows as
angular_velocity * t (rotation about +z / up). The Unity
AmbulanceTrajectorySubscriber treats this pose as a displacement from the
ambulance's starting position.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__("trajectory_publisher")

        self.declare_parameter("velocity", -5.0)         # m/s, forward (+x)
        self.declare_parameter("angular_velocity", 0.2)   # rad/s, yaw (about +z)
        self.declare_parameter("publish_rate", 20.0)      # Hz
        self.declare_parameter("topic", "/ambulance/trajectory")
        self.declare_parameter("frame_id", "map")

        self.velocity = self.get_parameter("velocity").value
        self.angular_velocity = self.get_parameter("angular_velocity").value
        self.frame_id = self.get_parameter("frame_id").value
        topic = self.get_parameter("topic").value
        publish_rate = self.get_parameter("publish_rate").value

        self.publisher = self.create_publisher(PoseStamped, topic, 10)
        self.start_time = self.get_clock().now()
        self.create_timer(1.0 / publish_rate, self.timer_callback)

        self.get_logger().info(
            f"Publishing PoseStamped on '{topic}' at {publish_rate} Hz, "
            f"velocity {self.velocity} m/s (+x forward), "
            f"angular_velocity {self.angular_velocity} rad/s (yaw)."
        )

    def timer_callback(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9  # seconds

        msg = PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = self.velocity * elapsed
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.0

        # Constant yaw about +z (up in FLU): quaternion for rotation `yaw` about z.
        yaw = self.angular_velocity * elapsed
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
