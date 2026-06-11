#!/usr/bin/env python3
"""Trajectory generators for the ambulance scenarios.

Each factory returns a callable  f(t) -> (x, y, yaw)  representing the
displacement from the ambulance's starting position at time t (seconds).

ROS FLU convention: +x is forward, +y is left, yaw is rotation about +z (up).

Edit the SCENARIOS dict at the bottom to adjust speeds and parameters.
"""

import math


def straight(speed: float = 5.0):
    """Constant-velocity straight line along +x."""
    def f(t: float):
        return speed * t, 0.0, 0.0
    return f


def circle(speed: float = 5.0, radius: float = 20.0):
    """Constant-speed circular arc.

    Positive radius curves left (CCW yaw), negative radius curves right.
    """
    def f(t: float):
        omega = speed / radius      # angular rate (rad/s)
        angle = omega * t
        x = radius * math.sin(angle)
        y = radius * (1.0 - math.cos(angle))
        return x, y, angle
    return f


def accel_decel(max_speed: float = 10.0, accel: float = 2.0, cruise_time: float = 5.0):
    """Trapezoidal velocity profile along +x.

    Ramp up to max_speed → cruise → ramp down to 0 → hold position.
    """
    t_ramp = max_speed / accel              # seconds to reach max_speed
    d_ramp = 0.5 * accel * t_ramp ** 2     # distance covered per ramp

    t1 = t_ramp
    t2 = t1 + cruise_time
    t3 = t2 + t_ramp

    x_cruise_end = d_ramp + max_speed * cruise_time
    x_final = x_cruise_end + d_ramp

    def f(t: float):
        if t < t1:                          # accelerating
            x = 0.5 * accel * t ** 2
        elif t < t2:                        # cruising
            x = d_ramp + max_speed * (t - t1)
        elif t < t3:                        # decelerating
            dt = t - t2
            x = x_cruise_end + max_speed * dt - 0.5 * accel * dt ** 2
        else:                               # stopped
            x = x_final
        return x, 0.0, 0.0
    return f


# Named, pre-configured scenarios.  Edit parameters here to tune each run.
# The trajectory_publisher selects one by name via --ros-args -p scenario:=<key>.
SCENARIOS = {
    "straight":    straight(speed=22.4),
    "circle":      circle(speed=22.4, radius=40.0),
    "accel_decel": accel_decel(max_speed=26.82, accel=2.682, cruise_time=5.0),  # 0-60 mph in 10 s
}
