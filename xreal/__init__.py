"""
xreal — read and fuse the XREAL One IMU stream.

Quickstart:

    from xreal import XrealOne, HeadTracker

    tracker = HeadTracker()
    with XrealOne() as dev:
        for ori in tracker.stream(dev):
            print(f"{ori.pitch:+.1f} {tracker.rel_yaw(ori):+.1f} {ori.roll:+.1f}")

Lower level:

    from xreal import XrealOne
    with XrealOne() as dev:
        for s in dev.samples():        # ImuSample: gyro rad/s, accel m/s^2
            ...

Note: the One is a 6-axis IMU (no magnetometer). Pitch/roll are drift-free;
yaw drifts slowly and is best zeroed on demand via HeadTracker.zero_yaw().
"""
from .device import XrealOne, DeviceError, DEFAULT_IP, DEFAULT_PORT
from .tracker import HeadTracker
from .fusion import MadgwickAHRS, Madgwick
from .types import ImuSample, Orientation
from .protocol import parse_frame, iter_frames, FRAME_LEN, HEADER, SENSOR_TAG
from .util import (wrap_deg, quat_rotate, forward_vector,
                   quat_conjugate, quat_mul, head_angles)

__version__ = "0.1.0"

__all__ = [
    "XrealOne", "DeviceError", "DEFAULT_IP", "DEFAULT_PORT",
    "HeadTracker", "MadgwickAHRS", "Madgwick",
    "ImuSample", "Orientation",
    "parse_frame", "iter_frames", "FRAME_LEN", "HEADER", "SENSOR_TAG",
    "wrap_deg", "quat_rotate", "forward_vector",
    "quat_conjugate", "quat_mul", "head_angles",
    "__version__",
]
