"""Data types shared across the xreal package."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ImuSample:
    """One raw IMU reading from the XREAL One (SI units)."""
    t: float          # seconds since stream start
    gx: float         # gyro x, rad/s
    gy: float         # gyro y, rad/s
    gz: float         # gyro z, rad/s
    ax: float         # accel x, m/s^2
    ay: float         # accel y, m/s^2
    az: float         # accel z, m/s^2
    temp: float       # sensor temperature, deg C

    @property
    def gyro(self):
        return (self.gx, self.gy, self.gz)

    @property
    def accel(self):
        return (self.ax, self.ay, self.az)

    @property
    def accel_mag(self):
        return math.sqrt(self.ax ** 2 + self.ay ** 2 + self.az ** 2)

    @property
    def valid(self):
        """False for the all-zero warm-up frames the device emits on connect."""
        return self.accel_mag > 0.5


@dataclass(frozen=True)
class Orientation:
    """A fused orientation estimate at a point in time.

    Angles are absolute (gravity-referenced for pitch/roll). Yaw has no
    absolute reference on the One — use HeadTracker.rel_yaw() for a zeroed,
    usable heading.
    """
    t: float
    quat: tuple       # (w, x, y, z)
    pitch: float      # degrees
    yaw: float        # degrees (absolute, drifts slowly)
    roll: float       # degrees
    temp: float       # deg C
    is_still: bool     # device currently stationary (ZUPT engaged)
