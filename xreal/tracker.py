"""High-level head tracking: raw samples -> fused Orientation stream."""
from .fusion import MadgwickAHRS
from .types import Orientation
from .util import wrap_deg


class HeadTracker:
    """Feed ImuSamples, get Orientation out. Also drives a whole device stream.

        tracker = HeadTracker()
        with XrealOne() as dev:
            for ori in tracker.stream(dev):
                print(ori.pitch, tracker.rel_yaw(ori), ori.roll)
    """

    def __init__(self, **filter_kwargs):
        self.ahrs = MadgwickAHRS(**filter_kwargs)
        self.yaw_zero = 0.0
        self._last_t = None

    # -- feed one sample -----------------------------------------------------
    def feed(self, sample):
        """Advance the filter by one sample; returns an Orientation."""
        if self._last_t is None:
            self._last_t = sample.t
            self.ahrs.update(sample.gx, sample.gy, sample.gz,
                             sample.ax, sample.ay, sample.az, 0.0)
        else:
            dt = sample.t - self._last_t
            self._last_t = sample.t
            self.ahrs.update(sample.gx, sample.gy, sample.gz,
                             sample.ax, sample.ay, sample.az, dt)
        pitch, yaw, roll = self.ahrs.euler()
        return Orientation(sample.t, tuple(self.ahrs.q), pitch, yaw, roll,
                           sample.temp, self.ahrs.is_still)

    # -- drive a device ------------------------------------------------------
    def stream(self, device, **samples_kwargs):
        """Yield Orientation for every sample from a connected XrealOne."""
        for sample in device.samples(**samples_kwargs):
            yield self.feed(sample)

    # -- reference handling --------------------------------------------------
    def zero_yaw(self, orientation=None):
        """Set the current yaw as the zero heading."""
        if orientation is not None:
            self.yaw_zero = orientation.yaw
        else:
            self.yaw_zero = self.ahrs.euler()[1]

    def rel_yaw(self, orientation):
        """Yaw relative to the last zero, wrapped to (-180, 180]."""
        return wrap_deg(orientation.yaw - self.yaw_zero)

    def reset(self, keep_bias=True):
        """Reset orientation and yaw reference."""
        self.ahrs.reset(keep_bias=keep_bias)
        self.yaw_zero = 0.0
        self._last_t = None

    @property
    def bias_dps(self):
        """Current tracked gyro bias magnitude in deg/s."""
        import math
        return math.sqrt(sum(b * b for b in self.ahrs.bias)) * 57.29578
