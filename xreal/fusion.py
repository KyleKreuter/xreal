"""
6-axis Madgwick AHRS + automatic gyro-bias tracking (ZUPT) for the XREAL One.

The One has gyro + accel only (no magnetometer, verified by RE), so:
  * pitch & roll are gravity-referenced -> drift-free
  * yaw is gyro-integrated only         -> drifts slowly

Yaw drift is fought by (1) Madgwick quaternion fusion and (2) zero-velocity
bias tracking: while the device rests still the true gyro bias is continuously
re-estimated and subtracted. Measured effect: yaw drift ~0.15 deg/s -> ~0.05.
"""
import math


class MadgwickAHRS:
    def __init__(self, beta=0.04, bias_gain=0.006):
        self.q = [1.0, 0.0, 0.0, 0.0]       # orientation quaternion (w,x,y,z)
        self.beta = beta                     # fusion gain (accel trust)
        self.bias = [0.0, 0.0, 0.0]          # tracked gyro bias (rad/s)
        self.bias_gain = bias_gain           # bias adaptation speed when still
        self._still_ctr = 0
        self._initialized = False

    # -- state control -------------------------------------------------------
    def reset(self, keep_bias=True):
        """Reset orientation to identity. Keeps the learned bias by default."""
        self.q = [1.0, 0.0, 0.0, 0.0]
        self._initialized = False
        if not keep_bias:
            self.bias = [0.0, 0.0, 0.0]
        self._still_ctr = 0

    @property
    def is_still(self):
        return self._still_ctr > 50

    def init_from_accel(self, ax, ay, az):
        """Snap the quaternion so measured gravity aligns to +Z, avoiding a
        multi-second settle when the device starts in a tilted rest pose."""
        n = math.sqrt(ax * ax + ay * ay + az * az)
        if n < 1e-6:
            return
        vx, vy, vz = ax / n, ay / n, az / n
        d = vz
        if d < -0.999999:
            self.q = [0.0, 1.0, 0.0, 0.0]
        else:
            s = math.sqrt((1 + d) * 2)
            self.q = [s / 2, vy / s, -vx / s, 0.0]
        self._initialized = True

    # -- core ----------------------------------------------------------------
    def _stationary(self, gx, gy, gz, amag):
        return (gx * gx + gy * gy + gz * gz) < (0.06 ** 2) and abs(amag - 9.81) < 0.6

    def update(self, gx, gy, gz, ax, ay, az, dt):
        if not self._initialized:
            self.init_from_accel(ax, ay, az)

        gx -= self.bias[0]; gy -= self.bias[1]; gz -= self.bias[2]
        amag = math.sqrt(ax * ax + ay * ay + az * az)

        # ZUPT: adapt bias toward raw gyro while stationary
        if self._stationary(gx, gy, gz, amag):
            self._still_ctr = min(self._still_ctr + 1, 1000)
            if self._still_ctr > 50:
                g = self.bias_gain
                self.bias[0] += g * gx
                self.bias[1] += g * gy
                self.bias[2] += g * gz
        else:
            self._still_ctr = 0

        q0, q1, q2, q3 = self.q

        if amag > 1e-6:
            ax_, ay_, az_ = ax / amag, ay / amag, az / amag
            f0 = 2.0 * (q1 * q3 - q0 * q2) - ax_
            f1 = 2.0 * (q0 * q1 + q2 * q3) - ay_
            f2 = 2.0 * (0.5 - q1 * q1 - q2 * q2) - az_
            s0 = -2.0 * q2 * f0 + 2.0 * q1 * f1
            s1 = 2.0 * q3 * f0 + 2.0 * q0 * f1 - 4.0 * q1 * f2
            s2 = -2.0 * q0 * f0 + 2.0 * q3 * f1 - 4.0 * q2 * f2
            s3 = 2.0 * q1 * f0 + 2.0 * q2 * f1
            n = math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
            if n > 0:
                s0 /= n; s1 /= n; s2 /= n; s3 /= n
        else:
            s0 = s1 = s2 = s3 = 0.0

        qd0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz) - self.beta * s0
        qd1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy) - self.beta * s1
        qd2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx) - self.beta * s2
        qd3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx) - self.beta * s3

        q0 += qd0 * dt; q1 += qd1 * dt; q2 += qd2 * dt; q3 += qd3 * dt
        n = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        self.q = [q0 / n, q1 / n, q2 / n, q3 / n]
        return self.q

    def euler(self):
        """Quaternion -> (pitch, yaw, roll) in degrees (aerospace ZYX)."""
        w, x, y, z = self.q
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        s = max(-1.0, min(1.0, 2 * (w * y - z * x)))
        pitch = math.asin(s)
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return math.degrees(pitch), math.degrees(yaw), math.degrees(roll)


# backwards-compatible alias
Madgwick = MadgwickAHRS
