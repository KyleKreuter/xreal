"""Small math helpers for consumers (viz, head-mouse, gestures)."""
import math


def wrap_deg(a):
    """Wrap an angle to (-180, 180]."""
    while a > 180:
        a -= 360
    while a < -180:
        a += 360
    return a


def quat_rotate(q, v):
    """Rotate 3-vector v by quaternion q=(w,x,y,z)."""
    w, x, y, z = q
    vx, vy, vz = v
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def forward_vector(q):
    """Unit vector the device points along (its local +Z), in world frame."""
    return quat_rotate(q, (0.0, 0.0, 1.0))


def quat_conjugate(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def quat_mul(a, b):
    """Hamilton product a ⊗ b."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def head_angles(q, q_ref=None):
    """Decompose an orientation into head (azimuth, elevation, roll) in degrees.

    Uses the XREAL One head frame determined by reverse-engineering the mount:
    up = sensor +Y, right = +X, forward = +Z. Unlike Euler angles this does not
    cross-couple — a pure nod changes only elevation, not roll.

    Pass q_ref (e.g. captured on "recenter") to get angles relative to that
    reference pose; the reference then reads (0, 0, 0).
    """
    if q_ref is not None:
        q = quat_mul(quat_conjugate(q_ref), q)
    fx, fy, fz = quat_rotate(q, (0.0, 0.0, 1.0))   # forward
    ux, uy, uz = quat_rotate(q, (0.0, 1.0, 0.0))   # up
    az = math.degrees(math.atan2(fx, fz))
    el = math.degrees(math.atan2(fy, math.hypot(fx, fz)))
    right = _cross((0.0, 1.0, 0.0), (fx, fy, fz))
    rn = math.sqrt(_dot(right, right))
    if rn > 1e-9:
        right = (right[0] / rn, right[1] / rn, right[2] / rn)
        realup = _cross((fx, fy, fz), right)
        roll = math.degrees(math.atan2(_dot((ux, uy, uz), right),
                                       _dot((ux, uy, uz), realup)))
    else:
        roll = 0.0
    return az, el, roll
