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
