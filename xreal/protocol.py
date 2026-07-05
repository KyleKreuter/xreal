"""
Wire protocol for the XREAL One IMU stream.

Reverse-engineered live from a real XREAL One (non-Pro) on 2026-07-05.
The framing differs from the One *Pro* demo it is often ported from.

Frame layout (fixed 134 bytes, ~1 kHz, little-endian):

    offset 0   : header      27 36 00 00 00 80   (Pro uses 28 36 …)
    offset 34  : gyro  x,y,z  float32   rad/s
    offset 46  : accel x,y,z  float32   m/s^2   (gravity ~9.6)
    offset 58  : field ?A     — NOT motion data (bit-packed, ignore)
    offset 62  : field ?B     — NOT motion data (bit-packed, ignore)
    offset 66  : field ?C     — constant -3200 sentinel
    offset 70  : temperature  float32   deg C
    offset 78  : sensor tag   00 40 1f 00 00 40   (frame anchor)

Verified: fields 58/62/66 show ZERO correlation with heading over a full
rotation -> the One has NO magnetometer (6-axis IMU only).
"""
import struct

from .types import ImuSample

HEADER = bytes.fromhex("273600000080")
SENSOR_TAG = bytes.fromhex("00401f000040")
FRAME_LEN = 134

OFF_GYRO = 34      # 3x float32
OFF_ACCEL = 46     # 3x float32
OFF_TEMP = 70      # 1x float32
TAG_OFFSET = 78


def parse_frame(frame: bytes, t: float = 0.0):
    """Parse one raw frame into an ImuSample, or return None if malformed."""
    if len(frame) < FRAME_LEN or frame.find(SENSOR_TAG) != TAG_OFFSET:
        return None
    try:
        gx, gy, gz = struct.unpack("<fff", frame[OFF_GYRO:OFF_GYRO + 12])
        ax, ay, az = struct.unpack("<fff", frame[OFF_ACCEL:OFF_ACCEL + 12])
        temp = struct.unpack("<f", frame[OFF_TEMP:OFF_TEMP + 4])[0]
    except struct.error:
        return None
    return ImuSample(t, gx, gy, gz, ax, ay, az, temp)


def iter_frames(recv):
    """Yield raw frames from a byte source.

    `recv` is a zero-arg callable returning bytes (e.g. ``sock.recv``-bound
    with a size, or any producer). Returns when it yields empty bytes.
    """
    buf = b""
    while True:
        chunk = recv()
        if not chunk:
            return
        buf += chunk
        while True:
            h = buf.find(HEADER)
            if h == -1:
                buf = buf[-4:] if len(buf) > 4 else buf  # keep partial header
                break
            nxt = buf.find(HEADER, h + 1)
            if nxt == -1:
                buf = buf[h:]
                break
            yield buf[h:nxt]
            buf = buf[nxt:]
