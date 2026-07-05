"""Connection + streaming for the XREAL One IMU."""
import socket
import time

from .protocol import iter_frames, parse_frame

DEFAULT_IP = "169.254.2.1"
DEFAULT_PORT = 52998


class DeviceError(RuntimeError):
    pass


class XrealOne:
    """TCP connection to the One's IMU stream.

    Use as a context manager and iterate samples:

        with XrealOne() as dev:
            for s in dev.samples():
                print(s.gyro, s.accel)
    """

    def __init__(self, ip=DEFAULT_IP, port=DEFAULT_PORT, timeout=5.0,
                 recv_size=8192):
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.recv_size = recv_size
        self.sock = None
        self._t0 = None

    # -- lifecycle -----------------------------------------------------------
    def connect(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(self.timeout)
            s.connect((self.ip, self.port))
        except OSError as e:
            raise DeviceError(
                f"Cannot reach XREAL One at {self.ip}:{self.port} ({e}). "
                f"Check the glasses are plugged in and the 169.254.2.x "
                f"link-local interface is up."
            ) from e
        self.sock = s
        self._t0 = time.monotonic()
        return self

    def close(self):
        if self.sock:
            self.sock.close()
            self.sock = None

    def __enter__(self):
        return self.connect()

    def __exit__(self, *exc):
        self.close()
        return False

    # -- streaming -----------------------------------------------------------
    def frames(self):
        """Yield raw (unparsed) frames."""
        if not self.sock:
            raise DeviceError("not connected")
        yield from iter_frames(lambda: self.sock.recv(self.recv_size))

    def samples(self, skip_invalid=True, warmup=0):
        """Yield ImuSample objects.

        skip_invalid: drop the all-zero warm-up frames emitted on connect.
        warmup:       additionally skip this many leading valid samples.
        """
        skipped = 0
        for frame in self.frames():
            s = parse_frame(frame, t=time.monotonic() - self._t0)
            if s is None:
                continue
            if skip_invalid and not s.valid:
                continue
            if skipped < warmup:
                skipped += 1
                continue
            yield s
