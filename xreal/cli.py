"""Console head-tracker CLI: `python -m xreal`."""
import sys
import time
import select
import contextlib

from .device import XrealOne, DeviceError
from .tracker import HeadTracker

try:
    import termios
    import tty
    _HAVE_TTY = True
except ImportError:
    _HAVE_TTY = False


@contextlib.contextmanager
def _raw_keyboard():
    if not (_HAVE_TTY and sys.stdin.isatty()):
        yield lambda: None
        return
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield lambda: (sys.stdin.read(1)
                       if select.select([sys.stdin], [], [], 0)[0] else None)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _bar(v, mx=45.0, w=21):
    n = max(-1.0, min(1.0, v / mx))
    c = w // 2
    row = ["-"] * w
    row[c] = "|"
    row[int(c + n * (c - 1))] = "●"
    return "".join(row)


def main():
    tracker = HeadTracker()
    try:
        dev = XrealOne().connect()
    except DeviceError as e:
        print("NO-GO:", e)
        return 1
    print("GO. Madgwick 6-axis + ZUPT.  [space] zero  [r] reset  [q] quit\n")

    n = 0
    t0 = time.monotonic()
    try:
        with _raw_keyboard() as key:
            for ori in tracker.stream(dev):
                k = key()
                if k in ("q", "\x03"):
                    break
                elif k == " ":
                    tracker.zero_yaw(ori); print("\n[yaw zeroed]")
                elif k == "r":
                    tracker.reset(); print("\n[reset]")
                n += 1
                if n % 100 == 0:
                    ry = tracker.rel_yaw(ori)
                    rate = n / (time.monotonic() - t0)
                    print(
                        f"\rP {ori.pitch:+6.1f}[{_bar(ori.pitch)}] "
                        f"Y {ry:+6.1f}[{_bar(ry)}] "
                        f"R {ori.roll:+6.1f}[{_bar(ori.roll)}] "
                        f"{ori.temp:4.1f}°C bias{tracker.bias_dps:4.2f}°/s "
                        f"{'STILL' if ori.is_still else '     '} {rate:4.0f}Hz",
                        end="", flush=True,
                    )
    except KeyboardInterrupt:
        pass
    finally:
        print("\nBye.")
        dev.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
