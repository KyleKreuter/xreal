#!/usr/bin/env python3
"""Print raw IMU samples (gyro, accel, temp) — the lowest-level view.

    python3 examples/raw_dump.py
"""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from xreal import XrealOne, DeviceError


def main():
    try:
        with XrealOne() as dev:
            n = 0
            for s in dev.samples():
                n += 1
                if n % 100 == 0:
                    print(
                        f"\rt={s.t:6.1f}s  "
                        f"gyro[{s.gx:+.3f} {s.gy:+.3f} {s.gz:+.3f}] rad/s  "
                        f"accel[{s.ax:+6.2f} {s.ay:+6.2f} {s.az:+6.2f}] m/s²  "
                        f"|a|={s.accel_mag:5.2f}  {s.temp:4.1f}°C",
                        end="", flush=True,
                    )
    except DeviceError as e:
        print("NO-GO:", e)
    except KeyboardInterrupt:
        print("\nBye.")


if __name__ == "__main__":
    main()
