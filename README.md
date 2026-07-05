# xreal

Head tracking for the [XREAL One](https://eu.shop.xreal.com/en-de/products/xreal-one) — reads the glasses' IMU stream and fuses it into a stable, drift-corrected 6-axis orientation. Pure Python standard library, no dependencies.

## About

The XREAL One streams raw inertial data over its USB-C link. This library connects to that stream, decodes it, and turns it into usable head-orientation (pitch, yaw, roll, and quaternion) at 1 kHz — a foundation for spatial interfaces, head-driven input, and motion analysis.

The wire protocol was reverse-engineered from a physical device. The glasses provide a 6-axis IMU (gyroscope and accelerometer, no magnetometer): pitch and roll are gravity-referenced and drift-free, while yaw drift is continuously suppressed through automatic zero-velocity bias tracking.

## Features

- Direct connection to the XREAL One IMU stream — no vendor SDK required
- 6-axis Madgwick AHRS fusion with quaternion and Euler output
- Automatic gyro-bias tracking to minimise yaw drift
- Layered API: raw samples, the filter in isolation, or high-level orientation
- Pure standard library, Python 3.8+

## Requirements

- XREAL One, connected via USB-C
- A link-local network interface to the device
- Python 3.8 or newer

## Protocol

Fixed 134-byte frames at ~1 kHz, little-endian:

| Offset | Field | Unit |
|-------:|-------|------|
| 34 | Gyroscope X, Y, Z | rad/s |
| 46 | Accelerometer X, Y, Z | m/s² |
| 70 | Temperature | °C |

## License

MIT
