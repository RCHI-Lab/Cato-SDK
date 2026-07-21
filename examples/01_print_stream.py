"""Print the live IMU stream (decimated to ~10 Hz so it's readable).

Usage: python examples/01_print_stream.py
"""

from cato import Cato


def main() -> None:
    with Cato() as cato:
        print("Connected. Streaming (Ctrl-C to stop)...")
        last_print = 0.0
        for s in cato.samples():
            if s.host_ts - last_print < 0.1:
                continue
            last_print = s.host_ts
            roll, pitch, yaw = s.euler()
            print(
                f"acc [{s.acc[0]:7.2f} {s.acc[1]:7.2f} {s.acc[2]:7.2f}] m/s²  "
                f"gyro [{s.gyro[0]:8.2f} {s.gyro[1]:8.2f} {s.gyro[2]:8.2f}] °/s  "
                f"rpy [{roll:7.1f} {pitch:7.1f} {yaw:7.1f}]°",
                end="\r",
            )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nDone.")
