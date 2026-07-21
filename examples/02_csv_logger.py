"""Log the IMU stream to a CSV file until Ctrl-C.

Columns include both the reference-frame values (acc/gyro/quaternion) and the
untouched on-wire values (raw_*) for offline analysis.

Usage: python examples/02_csv_logger.py [output.csv]
"""

import csv
import sys

from cato import Cato

FIELDS = [
    "host_ts",
    "device_ts",
    "acc_x", "acc_y", "acc_z",
    "gyro_x", "gyro_y", "gyro_z",
    "quat_w", "quat_x", "quat_y", "quat_z",
    "lin_acc_x", "lin_acc_y", "lin_acc_z",
    "raw_acc_y", "raw_acc_z", "raw_acc_x",
    "raw_gyro_y", "raw_gyro_z", "raw_gyro_x",
]


def main() -> None:
    path = sys.argv[1] if len(sys.argv) > 1 else "cato_log.csv"
    count = 0
    with Cato() as cato, open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(FIELDS)
        print(f"Logging to {path} (Ctrl-C to stop)...")
        try:
            for s in cato.samples():
                writer.writerow(
                    [s.host_ts, s.device_ts, *s.acc, *s.gyro,
                     *s.quaternion, *s.linear_acc, *s.raw[1:]]
                )
                count += 1
                if count % 100 == 0:
                    print(f"\r{count} samples", end="")
        except KeyboardInterrupt:
            pass
    print(f"\nWrote {count} samples to {path}")


if __name__ == "__main__":
    main()
