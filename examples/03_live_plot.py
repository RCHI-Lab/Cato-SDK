"""Realtime matplotlib plot of accelerometer and gyroscope traces.

Requires the [examples] extra: pip install -e ".[examples]"

Usage: python examples/03_live_plot.py
"""

from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from cato import Cato

WINDOW = 250  # samples, like the web demo's chart


def main() -> None:
    buffers = [[deque(maxlen=WINDOW) for _ in range(3)] for _ in range(2)]

    cato = Cato()
    cato.connect()
    cato.on_sample(
        lambda s: [
            buffers[0][i].append(s.acc[i]) or buffers[1][i].append(s.gyro[i])
            for i in range(3)
        ]
    )

    fig, (ax_acc, ax_gyro) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig.canvas.manager.set_window_title("Cato IMU")
    colors = ["#1e88e5", "#43a047", "#e53935"]
    labels = ["x", "y", "z"]
    lines = []
    for ax, title, unit in ((ax_acc, "Accelerometer", "m/s²"),
                            (ax_gyro, "Gyroscope", "°/s")):
        ax.set_title(title)
        ax.set_ylabel(unit)
        ax.grid(alpha=0.3)
        lines.append([ax.plot([], [], color=c, label=l, lw=1)[0]
                      for c, l in zip(colors, labels)])
        ax.legend(loc="upper right")
    ax_gyro.set_xlabel("sample")

    def update(_frame):
        for panel, axis_lines, ax in zip(buffers, lines, (ax_acc, ax_gyro)):
            for buf, line in zip(panel, axis_lines):
                line.set_data(range(len(buf)), list(buf))
            ax.relim()
            ax.autoscale_view()
        return [l for panel in lines for l in panel]

    _anim = FuncAnimation(fig, update, interval=33, cache_frame_data=False)
    try:
        plt.show()
    finally:
        cato.disconnect()


if __name__ == "__main__":
    main()
