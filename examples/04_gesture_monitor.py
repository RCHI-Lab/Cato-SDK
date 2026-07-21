"""Monitor device status changes and gesture inference reports.

Gesture reports (input report 8) only arrive when the device is in practice
mode / running gesture inference; status reports arrive periodically.

Usage: python examples/04_gesture_monitor.py
"""

import time

from cato import Cato, PracticeEvent, StatusEvent

_last_status = [None]


def show_status(st: StatusEvent) -> None:
    if st == _last_status[0]:
        return
    _last_status[0] = st
    flags = [
        name
        for name in ("hb_lost", "deep_sleep", "unsaved_changes", "pointer_sleep",
                     "scrolling", "practice", "hold_til_idle", "dwell_click")
        if getattr(st, name)
    ]
    print(f"[status] gesture={st.gesture} action={st.action} "
          f"profile={st.con_idx} flags={','.join(flags) or '-'}")


def show_gesture(ev: PracticeEvent) -> None:
    for g in ev.gestures:
        bar = "#" * (g.confidence // 4)
        marker = " <-- threshold" if g.threshold_crossed else ""
        print(f"[gesture] #{g.index:2d} conf {g.confidence:3d} |{bar:<32}|{marker}")


def main() -> None:
    with Cato() as cato:
        cato.on_status(show_status)
        cato.on_gesture(show_gesture)
        print("Monitoring status and gestures (Ctrl-C to stop)...")
        while cato.connected:
            time.sleep(0.2)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Done.")
