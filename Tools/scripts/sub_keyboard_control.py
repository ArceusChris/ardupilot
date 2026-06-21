#!/usr/bin/env python3
'''
Keyboard MANUAL_CONTROL sender for ArduSub.

Run with thrusters disabled first. This sends MAVLink MANUAL_CONTROL, so
ArduSub mode, arming, mixer, and failsafes remain in the control path.
'''

import argparse
import select
import sys
import termios
import time
import tty

from pymavlink import mavutil


MODES = {
    "1": ("MANUAL", 19),
    "2": ("STABILIZE", 0),
    "3": ("ALT_HOLD", 2),
}


class RawTerminal:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)


def read_keys():
    keys = []
    while select.select([sys.stdin], [], [], 0)[0]:
        keys.append(sys.stdin.read(1))
    return keys


def set_mode(master, mode_id):
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )


def arm(master, armed):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1.0 if armed else 0.0,
        0, 0, 0, 0, 0, 0,
    )


def axis_from_keys(now, last, positive, negative, step, hold_time):
    value = 0
    if now - last.get(positive, 0) <= hold_time:
        value += step
    if now - last.get(negative, 0) <= hold_time:
        value -= step
    return value


def main():
    parser = argparse.ArgumentParser(description="Control ArduSub with keyboard via MAVLink MANUAL_CONTROL")
    parser.add_argument("--master", default="udpin:0.0.0.0:14550", help="MAVLink endpoint")
    parser.add_argument("--rate", type=float, default=20.0, help="MANUAL_CONTROL send rate in Hz")
    parser.add_argument("--step", type=int, default=350, help="Axis command magnitude 0..1000")
    parser.add_argument("--hold-time", type=float, default=0.25, help="Seconds to hold a key press")
    args = parser.parse_args()

    step = max(0, min(1000, args.step))
    period = 1.0 / args.rate

    print(f"Connecting to {args.master} ...")
    master = mavutil.mavlink_connection(args.master, source_system=255)
    master.wait_heartbeat()
    print(f"Heartbeat from system {master.target_system}, component {master.target_component}")
    print("")
    print("Keys:")
    print("  w/s: forward/back    a/d: left/right")
    print("  r/f: up/down         q/e: yaw left/right")
    print("  space: neutral       o/p: arm/disarm")
    print("  1/2/3: MANUAL/STABILIZE/ALT_HOLD")
    print("  +/-: step up/down    Ctrl-C: quit")
    print("")

    last_key = {}

    with RawTerminal():
        try:
            while True:
                now = time.monotonic()
                for key in read_keys():
                    if key == "\x03":
                        raise KeyboardInterrupt
                    if key == " ":
                        last_key.clear()
                    elif key == "o":
                        arm(master, True)
                        print("arm")
                    elif key == "p":
                        arm(master, False)
                        print("disarm")
                    elif key in MODES:
                        name, mode_id = MODES[key]
                        set_mode(master, mode_id)
                        print(f"mode {name}")
                    elif key in ("+", "="):
                        step = min(1000, step + 50)
                        print(f"step {step}")
                    elif key in ("-", "_"):
                        step = max(0, step - 50)
                        print(f"step {step}")
                    else:
                        last_key[key.lower()] = now

                x = axis_from_keys(now, last_key, "w", "s", step, args.hold_time)
                y = axis_from_keys(now, last_key, "d", "a", step, args.hold_time)
                z = 500 + axis_from_keys(now, last_key, "r", "f", step, args.hold_time)
                r = axis_from_keys(now, last_key, "e", "q", step, args.hold_time)
                z = max(0, min(1000, z))

                master.mav.manual_control_send(
                    master.target_system,
                    int(x),
                    int(y),
                    int(z),
                    int(r),
                    0,
                )
                time.sleep(period)
        finally:
            master.mav.manual_control_send(master.target_system, 0, 0, 500, 0, 0)
            print("\nneutral")


if __name__ == "__main__":
    main()
