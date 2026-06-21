#!/usr/bin/env python3
'''
Bare PCA9685 PWM test over Linux i2c-dev.

This is intended for bench testing only. Disconnect motor power or remove
thrusters before driving ESC signal lines.
'''

import argparse
import fcntl
import os
import struct
import time


I2C_SLAVE = 0x0703

MODE1 = 0x00
MODE2 = 0x01
LED0_ON_L = 0x06
ALL_LED_ON_L = 0xFA
ALL_LED_OFF_H = 0xFD
PRE_SCALE = 0xFE

MODE1_RESTART = 1 << 7
MODE1_AI = 1 << 5
MODE1_SLEEP = 1 << 4
MODE2_OUTDRV = 1 << 2
FULL_OFF = 1 << 4

DEFAULT_OSC_CLOCK = 25_000_000.0


class PCA9685:
    def __init__(self, bus, address, osc_clock):
        self.path = f"/dev/i2c-{bus}"
        self.address = address
        self.osc_clock = osc_clock
        self.fd = os.open(self.path, os.O_RDWR | os.O_CLOEXEC)
        fcntl.ioctl(self.fd, I2C_SLAVE, self.address)
        self.frequency = None

    def close(self):
        os.close(self.fd)

    def write_reg(self, reg, value):
        os.write(self.fd, bytes((reg & 0xFF, value & 0xFF)))

    def write_regs(self, reg, values):
        os.write(self.fd, bytes((reg & 0xFF,)) + bytes(v & 0xFF for v in values))

    def set_freq(self, freq_hz):
        prescale = round(self.osc_clock / (4096.0 * freq_hz)) - 1
        prescale = max(3, min(255, int(prescale)))

        self.write_reg(MODE1, MODE1_SLEEP)
        self.write_reg(PRE_SCALE, prescale)
        self.write_reg(MODE2, MODE2_OUTDRV)
        self.write_reg(MODE1, MODE1_AI)
        time.sleep(0.01)
        self.write_reg(MODE1, MODE1_RESTART | MODE1_AI)

        self.frequency = self.osc_clock / (4096.0 * (prescale + 1))

    def reset_all(self):
        self.write_regs(ALL_LED_ON_L, (0, 0, 0, FULL_OFF))

    def set_pwm_us(self, channel, pulse_us):
        if not 0 <= channel <= 15:
            raise ValueError("channel must be 0..15")

        if pulse_us <= 0:
            values = (0, 0, 0, FULL_OFF)
        else:
            period_us = 1_000_000.0 / self.frequency
            off_count = round((pulse_us / period_us) * 4096.0)
            off_count = max(1, min(4095, int(off_count)))
            values = (0, 0, off_count & 0xFF, (off_count >> 8) & 0x0F)

        self.write_regs(LED0_ON_L + 4 * channel, values)


def main():
    parser = argparse.ArgumentParser(description="Output a test PWM signal from a PCA9685")
    parser.add_argument("--bus", type=int, default=0, help="Linux I2C bus number, default: 0")
    parser.add_argument("--addr", type=lambda x: int(x, 0), default=0x40, help="PCA9685 I2C address, default: 0x40")
    parser.add_argument("--channel", type=int, default=0, help="PCA9685 channel 0..15, default: 0")
    parser.add_argument("--freq", type=float, default=50.0, help="PWM frequency in Hz, default: 50")
    parser.add_argument("--pulse", type=float, default=1500.0, help="Pulse width in us, default: 1500")
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to run before turning outputs off; 0 keeps running")
    parser.add_argument("--osc-clock", type=float, default=DEFAULT_OSC_CLOCK, help="PCA9685 oscillator clock in Hz")
    parser.add_argument("--no-reset", action="store_true", help="Do not reset all channels before starting")
    parser.add_argument("--stop-all", action="store_true", help="Turn all PCA9685 channels off and exit")
    args = parser.parse_args()

    dev = PCA9685(args.bus, args.addr, args.osc_clock)
    try:
        if args.stop_all:
            dev.reset_all()
            print(f"Stopped all channels on /dev/i2c-{args.bus} addr 0x{args.addr:02x}")
            return

        if not args.no_reset:
            dev.reset_all()

        dev.set_freq(args.freq)
        dev.set_pwm_us(args.channel, args.pulse)

        print(
            f"/dev/i2c-{args.bus} addr 0x{args.addr:02x} "
            f"channel {args.channel}: {args.pulse:.1f} us @ {dev.frequency:.2f} Hz"
        )

        if args.duration > 0:
            time.sleep(args.duration)
            dev.reset_all()
            print("Stopped all channels")
    finally:
        dev.close()


if __name__ == "__main__":
    main()
