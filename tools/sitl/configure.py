#!/usr/bin/env python3
"""Configure a fresh Rotorflight SITL for the blackbox tests, over MSP.

Needs a running SITL and tools/sitl/sim_feed.py (the accelerometer
calibration needs still, level IMU data). Applies, then saves, then reboots
(the SITL process exits on a reboot; start it again):

  - Accelerometer calibration (MSP_ACC_CALIBRATION) if the arming disable
    flag ACC_CALIBRATION is set. A fresh eeprom.bin has no calibration.
  - Mode range 0: ARM on AUX1 (aux channel 0 = RC channel index 5),
    1700..2100 us (steps 40..120).
  - Serial: UART1 (TCP 5761) MSP, UART2 (TCP 5762) BLACKBOX at 2 Mbaud. A
    blackbox baud rate of 1 Mbaud or more removes the header write limit
    (blackbox_io.c); TCP has no baud rate.
  - Blackbox: device SERIAL, mode ARMED, --denom (default 2), grace period
    --grace (default 5 s), fields unchanged (the defaults of pg/blackbox.c).
  - Each --cli LINE (for example "set blackbox_params = OFF" or
    "set pid_process_denom = 4"), in the CLI after the MSP steps. Then CLI
    "save" (saves and reboots) instead of MSP_REBOOT. The CLI output is in
    the JSON ("cli").

The defaults of a fresh eeprom.bin give the rest: FEATURE_RX_MSP (the SITL
DEFAULT_RX_FEATURE), rcmap AETRC123, PID profile 1, no governor, no motor
protocol check (no motors in the default mixer).

Prints the configuration as JSON.
"""

import argparse
import json
import sys
import time

from msp import MSP, MSPError

FUNCTION_MSP = 1 << 0          # io/serial.h
FUNCTION_BLACKBOX = 1 << 7
BAUD_115200 = 5                # io/serial.h baudRate_e
BAUD_2000000 = 14
BLACKBOX_DEVICE_SERIAL = 3     # pg/blackbox.h
BLACKBOX_MODE_ARMED = 2
BOX_ARM = 0                    # msp_box.c permanent id
ACC_CALIBRATION_FLAG = 1 << 23  # fc/runtime_config.h


def pwm_to_step(us):
    return (us - 1500) // 5    # fc/rc_modes.h: CHANNEL_VALUE_TO_STEP (int8, sent as u8)


def cli_read(sock, until, timeout=5.0):
    """Read CLI output until it ends with 'until' (the prompt) or the time is up."""
    buf = b''
    deadline = time.monotonic() + timeout
    while not buf.endswith(until) and time.monotonic() < deadline:
        sock.settimeout(max(0.01, deadline - time.monotonic()))
        try:
            chunk = sock.recv(4096)
        except OSError:
            break
        if not chunk:
            break
        buf += chunk
    return buf.decode(errors='replace')


def run_cli(sock, lines, save=True):
    """Enter the CLI ('#'), run each line, then 'save' (the SITL saves and exits)."""
    out = []
    sock.sendall(b'#')
    out.append(cli_read(sock, b'\r\n# '))
    for line in lines:
        sock.sendall(line.encode() + b'\r')
        out.append(cli_read(sock, b'\r\n# '))
    if save:
        sock.sendall(b'save\r')
        out.append(cli_read(sock, b'Rebooting', timeout=5.0))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=5761, help='MSP port (UART1)')
    ap.add_argument('--blackbox-uart', type=int, default=2, help='UART number for BLACKBOX (TCP 5760+N)')
    ap.add_argument('--arm-aux', type=int, default=0, help='aux channel index of the ARM switch (0 = AUX1)')
    ap.add_argument('--denom', type=int, default=2, help='blackbox rate = PID rate / denom')
    ap.add_argument('--grace', type=int, default=5, help='blackbox grace period, s')
    ap.add_argument('--cli', action='append', default=[], help='a CLI line to run after the MSP steps (repeat)')
    ap.add_argument('--no-reboot', action='store_true')
    a = ap.parse_args()

    out = {}
    with MSP(a.host, a.port, timeout=3.0, connect_timeout=20.0) as fc:
        out['fc'] = dict(fc.api_version(), **fc.fc_version())

        # The arming flags are only valid after the first updateArmingStatus()
        # (fc/core.c, from the RX task). Until then a fresh eeprom.bin shows no
        # ACC_CALIBRATION. updateArmingStatus() also clears BOOT_GRACE_TIME
        # (3 s after boot), so wait for that.
        deadline = time.monotonic() + 15
        while True:
            st = fc.status()
            if 'BOOT_GRACE_TIME' not in st['arming_disabled_by']:
                break
            if time.monotonic() > deadline:
                sys.exit('configure: BOOT_GRACE_TIME did not clear (is sim_feed running?)')
            time.sleep(0.2)
        if st['arming_disable_flags'] & ACC_CALIBRATION_FLAG:
            fc.acc_calibration()
            deadline = time.monotonic() + 10
            while fc.status()['arming_disable_flags'] & ACC_CALIBRATION_FLAG:
                if time.monotonic() > deadline:
                    sys.exit('configure: accelerometer calibration did not finish (is sim_feed running?)')
                time.sleep(0.2)
            out['acc_calibration'] = 'done'
        else:
            out['acc_calibration'] = 'not needed'

        fc.set_mode_range(0, BOX_ARM, a.arm_aux, pwm_to_step(1700) & 0xFF, pwm_to_step(2100) & 0xFF)

        ports = fc.serial_config()
        bb_ident = a.blackbox_uart - 1           # SERIAL_PORT_USART1 = 0
        for p in ports:
            if p['identifier'] == 0:
                p['functions'] = FUNCTION_MSP
            elif p['identifier'] == bb_ident:
                p['functions'] = FUNCTION_BLACKBOX
                p['blackbox_baud'] = BAUD_2000000
            elif p['functions'] & FUNCTION_BLACKBOX:
                p['functions'] &= ~FUNCTION_BLACKBOX
        fc.set_serial_config(ports)

        bb = fc.blackbox_config()
        fc.set_blackbox_config(BLACKBOX_DEVICE_SERIAL, BLACKBOX_MODE_ARMED, a.denom, bb['fields'],
                               bb['initial_erase_kib'], bb['rolling_erase'], a.grace)

        fc.eeprom_write()

        out['mode_ranges'] = [r for r in fc.mode_ranges() if r[2] != r[3]]
        out['serial'] = [p for p in fc.serial_config() if p['functions']]
        out['blackbox'] = fc.blackbox_config()
        out['status'] = fc.status()

        if a.cli:
            out['cli'] = run_cli(fc.sock, a.cli + ['get blackbox_params', 'get pid_process_denom'], save=not a.no_reboot)
            out['rebooted'] = not a.no_reboot
        elif not a.no_reboot:
            fc.reboot()
            out['rebooted'] = True

    json.dump(out, sys.stdout, indent=1)
    print()


if __name__ == '__main__':
    try:
        main()
    except MSPError as e:
        sys.exit('configure: MSP error: %s' % e)
