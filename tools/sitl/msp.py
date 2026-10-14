#!/usr/bin/env python3
"""Minimal MSP v1/v2 client for the Rotorflight SITL (MSP on a TCP UART).

The decoded MSP payloads are shared by Rotorflight 4.6 and master a88bec8f8 (4.7):
src/main/msp/msp_protocol.h and src/main/msp/msp.c. Only the commands that the
SITL test harness needs are decoded; request() sends any command.

As a library:
    from msp import MSP
    with MSP(port=5761) as fc:
        print(fc.status())
        pid = fc.pid_tuning()
        pid['roll']['P'] = 80
        fc.set_pid_tuning(pid)

As a command:
    msp.py [--port 5761] status | pid | set-pid AXIS TERM VALUE | select-profile N |
           save | reboot | rc | api | raw CMD [HEX]
"""

import argparse
import json
import socket
import struct
import sys
import time

# src/main/msp/msp_protocol.h
MSP_API_VERSION = 1
MSP_FC_VARIANT = 2
MSP_FC_VERSION = 3
MSP_BUILD_INFO = 5
MSP_MODE_RANGES = 34
MSP_SET_MODE_RANGE = 35
MSP_FEATURE_CONFIG = 36
MSP_SET_FEATURE_CONFIG = 37
MSP_RX_CONFIG = 44
MSP_SERIAL_CONFIG = 54
MSP_SET_SERIAL_CONFIG = 55
MSP_ARMING_CONFIG = 61
MSP_SET_ARMING_CONFIG = 62
MSP_RX_MAP = 64
MSP_REBOOT = 68
MSP_BLACKBOX_CONFIG = 80
MSP_SET_BLACKBOX_CONFIG = 81
MSP_ADVANCED_CONFIG = 90
MSP_STATUS = 101
MSP_RC = 105
MSP_ATTITUDE = 108
MSP_PID_TUNING = 112
MSP_BOXIDS = 119
MSP_MOTOR_CONFIG = 131
MSP_SET_RAW_RC = 200
MSP_SET_PID_TUNING = 202
MSP_ACC_CALIBRATION = 205
MSP_SELECT_SETTING = 210
MSP_SET_MOTOR_CONFIG = 222
MSP_EEPROM_WRITE = 250

RATEPROFILE_MASK = 1 << 7          # msp.c: MSP_SELECT_SETTING

# src/main/fc/runtime_config.h, armingDisableFlags_e (the enum, not
# armingDisableFlagNames[], which has old names for bits 5 and 6)
ARMING_DISABLE_FLAGS = [
    'NO_GYRO', 'FAILSAFE', 'RX_FAILSAFE', 'BAD_RX_RECOVERY', 'BOXFAILSAFE',
    'GOVERNOR', 'RPM_SIGNAL', 'THROTTLE', 'ANGLE', 'BOOT_GRACE_TIME',
    'NOPREARM', 'LOAD', 'CALIBRATING', 'CLI', 'CMS_MENU', 'BST', 'MSP',
    'PARALYZE', 'GPS', 'RESC', 'RPMFILTER', 'REBOOT_REQUIRED',
    'DSHOT_BITBANG', 'ACC_CALIBRATION', 'MOTOR_PROTOCOL', 'OVERRIDE',
    'ARM_SWITCH',
]

# src/main/flight/pid.h: PID_AXIS_COUNT = 3, CYCLIC_AXIS_COUNT = 2
PID_AXES = ['roll', 'pitch', 'yaw']


def decode_arming_flags(flags):
    return [name for bit, name in enumerate(ARMING_DISABLE_FLAGS) if flags & (1 << bit)]


def crc8_dvb_s2(crc, data):
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


class MSPError(Exception):
    pass


class MSP:
    def __init__(self, host='127.0.0.1', port=5761, timeout=2.0, connect_timeout=10.0):
        self.host, self.port, self.timeout = host, port, timeout
        deadline = time.monotonic() + connect_timeout
        while True:
            try:
                self.sock = socket.create_connection((host, port), timeout=timeout)
                break
            except OSError:
                if time.monotonic() > deadline:
                    raise
                time.sleep(0.2)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.buf = b''

    def close(self):
        self.sock.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # --- framing -----------------------------------------------------------

    @staticmethod
    def encode_v1(cmd, payload=b''):
        size = len(payload)
        if size >= 255 or cmd > 254:
            raise ValueError('use MSP v2 for this frame')
        chk = size ^ cmd
        for b in payload:
            chk ^= b
        return b'$M<' + bytes([size, cmd]) + payload + bytes([chk])

    @staticmethod
    def encode_v2(cmd, payload=b'', flag=0):
        body = struct.pack('<BHH', flag, cmd, len(payload)) + payload
        return b'$X<' + body + bytes([crc8_dvb_s2(0, body)])

    def _read(self, n, deadline):
        while len(self.buf) < n:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise MSPError('timeout')
            self.sock.settimeout(remaining)
            try:
                chunk = self.sock.recv(4096)
            except socket.timeout:
                raise MSPError('timeout')
            if not chunk:
                raise MSPError('connection closed')
            self.buf += chunk
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def _read_frame(self, deadline):
        """Return (version, cmd, ok, payload) of the next reply frame."""
        while True:
            if self._read(1, deadline) != b'$':
                continue
            proto = self._read(1, deadline)
            if proto not in (b'M', b'X'):
                continue
            direction = self._read(1, deadline)
            ok = direction == b'>'
            if direction not in (b'>', b'!'):
                continue
            if proto == b'M':
                size, cmd = self._read(2, deadline)
                if size == 255:                       # JUMBO frame
                    size = struct.unpack('<H', self._read(2, deadline))[0]
                payload = self._read(size, deadline)
                chk = self._read(1, deadline)[0]
                calc = (255 if len(payload) >= 255 else len(payload)) ^ cmd
                if len(payload) >= 255:
                    calc ^= (len(payload) & 0xFF) ^ (len(payload) >> 8)
                for b in payload:
                    calc ^= b
                if calc != chk:
                    raise MSPError('v1 checksum error for cmd %d' % cmd)
                return 1, cmd, ok, payload
            hdr = self._read(5, deadline)
            _flag, cmd, size = struct.unpack('<BHH', hdr)
            payload = self._read(size, deadline)
            chk = self._read(1, deadline)[0]
            if crc8_dvb_s2(0, hdr + payload) != chk:
                raise MSPError('v2 checksum error for cmd %d' % cmd)
            return 2, cmd, ok, payload

    def request(self, cmd, payload=b'', version=None):
        """Send one command and return the payload of its reply."""
        if version is None:
            version = 2 if cmd > 254 or len(payload) >= 255 else 1
        frame = self.encode_v2(cmd, payload) if version == 2 else self.encode_v1(cmd, payload)
        self.sock.sendall(frame)
        deadline = time.monotonic() + self.timeout
        while True:
            _ver, rcmd, ok, data = self._read_frame(deadline)
            if rcmd != cmd:
                continue                              # a reply to an earlier command
            if not ok:
                raise MSPError('FC returned an error for cmd %d' % cmd)
            return data

    # --- commands ----------------------------------------------------------

    def api_version(self):
        d = self.request(MSP_API_VERSION)
        return {'protocol': d[0], 'api': '%d.%d' % (d[1], d[2])}

    def fc_version(self):
        variant = self.request(MSP_FC_VARIANT).decode(errors='replace')
        v = self.request(MSP_FC_VERSION)
        return {'variant': variant, 'version': '%d.%d.%d' % tuple(v[:3])}

    def status(self):
        d = self.request(MSP_STATUS)
        o = 0
        pid_us, gyro_us, sensors = struct.unpack_from('<HHH', d, o); o += 6
        mode_flags = struct.unpack_from('<I', d, o)[0]; o += 4
        o += 1                                        # compat: profile number
        max_load, avg_load = struct.unpack_from('<HH', d, o); o += 4
        extra = d[o]; o += 1 + extra
        nflags = d[o]; o += 1
        arming_flags = struct.unpack_from('<I', d, o)[0]; o += 4
        reboot_required, config_state = d[o], d[o + 1]; o += 2
        pid_profile, pid_profile_count, rate_profile, rate_profile_count = d[o:o + 4]; o += 4
        motor_count, servo_count, gyro_detect = d[o:o + 3]
        return {
            'pid_cycle_us': pid_us,
            'gyro_cycle_us': gyro_us,
            'sensors': sensors,
            'armed': bool(mode_flags & 1),            # bit 0 is BOXARM (msp_box.c)
            'mode_flags': mode_flags,
            'max_realtime_load': max_load,
            'avg_cpu_load': avg_load,
            'arming_disable_flags_count': nflags,
            'arming_disable_flags': arming_flags,
            'arming_disabled_by': decode_arming_flags(arming_flags),
            'reboot_required': reboot_required,
            'configuration_state': config_state,
            'pid_profile': pid_profile,
            'pid_profile_count': pid_profile_count,
            'rate_profile': rate_profile,
            'rate_profile_count': rate_profile_count,
            'motor_count': motor_count,
            'servo_count': servo_count,
            'gyro_detection_flags': gyro_detect,
        }

    def pid_tuning(self):
        """Gains of the current PID profile: P, I, D, F, B per axis, O for roll/pitch."""
        d = self.request(MSP_PID_TUNING)
        out = {}
        for i, axis in enumerate(PID_AXES):
            p, i_, dd, f = struct.unpack_from('<4H', d, i * 8)
            out[axis] = {'P': p, 'I': i_, 'D': dd, 'F': f}
        if len(d) >= 30:
            for i, axis in enumerate(PID_AXES):
                out[axis]['B'] = struct.unpack_from('<H', d, 24 + i * 2)[0]
        if len(d) >= 34:
            for i, axis in enumerate(PID_AXES[:2]):
                out[axis]['O'] = struct.unpack_from('<H', d, 30 + i * 2)[0]
        return out

    def set_pid_tuning(self, pid):
        """Write all gains of the current PID profile (msp.c applies them at once)."""
        payload = b''
        for axis in PID_AXES:
            a = pid[axis]
            payload += struct.pack('<4H', a['P'], a['I'], a['D'], a['F'])
        for axis in PID_AXES:
            payload += struct.pack('<H', pid[axis].get('B', 0))
        for axis in PID_AXES[:2]:
            payload += struct.pack('<H', pid[axis].get('O', 0))
        self.request(MSP_SET_PID_TUNING, payload)

    def select_pid_profile(self, index):
        self.request(MSP_SELECT_SETTING, bytes([index]))

    def select_rate_profile(self, index):
        self.request(MSP_SELECT_SETTING, bytes([index | RATEPROFILE_MASK]))

    def eeprom_write(self):
        """Save the configuration. The FC refuses this while armed."""
        self.request(MSP_EEPROM_WRITE)

    def reboot(self):
        """The SITL exits on a reboot; start it again."""
        try:
            self.request(MSP_REBOOT, bytes([0]))
        except MSPError:
            pass

    def rc(self):
        d = self.request(MSP_RC)
        return list(struct.unpack('<%dH' % (len(d) // 2), d))

    def set_raw_rc(self, channels):
        self.request(MSP_SET_RAW_RC, struct.pack('<%dH' % len(channels), *channels))

    def attitude(self):
        r, p, y = struct.unpack('<hhh', self.request(MSP_ATTITUDE)[:6])
        return {'roll_deg': r / 10, 'pitch_deg': p / 10, 'yaw_deg': y}

    def mode_ranges(self):
        d = self.request(MSP_MODE_RANGES)
        return [tuple(d[i:i + 4]) for i in range(0, len(d), 4)]

    def set_mode_range(self, index, permanent_id, aux_channel, start_step, end_step):
        self.request(MSP_SET_MODE_RANGE, bytes([index, permanent_id, aux_channel, start_step, end_step]))

    def feature_config(self):
        return struct.unpack('<I', self.request(MSP_FEATURE_CONFIG)[:4])[0]

    def set_feature_config(self, mask):
        self.request(MSP_SET_FEATURE_CONFIG, struct.pack('<I', mask))

    def serial_config(self):
        d = self.request(MSP_SERIAL_CONFIG)
        ports = []
        for o in range(0, len(d) - len(d) % 9, 9):
            ident, mask, msp_b, gps_b, tel_b, bb_b = struct.unpack_from('<BIBBBB', d, o)
            ports.append({'identifier': ident, 'functions': mask, 'msp_baud': msp_b,
                          'gps_baud': gps_b, 'telemetry_baud': tel_b, 'blackbox_baud': bb_b})
        return ports

    def set_serial_config(self, ports):
        payload = b''
        for p in ports:
            payload += struct.pack('<BIBBBB', p['identifier'], p['functions'], p['msp_baud'],
                                   p['gps_baud'], p['telemetry_baud'], p['blackbox_baud'])
        self.request(MSP_SET_SERIAL_CONFIG, payload)

    def blackbox_config(self):
        d = self.request(MSP_BLACKBOX_CONFIG)
        supported, device, mode = d[0], d[1], d[2]
        denom, fields = struct.unpack_from('<HI', d, 3)
        erase_kib, rolling, grace = struct.unpack_from('<HBB', d, 9)
        return {'supported': supported, 'device': device, 'mode': mode, 'denom': denom,
                'fields': fields, 'initial_erase_kib': erase_kib, 'rolling_erase': rolling,
                'grace_period': grace}

    def set_blackbox_config(self, device, mode, denom, fields, erase_kib=0, rolling=1, grace=5):
        self.request(MSP_SET_BLACKBOX_CONFIG,
                     struct.pack('<BBHIHBB', device, mode, denom, fields, erase_kib, rolling, grace))

    def motor_config(self):
        return self.request(MSP_MOTOR_CONFIG)

    def acc_calibration(self):
        self.request(MSP_ACC_CALIBRATION)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=5761)
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('api')
    sub.add_parser('status')
    sub.add_parser('pid')
    sp = sub.add_parser('set-pid')
    sp.add_argument('axis', choices=PID_AXES)
    sp.add_argument('term', choices=['P', 'I', 'D', 'F', 'B', 'O'])
    sp.add_argument('value', type=int)
    sp = sub.add_parser('select-profile')
    sp.add_argument('index', type=int, help='PID profile 0..5 (Configurator: 1..6)')
    sub.add_parser('save')
    sub.add_parser('reboot')
    sub.add_parser('rc')
    sub.add_parser('blackbox')
    sub.add_parser('serial')
    sp = sub.add_parser('raw')
    sp.add_argument('command', type=int)
    sp.add_argument('payload', nargs='?', default='')
    a = ap.parse_args()

    with MSP(a.host, a.port) as fc:
        if a.cmd == 'api':
            out = dict(fc.api_version(), **fc.fc_version())
        elif a.cmd == 'status':
            out = fc.status()
        elif a.cmd == 'pid':
            out = fc.pid_tuning()
        elif a.cmd == 'set-pid':
            pid = fc.pid_tuning()
            before = pid[a.axis][a.term]
            pid[a.axis][a.term] = a.value
            fc.set_pid_tuning(pid)
            out = {'axis': a.axis, 'term': a.term, 'before': before,
                   'after': fc.pid_tuning()[a.axis][a.term]}
        elif a.cmd == 'select-profile':
            fc.select_pid_profile(a.index)
            out = {'pid_profile': fc.status()['pid_profile']}
        elif a.cmd == 'save':
            fc.eeprom_write()
            out = {'saved': True}
        elif a.cmd == 'reboot':
            fc.reboot()
            out = {'reboot': True}
        elif a.cmd == 'rc':
            out = fc.rc()
        elif a.cmd == 'blackbox':
            out = fc.blackbox_config()
        elif a.cmd == 'serial':
            out = fc.serial_config()
        else:
            out = fc.request(a.command, bytes.fromhex(a.payload)).hex()
    json.dump(out, sys.stdout, indent=1)
    print()


if __name__ == '__main__':
    main()
