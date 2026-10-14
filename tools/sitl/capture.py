#!/usr/bin/env python3
"""Capture the blackbox serial output of the Rotorflight SITL to a .bbl file.

Connects to the TCP UART that has the BLACKBOX function (default UART2 =
127.0.0.1:5762) and writes every byte to the output file, as an OpenLog on
a real UART would. The SITL opens this port when the first log starts, so the
script retries the connection every 20 ms until it succeeds; the SITL keeps
the log header in its TX buffer until a client is connected.

Stops on SIGTERM or SIGINT, or after --idle seconds without data once data
has arrived (0 = never). Also writes <output>.chunks.csv: the CLOCK_MONOTONIC
time, file offset and size of each received chunk, to relate log bytes to
host time. Prints a JSON summary.
"""

import argparse
import json
import signal
import socket
import sys
import time


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('output')
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=5762)
    ap.add_argument('--idle', type=float, default=0.0)
    a = ap.parse_args()

    running = [True]

    def stop(*_):
        running[0] = False
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    stats = {'output': a.output, 'port': a.port, 'bytes': 0, 'connects': 0,
             't_start': time.monotonic(), 't_connect': None, 't_first_byte': None, 't_last_byte': None}
    sock = None
    with open(a.output, 'wb') as out, open(a.output + '.chunks.csv', 'w') as chunks:
        chunks.write('t_monotonic,offset,size\n')
        while running[0]:
            if sock is None:
                try:
                    sock = socket.create_connection((a.host, a.port), timeout=0.2)
                    sock.settimeout(0.1)
                    stats['connects'] += 1
                    if stats['t_connect'] is None:
                        stats['t_connect'] = time.monotonic()
                except OSError:
                    sock = None
                    time.sleep(0.02)
                    continue
            try:
                data = sock.recv(65536)
            except socket.timeout:
                if a.idle and stats['t_last_byte'] and time.monotonic() - stats['t_last_byte'] > a.idle:
                    break
                continue
            except OSError:
                data = b''
            if not data:                      # SITL closed the connection (exit or reboot)
                sock.close()
                sock = None
                continue
            now = time.monotonic()
            if stats['t_first_byte'] is None:
                stats['t_first_byte'] = now
            stats['t_last_byte'] = now
            chunks.write('%.6f,%d,%d\n' % (now, stats['bytes'], len(data)))
            out.write(data)
            out.flush()
            stats['bytes'] += len(data)
    if sock is not None:
        sock.close()
    stats['t_stop'] = time.monotonic()
    json.dump(stats, sys.stdout)
    print()


if __name__ == '__main__':
    main()
