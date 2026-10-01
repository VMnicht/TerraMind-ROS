#!/usr/bin/env python3
"""Independent wire oracle. Creates a PTY emulator; never opens a hardware device."""
import argparse
import binascii
import os
from pathlib import Path
import select
import struct
import subprocess
import tempfile
import termios
import time


def control(seq, flags=0, linear=0., angular=0., lift=False, extra=b'', sprayer=None):
    payload = bytes([1, 1, flags, 0x10, 8]) + struct.pack('<ff', linear, angular)
    for ident in (0x20, 0x21, 0x30, 0x40, 0x50):
        on = (lift and ident == 0x40) or (sprayer is not None and ident == 0x50)
        value = sprayer if sprayer is not None and ident == 0x50 else 0.
        payload += bytes([ident, 5, int(on)]) + struct.pack('<f', value)
    payload += extra
    body = struct.pack('<BBHH', 1, 1, seq, len(payload)) + payload
    return b'\xfc\xfb' + body + struct.pack('<H', binascii.crc_hqx(body, 0)) + b'\xfd\xfe'


class Reader:
    def __init__(self, fd):
        self.fd, self.buffer = fd, b''

    def frames(self, timeout=.05):
        if select.select([self.fd], [], [], timeout)[0]:
            self.buffer += os.read(self.fd, 4096)
        out = []
        while len(self.buffer) >= 8:
            if self.buffer[:2] != b'\xfc\xfb':
                self.buffer = self.buffer[1:]
                continue
            n = struct.unpack_from('<H', self.buffer, 6)[0]
            if n > 256:
                self.buffer = self.buffer[1:]
                continue
            if len(self.buffer) < n + 12:
                break
            frame, self.buffer = self.buffer[:n+12], self.buffer[n+12:]
            if frame[2:4] != b'\x01\x81' or frame[-2:] != b'\xfd\xfe':
                continue
            if binascii.crc_hqx(frame[2:-4], 0) != struct.unpack_from('<H', frame, len(frame)-4)[0]:
                continue
            blocks, p = {}, 8
            while p < n + 8:
                ident, size = frame[p:p+2]
                assert ident not in blocks
                blocks[ident] = frame[p+2:p+2+size]
                p += 2 + size
            assert set(blocks) == {1, 0x10, 0x20, 0x21, 0x30, 0x40, 0x50}
            uptime, seq, result, mode, faults, caps, age, errors = struct.unpack('<IHBBHHHH', blocks[1])
            chassis = struct.unpack('<ffffff', blocks[0x10])
            spray_on, spray_target, spray_actual, spray_valid = struct.unpack('<BffB', blocks[0x50])
            out.append(dict(seq=seq, result=result, mode=mode, faults=faults, age=age, caps=caps,
                            linear=chassis[0], left_target=chassis[2], right_target=chassis[3],
                            spray_on=spray_on, spray_target=spray_target,
                            spray_actual=spray_actual, spray_valid=spray_valid))
        return out

    def wait(self, predicate, timeout=2.):
        end = time.monotonic()+timeout
        last = None
        while time.monotonic() < end:
            for state in self.frames():
                last = state
                if predicate(state):
                    return state
        raise AssertionError(f'state predicate timed out, last={last}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--emulator')
    args = parser.parse_args()
    if not args.emulator:
        from ament_index_python.packages import get_package_prefix
        args.emulator = str(Path(get_package_prefix('terramind_sim'))/'lib/terramind_sim/serial_board_emulator')
    assert binascii.crc_hqx(b'123456789', 0) == 0x31c3
    assert control(0)[-4:-2] == bytes.fromhex('bb1e')
    with tempfile.TemporaryDirectory(prefix='terramind-wire-') as directory:
        link = str(Path(directory)/'mcu')
        process = subprocess.Popen([args.emulator, '--link', link, '--fragment', '7'], stdout=subprocess.PIPE, text=True)
        fd = None
        try:
            end = time.monotonic()+5
            while not os.path.exists(link) and time.monotonic() < end:
                if process.poll() is not None:
                    raise RuntimeError('emulator failed to start')
                time.sleep(.01)
            fd = os.open(link, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            attrs = termios.tcgetattr(fd)
            attrs[0] = attrs[1] = attrs[3] = 0
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            attrs[4] = attrs[5] = termios.B115200
            attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            reader = Reader(fd)
            reader.wait(lambda s: s['mode'] == 0 and s['age'] == 65535)
            packet = control(65535, flags=1, linear=.2)
            # Noise and every possible read boundary are handled by the actual C++ parser.
            os.write(fd, b'noise')
            for byte in packet:
                os.write(fd, bytes([byte]))
            reader.wait(lambda s: s['seq'] == 65535 and s['mode'] == 1 and abs(s['linear']-.2) < 1e-5)
            os.write(fd, control(0, 1, .1, extra=b'\x99\x03abc'))
            reader.wait(lambda s: s['seq'] == 0 and s['result'] == 0)
            corrupt = bytearray(control(1, 1, .3)); corrupt[-4] ^= 1
            os.write(fd, corrupt)
            reader.wait(lambda s: s['seq'] == 0)
            os.write(fd, control(2, 1, 2.))
            reader.wait(lambda s: s['seq'] == 2 and s['result'] == 2)
            reader.wait(lambda s: s['faults'] & 1 and s['mode'] == 2 and s['left_target'] == 0)
            os.write(fd, control(3, 1, .1))
            reader.wait(lambda s: s['seq'] == 3 and s['mode'] == 1 and not s['faults'] & 1)
            os.write(fd, control(4, 1, .3, lift=True))
            reader.wait(lambda s: s['seq'] == 4 and s['result'] == 3 and abs(s['linear']-.1) < 1e-5)
            os.write(fd, control(5, extra=b'\x01\x01\x00'))
            reader.wait(lambda s: s['seq'] == 5 and s['result'] == 1)
            os.write(fd, control(6, flags=2))
            reader.wait(lambda s: s['seq'] == 6 and s['mode'] == 2 and s['left_target'] == 0)
            os.write(fd, control(7, flags=1, sprayer=37.5))
            reader.wait(lambda s: s['seq'] == 7 and s['result'] == 0 and s['caps'] == 47
                        and s['spray_on'] and s['spray_target'] == 37.5
                        and s['spray_valid'] and s['spray_actual'] > 0)
            os.write(fd, control(8, flags=1, sprayer=101.))
            reader.wait(lambda s: s['seq'] == 8 and s['result'] == 2 and s['spray_target'] == 37.5)
            reader.wait(lambda s: s['faults'] & 1 and not s['spray_on'] and s['spray_target'] == 0)
            os.write(fd, control(9, flags=1, sprayer=25.))
            reader.wait(lambda s: s['seq'] == 9 and s['spray_on'] and s['spray_target'] == 25.)
            os.write(fd, control(10, flags=2))
            reader.wait(lambda s: s['seq'] == 10 and not s['spray_on'] and s['spray_target'] == 0)
            print('PASS: independent CRC/TLV, fragmented serial, sequence wrap, invalid CRC/value, unsupported/duplicate TLV, sprayer feedback, timeout and stop')
        finally:
            if fd is not None:
                os.close(fd)
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()


if __name__ == '__main__':
    main()
