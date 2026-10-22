#!/usr/bin/env python3
"""Exercise the real motor driver over a PTY, without hardware."""
import os
import pty
import select
import struct
import subprocess
import sys


def crc(data):
    value = 0x2CBB
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x8408 if value & 1 else 0)
    return value


def exercise(tool, error=0, corrupt=False):
    master, slave = pty.openpty()
    child = subprocess.Popen([tool, os.ttyname(slave), "1", "20"],
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        count = 1 if error or corrupt else 20
        for _ in range(count):
            command = b""
            while len(command) < 17:
                assert select.select([master], [], [], 2)[0], "BRAKE request timed out"
                command += os.read(master, 17 - len(command))
            assert command[:3] == b"\xfe\xee\x01", command
            assert command[3:15] == bytes(12), "nonzero torque/position/gains"
            assert struct.unpack("<H", command[15:])[0] == crc(command[:15])
            frame = b"\xfd\xee\x01" + struct.pack("<hhiBH", 0, 0, 32768, 30, error)
            frame += struct.pack("<H", crc(frame) ^ int(corrupt))
            os.write(master, frame)
        stdout, stderr = child.communicate(timeout=3)
        if error or corrupt:
            assert child.returncode != 0, (stdout, stderr)
        else:
            assert child.returncode == 0, (stdout, stderr)
            assert stdout.count("reply id=1") == 20
            assert "valid replies: 20/20" in stdout
    finally:
        if child.poll() is None:
            child.kill()
            child.wait()
        os.close(master)
        os.close(slave)


if __name__ == "__main__":
    for error, corrupt in ((0, False), (1, False), (0, True)):
        exercise(sys.argv[1], error, corrupt)
    invalid = subprocess.run([sys.argv[1], "/dev/not-opened", "1junk", "20"], capture_output=True)
    assert invalid.returncode == 2
    print("PASS: BRAKE-only wire frames, valid feedback, motor error, CRC rejection, argument validation")
