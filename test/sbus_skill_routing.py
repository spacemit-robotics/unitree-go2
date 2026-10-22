#!/usr/bin/env python3
"""Exercise model selection, G pose holds and emergency priority through a real SBUS PTY.

Run on K3 after sourcing output/staging/setup.bash. The service only records
requests; no control node or hardware device is started.
"""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--binary', type=Path)
    parser.add_argument('--guard', type=Path)
    args = parser.parse_args()
    os.environ.update(ROS_DOMAIN_ID='198', ROS_LOCALHOST_ONLY='1')
    import rclpy
    from unitree_go2.srv import SetMode
    from std_srvs.srv import Trigger
    rclpy.init()
    node = rclpy.create_node('sbus_skill_routing_test')
    received = []
    reject_walk = False

    def request(req, response):
        received.append(req.mode)
        response.accepted = not (reject_walk and req.mode == 'walk')
        response.message = 'recorded without actuation'
        return response

    light_requests = []

    def toggle_light(_req, response):
        light_requests.append(True)
        response.success = True
        response.message = 'on' if len(light_requests) % 2 else 'off'
        return response

    light_service = node.create_service(Trigger, '/go2/toggle_white_light', toggle_light)
    service = node.create_service(SetMode, '/go2/set_mode', request)
    master, slave = os.openpty()
    channels = [1002] * 16
    channels[4] = channels[5] = channels[6] = channels[7] = channels[8] = 282
    channels[6] = 1722  # A already lit before the first valid SBUS frame.
    lock, stopped = threading.Lock(), threading.Event()

    def feed():
        while not stopped.is_set():
            with lock:
                value = sum(v << (11*i) for i, v in enumerate(channels))
            os.write(master, bytes([15]) + value.to_bytes(22, 'little') + bytes([0, 0]))
            stopped.wait(.01)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)

    guard = args.guard or args.root / 'output/build/ros2/application/unitree_go2/libno_hardware_io.so'
    assert guard.is_file()
    with tempfile.TemporaryFile(mode='w+') as log:
        process = subprocess.Popen([str(args.binary or args.root / 'output/staging/lib/unitree_go2/sbus_teleop_node'),
            '--ros-args', '-p', 'device:=' + os.ttyname(slave), '-p', 'bright_modes_enabled:=true'],
            env=dict(os.environ, LD_PRELOAD=str(guard)), stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        feeder = threading.Thread(target=feed, daemon=True)
        feeder.start()
        try:
            spin(1)
            assert not received, received  # Initial A level must not trigger any operation.
            with lock:
                channels[6] = 282
            spin(.2)
            assert not received, received
            assert not light_requests, light_requests
            # B/F/E select the model; each C edge sends one request.
            for b, f, e, expected in (
                (282, 1722, 1722, 'walk'),
                (282, 1722, 1002, 'forward_jump'),
                (1722, 282, 1722, 'sit'),
                (1722, 1002, 1722, None),  # Slot 15 is empty.
            ):
                before = len(received)
                with lock:
                    channels[7], channels[5], channels[4] = b, f, e
                    channels[8] = 282 if channels[8] == 1722 else 1722
                spin(1.2)
                assert received[before:] == ([] if expected is None else [expected]), received
            # A rejected request must remain retryable on the next C edge.
            reject_walk = True
            with lock:
                channels[7], channels[5], channels[4], channels[11] = 282, 1722, 1722, 1002
            spin(.2)
            for _ in range(2):
                before = len(received)
                with lock:
                    channels[8] = 282 if channels[8] == 1722 else 1722
                spin(1.2)
                assert received[before:] == ['walk'], received
            reject_walk = False
            before = len(received)
            with lock:
                channels[8] = 282 if channels[8] == 1722 else 1722
            spin(1.2)
            assert received[before:] == ['walk'], received
            # G holds remain independent of the reserved A button.
            before = len(received)
            with lock:
                channels[10] = 1722
            spin(.3)
            assert len(received) == before, received
            with lock:
                channels[10] = 1002
            spin(.15)
            with lock:
                channels[10] = 1722
            spin(.3)
            assert len(received) == before, received
            spin(.35)
            assert received[before:] == ['stand'], received
            spin(1.2)
            assert received[before:] == ['stand'], received
            # D wins and holding G at the same endpoint cannot restart stand.
            with lock:
                channels[9] = 1722
            spin(.3)
            assert received[-1:] == ['passive'], received
            before = len(received)
            spin(1.2)
            assert len(received) == before, received
            # Reposition G to arm a new half-second crouch hold.
            with lock:
                channels[10] = 282
            spin(.3)
            assert len(received) == before, received
            spin(.3)
            assert received[before:] == ['crouch'], received
            spin(1.2)
            assert received[before:] == ['crouch'], received
            # H-high alone no longer toggles lights or changes motion mode.
            before = len(received)
            with lock:
                channels[11] = 1722
            spin(.7)
            assert len(received) == before and not light_requests
            # Each C edge with H-high toggles the light once, without selecting a model.
            for count in (1, 2):
                with lock:
                    channels[8] = 282 if channels[8] == 1722 else 1722
                spin(.3)
                assert len(light_requests) == count, light_requests
                assert len(received) == before, received
                spin(.6)
                assert len(light_requests) == count, light_requests
            # Simultaneous H-high + D, A and C select only emergency stop.
            with lock:
                channels[6] = 1722
                channels[9] = 282
                channels[8] = 282 if channels[8] == 1722 else 1722
            spin(.3)
            assert received[before:] == ['passive_direct'], received
            assert len(light_requests) == 2, light_requests
            spin(1.2)
            assert received[before:] == ['passive_direct'], received
            # A is reserved and must not brake or select a model.
            with lock:
                channels[6] = 282
            spin(.3)
            assert received[before:] == ['passive_direct'], received
            # D with H-middle requests normal sleep, then H-high + D overrides it.
            with lock:
                channels[11] = 1002
                channels[9] = 1722
            spin(.1)
            with lock:
                channels[11] = 1722
                channels[9] = 282
            spin(.3)
            assert received[before:] == ['passive_direct', 'passive', 'passive_direct'], received
            # Emergency stop must also work in slot 4 (web control).
            before = len(received)
            with lock:
                channels[7], channels[5], channels[4] = 282, 1002, 282
                channels[9] = 1722
            spin(.3)
            assert received[before:] == ['passive_direct'], received
            with lock:
                channels[7], channels[5], channels[4] = 282, 282, 282
                channels[11] = 1002
            spin(.2)
            # Rearm G for the existing rollover checks.
            with lock:
                channels[10] = 1002
            spin(.15)
            with lock:
                channels[10] = 282
            spin(.6)
            assert received[-1:] == ['crouch'], received
            # H-low hold must be continuous; releasing before 0.5s cancels.
            with lock:
                channels[11] = 282
            spin(.3)
            assert received[-1:] == ['crouch'], received
            with lock:
                channels[11] = 1002
            spin(.15)
            # H alone rolls over after half a second, independently of A.
            with lock:
                channels[11], channels[6] = 282, 282
            spin(.3)
            assert received[-1:] == ['crouch'], received
            spin(.3)
            assert received[-1:] == ['rollover'], received
            count=len(received)
            spin(1.2)
            assert len(received)==count
            # D while H remains low cannot retrigger recovery.
            with lock:
                channels[9] = 282
            spin(1.2)
            assert received[-1:] == ['passive'], received
            assert len(received)==count+1
            assert process.poll() is None
            print('PASS SBUS PTY: model routing, rejected request retry, G/H holds, '
                  'A reserved, H+C light toggle, D sleep and H+D emergency priority')
        finally:
            stopped.set()
            feeder.join(timeout=2)
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
            log.seek(0)
            assert 'NO_HARDWARE_IO' not in log.read()
            os.close(master)
            os.close(slave)
            node.destroy_service(light_service)
            node.destroy_service(service)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
