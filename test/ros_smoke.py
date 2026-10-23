#!/usr/bin/env python3
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0
"""PC/K3 software acceptance: mock motor control.

Requires the test-only no_hardware_io library. Never uses physical devices.
Run after sourcing the freshly built output/staging/setup.bash.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import threading
import subprocess
import tempfile
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(os.environ.get("SROBOTIS_ROOT", ".")))
    parser.add_argument("--domain", type=int, default=197)
    parser.add_argument("--prefix", type=Path, help="Go2 install prefix; default: output/staging")
    parser.add_argument("--hardware-guard", type=Path, help="Path to libno_hardware_io.so")
    parser.add_argument("--skip-robot-isolation", action="store_true",
                        help="Skip concurrent web instances on a board with one network manager")
    args = parser.parse_args()
    root = args.root.resolve()
    app = Path(__file__).resolve().parents[1]
    staging = (args.prefix or root / "output/staging").resolve()
    guard = (args.hardware_guard or root / "output/build/ros2/application/unitree_go2/libno_hardware_io.so").resolve()
    if not guard.is_file():
        raise RuntimeError("build unitree_go2 with BUILD_TESTING=ON first")
    output = root / "output/validation"
    output.mkdir(parents=True, exist_ok=True)
    run = Path(tempfile.mkdtemp(prefix="ros-smoke-", dir=output))
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY="1", ROS_LOG_DIR=str(run / "ros-log"))
    env = dict(os.environ, LD_PRELOAD=str(guard))
    # Verify the tripwire before starting application processes.
    check = subprocess.run(["python3", "-c", "import os;os.open('/dev/ttyMUST_NOT_OPEN',os.O_RDONLY)"],
                           env=env, capture_output=True)
    assert check.returncode == 86 and b"NO_HARDWARE_IO" in check.stderr

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from sensor_msgs.msg import Imu, JointState
    from unitree_go2.srv import Calibration, SetMode
    rclpy.init()
    node = rclpy.create_node("go2_software_acceptance")
    processes, logs, subscriptions = [], [], []
    report = {"synthetic_only": True, "run_directory": str(run)}

    def start(name, command):
        log = open(run / (name + ".log"), "w")
        logs.append(log)
        p = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(p)
        return p

    def stop(p):
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGTERM)
                p.wait(timeout=5)

    def spin(sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=min(.02, max(0, end-time.monotonic())))

    def receive(cls, topic, container, qos=qos_profile_sensor_data):
        subscriptions.append(node.create_subscription(cls, topic, container.append, qos))

    try:
        base = [str(staging / "lib/unitree_go2/go2_node"), "--ros-args", "--params-file",
                str(app / "config/go2.yaml"),
                "-p", "motion_log_path:=" + str(run / "motion.csv")]
        # Reject an unauthorized serial launch before attempting any device open.
        rejected = subprocess.run(base + ["-p", "backend:=serial",
            "-p", "motor_io_authorized:=false",
            "-p", "actuation_enabled:=false",
            "-p", "hardware_configuration_verified:=false",
            "-p", "motor_ids_verified:=false",
            "-p", "joint_calibration_verified:=false",
            "-p", "suspended_test_verified:=false"],
            env=env, capture_output=True, text=True, timeout=20)
        assert rejected.returncode != 0 and rejected.returncode != 86
        assert "all hardware and actuation approvals" in rejected.stderr
        control = start("control", base + ["-p", "backend:=mock",
            "-p", "motor_io_authorized:=false", "-p", "actuation_enabled:=false"])
        joints, body_imu, statuses = [], [], []
        receive(JointState, "/joint_states", joints)
        receive(Imu, "/go2/body_imu", body_imu)
        receive(String, "/go2/status", statuses, 10)
        client = node.create_client(SetMode, "/go2/set_mode")
        assert client.wait_for_service(timeout_sec=15)

        def mode(value):
            req = SetMode.Request()
            req.mode = value
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            assert future.done(), "mode service timed out"
            return future.result()

        command = node.create_publisher(Twist, "/cmd_vel_direct", 10)
        spin(.5)
        velocity = Twist()
        velocity.linear.x = .1
        command.publish(velocity)
        spin(.5)
        assert mode("passive").message == "already passive", "cmd_vel_direct activated control"
        calibration = node.create_client(Calibration, "/go2/calibration")
        assert calibration.wait_for_service(timeout_sec=5)
        # Exercise hardware lifecycle changes while IMU/status callbacks run.
        for operation in ("finish", "resume", "abort") * 3:
            for action in ("prepare", operation):
                request = Calibration.Request()
                request.operation = action
                if action == "finish":
                    request.joint_zero_offsets_rad = [0.] * 12
                    request.startup_reference_rotor_positions_rad = [0.] * 12
                future = calibration.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                assert future.done() and future.result().accepted, action
                spin(.03)
                assert control.poll() is None, "node exited during calibration"
        report["calibration_lifecycle"] = "PASS: prepare/finish/resume/abort with concurrent publishers"
        # Removed modes are rejected by every public mode service.
        for service_name in ("/go2/set_mode", "/go2/set_mode_rc", "/go2/set_mode_web"):
            request_client = node.create_client(SetMode, service_name)
            assert request_client.wait_for_service(timeout_sec=5)
            for removed_mode in ("run", "heart", "right_flip"):
                request = SetMode.Request()
                request.mode = removed_mode
                future = request_client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                assert future.done(), service_name
                result = future.result()
                assert not result.accepted, (service_name, result.message)
                spin(.1)
                assert json.loads(statuses[-1].data)["mode"] == "passive"
            node.destroy_client(request_client)
        spin(.2)
        queued_walk = mode("walk")
        assert queued_walk.accepted and "FixStand" in queued_walk.message
        spin(1.4)
        assert json.loads(statuses[-1].data)["mode"] == "walk"
        assert mode("passive").accepted
        spin(2.3)
        assert json.loads(statuses[-1].data)["mode"] == "passive"
        queued_terrain = mode("terrain")
        assert queued_terrain.accepted and "FixStand" in queued_terrain.message
        assert not mode("invalid").accepted
        assert not mode("free_avoid").accepted
        assert mode("stand").accepted
        spin(1.3)
        assert joints and body_imu and len(joints[-1].name) == 12
        status = json.loads(statuses[-1].data)
        assert status["mode"] == "stand" and not status["protective_crouch"]
        assert status["robot"] == "go2" and status["ros_domain_id"] == args.domain
        assert len(status["joints"]) == 12 and len(status["leg_errors"]) == 4
        assert status["imu_valid"] and all(joint["valid"] for joint in status["joints"])
        assert joints[-1].name[0] == "FR_hip_joint" and body_imu[-1].header.frame_id == "go2_body_imu"
        assert not mode("isaacgym_handstand").accepted
        assert not mode("jump").accepted
        assert mode("terrain").accepted
        spin(.3)
        # Exercise mutually exclusive RC/navigation forwarding, including data
        # remaining observable on RC while navigation owns the controller.
        forwarded, observed_rc = [], []
        receive(Twist, "/cmd_vel_direct", forwarded)
        receive(Twist, "/cmd_vel_rc", observed_rc)
        nav_command = node.create_publisher(Twist, "/cmd_vel", 10)
        ai_command = node.create_publisher(Twist, "/cmd_vel_ai", 10)
        rc_command = node.create_publisher(Twist, "/cmd_vel_rc", 10)
        spin(.3)
        rc_velocity = Twist()
        rc_velocity.linear.x = .4
        nav_velocity = Twist()
        nav_velocity.linear.x = .2
        ai_velocity = Twist()
        ai_velocity.linear.x = .317
        assert not mode("follow").accepted
        for selected, expected in (("terrain", .4), ("navigation", .2), ("ai_agent", .317), ("terrain", .4)):
            result = mode(selected)
            assert result.accepted
            spin(1.3 if "FixStand" in result.message else .15)
            state = json.loads(statuses[-1].data)
            assert state["mode"] == ("walk" if selected in ("navigation", "ai_agent") else "terrain"), state
            forwarded.clear()
            observed_rc.clear()
            for _ in range(5):
                rc_command.publish(rc_velocity)
                nav_command.publish(nav_velocity)
                ai_command.publish(ai_velocity)
                spin(.05)
            assert observed_rc and all(abs(v.linear.x - .4) < 1e-6 for v in observed_rc)
            assert forwarded and all(abs(v.linear.x - expected) < 1e-6 for v in forwarded), selected
        assert mode("navigation").accepted
        spin(.15)
        forwarded.clear()
        for _ in range(8):
            rc_command.publish(rc_velocity)
            spin(.05)
        assert not forwarded, "RC leaked into direct input with navigation silent"
        assert not mode("invalid").accepted
        rc_command.publish(rc_velocity)
        spin(.1)
        assert not forwarded, "rejected mode request changed RC routing"
        assert mode("terrain").accepted
        spin(.15)
        forwarded.clear()
        rc_velocity.linear.x = float("nan")
        rc_command.publish(rc_velocity)
        nav_command.publish(nav_velocity)
        spin(.15)
        assert not forwarded, "invalid RC or inactive navigation input was forwarded"

        # Feed real SBUS parser/node through a PTY. This verifies that selecting
        # navigation still publishes live sticks instead of zeros or silence.
        master, slave = os.openpty()
        channels = [1002] * 16
        channels[2] = 1362  # Forward stick, half scale.
        channels[7] = channels[8] = 282
        channels[4] = channels[5] = 282  # Slot 1: terrain ONNX.
        frame_lock = threading.Lock()
        feed_stop = threading.Event()

        def feed_sbus():
            while not feed_stop.is_set():
                with frame_lock:
                    packed = sum(value << (11 * i) for i, value in enumerate(channels))
                os.write(master, bytes([0x0f]) + packed.to_bytes(22, "little") + bytes([0, 0]))
                feed_stop.wait(.01)

        sbus = start("sbus", [str(staging / "lib/unitree_go2/sbus_teleop_node"),
            "--ros-args", "-p", "device:=" + os.ttyname(slave)])
        feeder = threading.Thread(target=feed_sbus, daemon=True)
        feeder.start()
        try:
            spin(.6)
            assert sbus.poll() is None
            with frame_lock:
                channels[8] = 1722  # C edge selects terrain.
            spin(.5)
            observed_rc.clear()
            forwarded.clear()
            # The PTY publisher joins an already running ROS graph. Wait for
            # DDS discovery and both routes instead of assuming a 200 ms match.
            deadline = time.monotonic() + 5.
            while time.monotonic() < deadline:
                spin(.05)
                if (observed_rc and observed_rc[-1].linear.x > .4 and
                        forwarded and forwarded[-1].linear.x > .4):
                    break
            assert observed_rc and observed_rc[-1].linear.x > .4, [v.linear.x for v in observed_rc]
            assert forwarded and forwarded[-1].linear.x > .4
            with frame_lock:
                channels[4] = channels[5] = 1002  # Slot 5: navigation.
                channels[8] = 282
            spin(.5)
            observed_rc.clear()
            forwarded.clear()
            for _ in range(5):
                nav_command.publish(nav_velocity)
                spin(.05)
            assert observed_rc and all(v.linear.x > .4 for v in observed_rc)
            assert forwarded and all(abs(v.linear.x - .2) < 1e-6 for v in forwarded)
            with frame_lock:
                channels[0] = channels[3] = 1722
            spin(.2)
            observed_rc.clear()
            spin(.2)
            assert observed_rc and all(abs(abs(v.linear.y) - .4) < 1e-6 for v in observed_rc)
            assert all(abs(abs(v.angular.z) - 1.) < 1e-6 for v in observed_rc)
            with frame_lock:
                channels[0] = channels[3] = 1002
                channels[2] = 642  # Live direction change during navigation.
            spin(.1)
            observed_rc.clear()
            forwarded.clear()
            spin(.2)
            assert observed_rc and all(v.linear.x < -.4 for v in observed_rc)
            assert not forwarded, "SBUS guidance controlled robot during navigation"
            with frame_lock:
                channels[4] = channels[5] = 282
                channels[8] = 1722  # Back to terrain.
            spin(.4)
            forwarded.clear()
            spin(.2)
            assert forwarded and all(v.linear.x < -.4 for v in forwarded)
            with frame_lock:
                channels[4] = channels[5] = 1002
                channels[8] = 282
            spin(.4)
            feed_stop.set()
            feeder.join(timeout=2)
            spin(.4)
            observed_rc.clear()
            forwarded.clear()
            nav_command.publish(nav_velocity)
            spin(.2)
            assert observed_rc and all(v.linear.x == 0 for v in observed_rc)
            assert forwarded and abs(forwarded[-1].linear.x - .2) < 1e-6, "RC loss exited navigation"
        finally:
            feed_stop.set()
            feeder.join(timeout=2)
            stop(sbus)
            os.close(master)
            os.close(slave)

        assert control.poll() is None
        assert mode("passive").accepted
        spin(.2)
        assert "NO_HARDWARE_IO" not in (run / "control.log").read_text()
        report["control_smoke"] = "PASS: mock control, RC/navigation routing, SBUS PTY guidance and dropout"
        stop(control)
        # Two installed launch instances with identical topic/service names.
        # All processes remain localhost-only and guarded against hardware I/O.
        other_context = rclpy.context.Context()
        rclpy.init(context=other_context, domain_id=args.domain + 1)
        other_node = rclpy.create_node("go2_other_robot_acceptance", context=other_context)
        other_statuses, other_velocities = [], []
        other_subscriptions = [
            other_node.create_subscription(String, "/go2/status", other_statuses.append, 10),
            other_node.create_subscription(Twist, "/cmd_vel_direct", other_velocities.append, 10),
        ]
        ports = []
        for _ in range(2):
            with socket.socket() as reservation:
                reservation.bind(("127.0.0.1", 0))
                ports.append(reservation.getsockname()[1])

        def spin_both(sec):
            end = time.monotonic() + sec
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.01)
                rclpy.spin_once(other_node, timeout_sec=.01)

        def web_request(port, payload=None):
            path = "/api/status" if payload is None else "/api/command"
            if payload is not None:
                payload = {"axes": [0, 0, 0], **payload}
            request = Request("http://127.0.0.1:" + str(port) + path,
                              data=None if payload is None else json.dumps(payload).encode(),
                              headers={"Content-Type": "application/json", "X-Go2-Application": "unitree-go2-sdk"})
            with urlopen(request, timeout=5) as response:
                return json.load(response)

        try:
            if args.skip_robot_isolation:
                report["robot_isolation"] = "SKIP: single-board network manager owns an exclusive lock"
            else:
                for robot, domain, port in (("go2", args.domain, ports[0]),
                                            ("robot-a", args.domain + 1, ports[1])):
                    start(robot + "-launch", ["ros2", "launch", "unitree_go2", "go2.launch.py",
                        "robot:=" + robot, "backend:=mock", "board_profile:=new_bigcore",
                        "local_config:=/nonexistent/go2-robot.yaml",
                        "ros_domain_id:=" + str(domain), "web:=true", "sbus:=false",
                        "web_port:=" + str(port), "motion_log_path:=" + str(run / (robot + ".csv"))])
                deadline = time.monotonic() + 20
                while time.monotonic() < deadline:
                    spin_both(.1)
                    try:
                        if all(web_request(port)["connected"] for port in ports) and other_statuses:
                            break
                    except OSError:
                        continue
                else:
                    raise AssertionError("isolated web/control instances did not become ready")
                statuses.clear()
                spin_both(.5)
                assert statuses and other_statuses
                assert all(json.loads(msg.data)["robot"] == "go2" for msg in statuses)
                assert all(json.loads(msg.data)["robot"] == "robot-a" for msg in other_statuses)
                assert web_request(ports[0], {"mode": "stand"})["accepted"]
                spin_both(1.5)
                assert json.loads(statuses[-1].data)["mode"] == "stand"
                assert json.loads(other_statuses[-1].data)["mode"] == "passive"
                assert web_request(ports[1], {"mode": "crouch"})["accepted"]
                spin_both(1.5)
                assert json.loads(other_statuses[-1].data)["mode"] == "crouch"
                assert json.loads(statuses[-1].data)["mode"] == "stand"
                forwarded.clear()
                other_velocities.clear()
                rc_velocity.linear.x = .417
                for _ in range(10):
                    rc_command.publish(rc_velocity)
                    spin_both(.05)
                assert any(abs(msg.linear.x - .417) < 1e-6 for msg in forwarded)
                assert all(abs(msg.linear.x) < 1e-6 for msg in other_velocities)
                spin_both(.5)
                forwarded.clear()
                assert web_request(ports[1], {"axes": [0.173, 0, 0]})["accepted"]
                spin_both(.2)
                assert any(abs(msg.linear.x - .173) < 1e-6 for msg in other_velocities)
                assert all(abs(msg.linear.x) < 1e-6 for msg in forwarded)
                report["robot_isolation"] = (
                    "PASS: web modes, RC/web velocities and JSON status stay in their robot domain"
                )
        finally:
            for subscription in other_subscriptions:
                other_node.destroy_subscription(subscription)
            other_node.destroy_node()
            other_context.shutdown()
        # Exercise the installed combined launch through real UART decoding on
        # a PTY, together with HTTP modes and the final velocity topic.
        for process in processes:
            stop(process)
        with socket.socket() as reservation:
            reservation.bind(("127.0.0.1", 0))
            shared_port = reservation.getsockname()[1]
        master, slave = os.openpty()
        channels = [1002] * 16
        channels[2] = 1362
        channels[7] = channels[8] = 282
        channels[4] = channels[5] = 282  # Slot 1: terrain ONNX.
        frame_lock = threading.Lock()
        feed_stop = threading.Event()
        feed_paused, frame_flags = True, 0

        def feed_shared_sbus():
            while not feed_stop.is_set():
                with frame_lock:
                    packed = sum(value << (11 * i) for i, value in enumerate(channels))
                    paused, flags = feed_paused, frame_flags
                if not paused:
                    os.write(master, bytes([0x0f]) + packed.to_bytes(22, "little") + bytes([flags, 0]))
                feed_stop.wait(.01)

        start("shared-launch", ["ros2", "launch", "unitree_go2", "go2.launch.py",
            "robot:=robot-a", "backend:=mock", "board_profile:=new_bigcore",
            "local_config:=/nonexistent/go2-robot.yaml",
            "ros_domain_id:=" + str(args.domain), "web:=true", "sbus:=true",
            "sbus_device:=" + os.ttyname(slave), "web_port:=" + str(shared_port),
            "motion_log_path:=" + str(run / "shared.csv")])
        feeder = threading.Thread(target=feed_shared_sbus, daemon=True)
        feeder.start()
        try:
            deadline = time.monotonic() + 25
            while time.monotonic() < deadline:
                spin(.1)
                try:
                    if web_request(shared_port)["connected"]:
                        break
                except OSError:
                    continue
            else:
                raise AssertionError("combined control/web/SBUS launch did not become ready")
            spin(.3)
            assert web_request(shared_port)["control_source"] == "web", "unpowered RC must leave web control"
            with frame_lock:
                feed_paused = False
            spin(.6)
            assert web_request(shared_port)["control_source"] == "sbus"
            assert not web_request(shared_port, {"mode": "stand"})["accepted"]
            with frame_lock:
                channels[8] = 1722
            spin(1.5)
            assert json.loads(statuses[-1].data)["mode"] == "terrain"
            forwarded.clear()
            assert not web_request(shared_port, {"axes": [.831, 0, 0]})["accepted"]
            spin(.2)
            assert forwarded and all(.4 < msg.linear.x < .6 for msg in forwarded), "web raced RC velocity"

            with frame_lock:
                feed_paused = True
            spin(.45)
            state = json.loads(statuses[-1].data)
            assert state["control_source"] == "sbus" and state["mode"] == "terrain", state
            assert abs(forwarded[-1].linear.x) < 1e-6, "stale sticks were not zeroed"
            with frame_lock:
                feed_paused = False
            spin(.25)
            with frame_lock:
                frame_flags = 4  # A burst of frame-lost flags is still a brief gap.
            spin(.35)
            assert web_request(shared_port)["control_source"] == "sbus"
            with frame_lock:
                frame_flags = 0
            spin(.25)
            with frame_lock:
                frame_flags = 8  # Continuous failsafe traffic when transmitter is off.
            spin(1.35)
            assert web_request(shared_port)["control_source"] == "web"
            assert web_request(shared_port, {"mode": "stand"})["accepted"]
            with frame_lock:
                frame_flags = 0
                channels[4], channels[5] = 282, 1002  # Slot 4, with no C edge.
            spin(1.2)
            state = json.loads(statuses[-1].data)
            assert state["control_source"] == "web" and state["sbus_slot"] == 4, state
            assert not state["ai_agent_enabled"], "web slot enabled AI"
            forwarded.clear()
            assert web_request(shared_port, {"axes": [.173, 0, 0]})["accepted"]
            spin(.2)
            assert any(abs(msg.linear.x - .173) < 1e-6 for msg in forwarded), "slot 4 did not forward web"
            rc_client = node.create_client(SetMode, "/go2/set_mode_rc")
            assert rc_client.wait_for_service(timeout_sec=2)
            request = SetMode.Request()
            request.mode = "crouch"
            future = rc_client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=3)
            assert future.done() and not future.result().accepted, "RC mode request bypassed web ownership"
            request.mode = "passive_direct"
            future = rc_client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=3)
            assert future.done() and future.result().accepted, "RC emergency stop was blocked by web ownership"
            spin(1.3)  # Finish the emergency crouch before requesting stand again.
            assert web_request(shared_port, {"mode": "stand"})["accepted"]
            # Slot 5 restores RC ownership; C selects external navigation.
            with frame_lock:
                channels[4] = 1002
                channels[8] = 282
            spin(.5)
            assert web_request(shared_port)["control_source"] == "sbus", "leaving slot 4 did not restore RC"
            with frame_lock:
                channels[8] = 1722
            spin(1.3)  # Navigation enters flat walking after FixStand.
            state = json.loads(statuses[-1].data)
            assert state["sbus_slot"] == 5 and state["navigation_enabled"] and not state["ai_agent_enabled"], state
            assert state["mode"] == "walk", state
            forwarded.clear()
            for _ in range(5):
                nav_command.publish(nav_velocity)
                ai_command.publish(ai_velocity)
                spin(.05)
            assert forwarded and all(abs(msg.linear.x - .2) < 1e-6 for msg in forwarded)
            assert web_request(shared_port)["navigation_enabled"], "web status missed RC navigation selection"
            # Navigation must retain velocity ownership across RC loss/reconnect.
            with frame_lock:
                feed_paused = True
            spin(1.35)
            assert json.loads(statuses[-1].data)["navigation_enabled"], "RC dropout cancelled navigation"
            with frame_lock:
                feed_paused = False
            spin(.5)
            forwarded.clear()
            spin(.3)
            state = json.loads(statuses[-1].data)
            assert state["navigation_enabled"] and not forwarded, "RC reconnect restored stick control"
            # C must reselect navigation after an explicit slot 4 handoff.
            with frame_lock:
                channels[4] = 282
            spin(.5)
            with frame_lock:
                channels[4], channels[8] = 1002, 282
            spin(.3)
            with frame_lock:
                channels[8] = 1722
            spin(.5)
            state = json.loads(statuses[-1].data)
            assert state["navigation_enabled"], "C did not reselect navigation after web handoff"
            with frame_lock:
                channels[4], channels[8] = 1722, 282  # Slot 6, release C first.
            spin(.3)
            with frame_lock:
                channels[8] = 1722
            spin(1.3)
            state = json.loads(statuses[-1].data)
            assert state["control_source"] == "sbus" and state["sbus_slot"] == 6, state
            assert state["ai_agent_enabled"] and not state["navigation_enabled"], state
            assert state["mode"] == "walk", state
            assert not web_request(shared_port, {"axes": [.831, 0, 0]})["accepted"]
            forwarded.clear()
            for _ in range(5):
                nav_command.publish(nav_velocity)
                ai_command.publish(ai_velocity)
                spin(.05)
            assert forwarded and all(abs(msg.linear.x - .317) < 1e-6 for msg in forwarded), "AI input routing failed"
            forwarded.clear()
            spin(.3)
            assert not forwarded, "RC sticks leaked into AI mode without AI commands"
            # Return to the web slot and verify the old AI stream is gated.
            with frame_lock:
                channels[4] = 282
            spin(.5)
            state = json.loads(statuses[-1].data)
            assert state["control_source"] == "web" and not state["ai_agent_enabled"], state
            forwarded.clear()
            for _ in range(5):
                ai_command.publish(ai_velocity)
                spin(.05)
            assert all(abs(msg.linear.x - .317) > 1e-6 for msg in forwarded), "inactive AI input was forwarded"
            try:
                web_request(shared_port, {"mode": "follow"})
            except HTTPError as error:
                assert error.code == 400 and json.load(error)["error"] == "unknown mode"
            else:
                raise AssertionError("removed follow mode was accepted by the web")
            assert web_request(shared_port, {"mode": "navigation"})["accepted"]
            spin(.4)
            assert json.loads(statuses[-1].data)["navigation_enabled"], "slot 4 heartbeat cancelled web navigation"
            assert web_request(shared_port, {"mode": "stand"})["accepted"]
            with frame_lock:
                channels[4] = channels[5] = 282
            spin(.5)
            with frame_lock:
                feed_paused = True
            spin(1.35)
            assert web_request(shared_port)["control_source"] == "web", "sustained UART silence did not restore web"
            report["shared_control"] = ("PASS: slots 4 web / 5 navigation / 6 AI, exclusive velocity routing, "
                                        "brief gaps, navigation dropout/reconnect, C reselection and web navigation")
        finally:
            feed_stop.set()
            feeder.join(timeout=2)
            os.close(master)
            os.close(slave)
        report["result"] = "PASS (control software only)"
        (run / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
        print(json.dumps(report, ensure_ascii=False, indent=2))
    finally:
        for process in reversed(processes):
            stop(process)
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()
        print("Acceptance logs:", run)


if __name__ == "__main__":
    main()
