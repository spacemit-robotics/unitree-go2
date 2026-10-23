# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0
"""隔离验证网页图传生命周期，不启动 ROS 或图传程序。"""

import ast
from http.client import HTTPConnection, HTTPException
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
import os
from pathlib import Path
import runpy
import shutil
import signal
import subprocess
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
from urllib.parse import urlsplit


source = Path(__file__).resolve().parents[1] / "scripts/web_teleop_server.py"
tree = ast.parse(source.read_text())
definitions = [item for item in tree.body
               if isinstance(item, ast.ClassDef) and item.name in {"FpvProcess", "Camera"}]
namespace = {"Path": Path, "os": os, "shutil": shutil, "signal": signal,
             "subprocess": subprocess, "threading": threading, "tempfile": __import__("tempfile"),
             "json": json, "time": time, "Any": object}
exec(compile(ast.Module(body=definitions, type_ignores=[]), str(source), "exec"), namespace)
FpvProcess = namespace["FpvProcess"]
Camera = namespace["Camera"]
web_node = next(item for item in tree.body if isinstance(item, ast.ClassDef) and item.name == "Go2WebNode")
handler = next(item for item in web_node.body if isinstance(item, ast.FunctionDef) and item.name == "handler")
with patch.dict("sys.modules", {"dbus": Mock()}):
    network_module = runpy.run_path(str(source.with_name("go2_network.py")))
namespace.update({"BaseHTTPRequestHandler": BaseHTTPRequestHandler, "json": json,
                  "urlsplit": urlsplit, "CommandError": ValueError, "time": time,
                  "HTTPConnection": HTTPConnection, "HTTPException": HTTPException,
                  "APPLICATION_ID": "unitree-go2-sdk",
                  "valid_robot_id": network_module["valid_robot_id"]})
exec(compile(ast.Module(body=[handler], type_ignores=[]), str(source), "exec"), namespace)
namespace["MODES"] = ast.literal_eval(next(item.value for item in tree.body
    if isinstance(item, ast.Assign) and any(isinstance(target, ast.Name) and target.id == "MODES"
                                           for target in item.targets)))
for name in ("fpv_status", "update_fpv_rc", "set_mode"):
    method = next(item for item in web_node.body if isinstance(item, ast.FunctionDef) and item.name == name)
    namespace["Int32"] = SimpleNamespace
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), "exec"), namespace)


class CameraTests(unittest.TestCase):
    def test_missing_usb_camera_reports_cause_without_starting_processes(self):
        camera = Camera()
        with patch.object(camera, "_candidates", return_value=[]), patch.object(subprocess, "Popen") as start:
            self.assertFalse(camera.set_enabled(True))
            self.assertIn("未识别到 USB 摄像头", camera.status()["message"])
            start.assert_not_called()
        camera.close()
        self.assertFalse(camera.status()["enabled"])

    def test_missing_runtime_reports_dependencies(self):
        camera = Camera()
        with patch.object(camera, "_candidates", return_value=["/dev/video1"]), \
                patch.object(shutil, "which", return_value=None), patch.object(subprocess, "Popen") as start:
            self.assertFalse(camera.set_enabled(True))
            self.assertIn("MediaMTX", camera.status()["message"])
            start.assert_not_called()

    def test_publisher_failure_stops_server_and_keeps_diagnostic(self):
        camera = Camera()
        publisher = Mock(pid=999999, returncode=1)
        publisher.poll.return_value = 1
        server = Mock(pid=999998)
        server.poll.return_value = None
        camera._processes = [server, publisher]
        log = namespace["tempfile"].TemporaryFile()
        log.write(b"USB disconnected")
        log.flush()
        camera._logs = [log]
        with patch.object(os, "killpg") as kill:
            state = camera.status()
            self.assertFalse(state["enabled"])
            self.assertIn("USB disconnected", state["message"])
            kill.assert_called_once_with(server.pid, signal.SIGTERM)
            server.wait.assert_called_once_with(timeout=1)
            publisher.wait.assert_not_called()
        self.assertTrue(log.closed)
        self.assertEqual(camera._processes, [])


class WifiStartupTests(unittest.TestCase):
    def test_ap_name_updates_new_and_existing_profiles(self):
        network_type = network_module["RobotNetwork"]
        globals_ = network_type.__init__.__globals__
        for existing in (False, True):
            with self.subTest(existing=existing):
                state = {"ssid": "OldHotspot"}

                def command(*args):
                    if args[1:4] == ("-t", "-f", "DEVICE,TYPE"):
                        return "wlan0:wifi"
                    if args[1:3] == ("-g", "GENERAL.CON-UUID"):
                        return "wifi-uuid"
                    if args[1:3] == ("-g", "NAME"):
                        return "go2-sdk-ap-robot-a" if existing else ""
                    if args[1:3] == ("-g", "connection.uuid"):
                        return "ap-uuid"
                    if args[1:3] == ("connection", "modify") and "802-11-wireless.ssid" in args:
                        state["ssid"] = args[args.index("802-11-wireless.ssid") + 1]
                    if args[0] == "ip":
                        return json.dumps([{"addr_info": [
                            {"scope": "global", "local": "10.0.91.250", "prefixlen": 23},
                        ]}])
                    return ""

                command_mock = Mock(side_effect=command)
                settings = Mock()
                with patch.dict(globals_, {"command": command_mock, "open": Mock(return_value=Mock())}), \
                        patch.object(globals_["fcntl"], "flock"), \
                        patch.object(globals_["socket"], "socket"), \
                        patch.object(globals_["dbus"], "Interface", return_value=settings), \
                        patch.object(globals_["dbus"], "Dictionary", side_effect=lambda value, **_: value), \
                        patch.object(globals_["dbus"], "ByteArray", side_effect=bytes):
                    manager = network_type("robot-a", "ap", 8082, "10.0.91.250/23", "", "go2control",
                                           lambda: True, Mock(), ap_ssid="NewHotspot")
                    try:
                        self.assertEqual(state["ssid"], "NewHotspot")
                        self.assertEqual(manager.ssid, state["ssid"])
                        self.assertEqual(manager._restore_connection, "wifi-uuid")
                        self.assertFalse(any(call.args[1:3] == ("connection", "add")
                                             for call in command_mock.call_args_list))
                        self.assertEqual(settings.AddConnection.call_count, 0 if existing else 1)
                        if not existing:
                            profile = settings.AddConnection.call_args.args[0]
                            self.assertEqual(profile["802-11-wireless"], {"ssid": b"NewHotspot", "mode": "ap"})
                            self.assertEqual(profile["connection"]["interface-name"], "wlan0")
                            self.assertFalse(profile["connection"]["autoconnect"])
                            self.assertEqual(profile["ipv4"]["method"], "shared")
                    finally:
                        manager.close()

    def test_sdk_discovery_rejects_legacy_and_unrelated_peers(self):
        with patch.dict("sys.modules", {"dbus": Mock()}):
            module = runpy.run_path(str(source.with_name("go2_network.py")))
        network = module["RobotNetwork"].__new__(module["RobotNetwork"])
        network.robot = "robot-a"
        network.name = "Local robot"
        network.ready = lambda: True
        network.ssid = "Test Wi-Fi"
        network.mode = "wifi"
        network.hostname = "go2-sdk.local"
        network.subnet = ipaddress.ip_network("192.168.1.0/24")
        network._lock = threading.Lock()
        network._peers = {}
        packet = {"protocol": "go2-sdk-web-1", "robot": "robot-b", "port": 8082,
                  "network_mode": "wifi", "hostname": "go2-sdk.local", "connected": True}
        for changed in ({"protocol": "go2-web-1"}, {"hostname": "go2.local"},
                        {"robot": "../robot"}, {"robot": ""}, {"robot": ["bad"]}):
            network._receive(json.dumps({**packet, **changed}).encode(), "192.168.1.20")
            self.assertIsNone(network.endpoint("robot-b"))
        network._receive(json.dumps(packet).encode(), "192.168.1.20")
        self.assertEqual(network.endpoint("robot-b"), ("192.168.1.20", 8082))
        self.assertEqual([row["robot"] for row in network.devices()], ["robot-a", "robot-b"])
        self.assertEqual(network.devices()[0]["name"], "Local robot")
        packet.update(robot="custom_robot-003", name="Third robot")
        network._receive(json.dumps(packet).encode(), "192.168.1.21")
        devices = {row["robot"]: row for row in network.devices()}
        self.assertEqual(devices["custom_robot-003"]["name"], "Third robot")
        self.assertTrue(devices["custom_robot-003"]["connected"])

    def test_unavailable_preferred_wifi_preserves_current_or_uses_fallback(self):
        with patch.dict("sys.modules", {"dbus": Mock()}):
            network_type = runpy.run_path(str(source.with_name("go2_network.py")))["RobotNetwork"]
        globals_ = network_type.__init__.__globals__
        for failure, active, expected_fallback in (
            ("modify", "current-uuid", False),
            ("modify", "--", True),
            ("up", "current-uuid", False),
            ("timeout", "current-uuid", False),
            (None, "current-uuid", False),
            ("empty", "current-uuid", False),
            ("empty", "--", True),
        ):
            with self.subTest(failure=failure, active=active):
                state = {"active": active}

                def command(*args):
                    if args[1:4] == ("-t", "-f", "DEVICE,TYPE"):
                        return "wlan0:wifi"
                    if args[1:3] == ("-g", "GENERAL.CON-UUID"):
                        return state["active"]
                    if args[1:3] == ("-g", "GENERAL.CONNECTION"):
                        return "Current Wi-Fi"
                    if args[1:3] == ("connection", "modify"):
                        if failure == "modify":
                            raise RuntimeError("unknown connection JDSK")
                        if failure == "timeout":
                            raise subprocess.TimeoutExpired(args, 30)
                    if args[1:5] == ("--wait", "20", "connection", "up"):
                        if failure == "up":
                            raise RuntimeError("preferred network unavailable")
                        state["active"] = "preferred-uuid"
                    if args[1:3] == ("-g", "802-11-wireless.ssid"):
                        return "Connected Wi-Fi"
                    if args[0] == "ip":
                        return json.dumps([{"addr_info": [
                            {"scope": "global", "local": "192.168.1.10", "prefixlen": 24},
                        ]}])
                    return ""

                def fallback(_network, preferred):
                    self.assertEqual(preferred, "" if failure == "empty" else "JDSK")
                    state["active"] = "fallback-uuid"

                command_mock = Mock(side_effect=command)
                with patch.dict(globals_, {"command": command_mock, "open": Mock(return_value=Mock())}), \
                        patch.object(globals_["fcntl"], "flock"), \
                        patch.object(globals_["socket"], "socket"), \
                        patch.object(network_type, "_connect_fallback", autospec=True,
                                     side_effect=fallback) as connect_fallback:
                    network = network_type("robot-a", "wifi", 8080, "10.0.91.250/23",
                                           "" if failure == "empty" else "JDSK",
                                           "go2control", lambda: True, Mock())
                    try:
                        self.assertEqual(network.address, "192.168.1.10")
                        self.assertEqual(connect_fallback.called, expected_fallback)
                        if failure == "empty":
                            self.assertFalse(any("" in call.args for call in command_mock.call_args_list))
                        if failure in ("modify", "timeout"):
                            self.assertNotIn(unittest.mock.call(
                                "nmcli", "--wait", "20", "connection", "up", "JDSK", "ifname", "wlan0"
                            ), command_mock.call_args_list)
                    finally:
                        network.close()


class FpvProcessTests(unittest.TestCase):
    def setUp(self):
        self.manager = FpvProcess("fpv_sky", str(source), 8081, control_enabled=True)

    def test_native_is_disabled_by_default_without_probing_or_starting(self):
        with patch.object(shutil, "which") as probe, patch.object(subprocess, "Popen") as start, \
                patch.object(subprocess, "run") as command:
            manager = FpvProcess("fpv_sky", str(source), 8081)
            state = manager.status()
            self.assertEqual(state["mode"], "native")
            self.assertFalse(state["control_enabled"])
            self.assertFalse(state["available"])
            self.assertFalse(state["enabled"])
            self.assertFalse(manager.set_enabled(True)["success"])
            self.assertTrue(manager.set_enabled(False)["success"])
            manager.close()
            probe.assert_not_called()
            start.assert_not_called()
            command.assert_not_called()

    def test_missing_binary_does_not_start(self):
        with patch.object(shutil, "which", return_value=None), patch.object(subprocess, "Popen") as start:
            self.assertFalse(self.manager.set_enabled(True)["success"])
            start.assert_not_called()

    def test_start_is_idempotent_and_stop_reaps_owned_process(self):
        process = Mock(pid=123)
        process.poll.return_value = None
        process.wait.side_effect = [subprocess.TimeoutExpired("fpv_sky", 0.25), 0]
        with patch.object(shutil, "which", return_value="/bin/fpv_sky"), \
                patch.object(subprocess, "Popen", return_value=process) as start, \
                patch.object(os, "killpg") as kill:
            self.assertTrue(self.manager.set_enabled(True)["success"])
            self.assertTrue(self.manager.set_enabled(True)["enabled"])
            start.assert_called_once_with(
                ["fpv_sky", "--config", str(source), "--control-port", "8081"],
                stdin=subprocess.DEVNULL, start_new_session=True,
            )
            self.assertTrue(self.manager.set_enabled(False)["success"])
            kill.assert_called_once_with(123, signal.SIGTERM)

    def test_immediate_exit_is_reported_as_failure(self):
        process = Mock(returncode=2)
        process.poll.return_value = 2
        with patch.object(shutil, "which", return_value="/bin/fpv_sky"), \
                patch.object(subprocess, "Popen", return_value=process):
            result = self.manager.set_enabled(True)
            self.assertFalse(result["success"])
            self.assertIn("返回码 2", result["message"])

    def test_stop_timeout_keeps_process_for_retry(self):
        process = Mock(pid=123)
        process.poll.return_value = None
        process.wait.side_effect = subprocess.TimeoutExpired("fpv_sky", 2)
        self.manager._process = process
        with patch.object(os, "killpg"), patch.object(shutil, "which", return_value="/bin/fpv_sky"):
            result = self.manager.set_enabled(False)
            self.assertFalse(result["success"])
            self.assertTrue(result["enabled"])
            self.assertIs(self.manager._process, process)

    def test_missing_process_with_wait_timeout_is_handled(self):
        process = Mock(pid=123)
        process.poll.return_value = None
        process.wait.side_effect = subprocess.TimeoutExpired("fpv_sky", 2)
        self.manager._process = process
        with patch.object(os, "killpg", side_effect=ProcessLookupError):
            self.assertFalse(self.manager.set_enabled(False)["success"])
            self.assertIs(self.manager._process, process)

    def test_stop_os_error_is_reported(self):
        process = Mock(pid=123)
        process.poll.return_value = None
        self.manager._process = process
        with patch.object(os, "killpg", side_effect=PermissionError("denied")):
            self.assertFalse(self.manager.set_enabled(False)["success"])
            self.assertIs(self.manager._process, process)

    def test_close_escalates_after_stop_timeout(self):
        process = Mock(pid=123)
        process.poll.return_value = None
        process.wait.side_effect = [subprocess.TimeoutExpired("fpv_sky", 2), 0]
        self.manager._process = process
        with patch.object(os, "killpg") as kill:
            self.manager.close()
            self.assertEqual(kill.call_args_list,
                             [unittest.mock.call(123, signal.SIGTERM),
                              unittest.mock.call(123, signal.SIGKILL)])
            self.assertIsNone(self.manager._process)

    def test_close_prevents_late_restart(self):
        with patch.object(shutil, "which", return_value="/bin/fpv_sky"), \
                patch.object(subprocess, "Popen") as start:
            self.manager.close()
            self.assertFalse(self.manager.set_enabled(True)["success"])
            start.assert_not_called()


class FpvRemoteTests(unittest.TestCase):
    def setUp(self):
        self.fpv = Mock()
        self.fpv.status.return_value = {"enabled": False, "available": True,
                                        "mode": "native", "control_enabled": True}
        self.fpv.set_enabled.return_value = {"success": True}
        self.camera = Mock()
        self.camera.status.return_value = {"enabled": False}
        self.node = SimpleNamespace(
            fpv=self.fpv, camera=self.camera, _capture_lock=threading.RLock(),
            _fpv_rc_state=0, _fpv_rc_at=0.0, get_logger=lambda: Mock(),
        )

    def frame(self, state):
        namespace["update_fpv_rc"](self.node, SimpleNamespace(data=state))

    def test_default_is_web_control_off(self):
        state = namespace["fpv_status"](self.node)
        self.assertFalse(state["remote_controlled"])
        self.assertFalse(state["enabled"])
        self.fpv.set_enabled.assert_not_called()

    def test_disabled_native_ignores_a_and_does_not_report_capture_on(self):
        self.fpv.status.return_value.update(control_enabled=False, available=False)
        self.frame(2)
        state = namespace["fpv_status"](self.node)
        self.assertFalse(state["enabled"])
        self.assertFalse(state["remote_controlled"])
        self.assertIsNone(state["rc_enabled"])
        self.fpv.set_enabled.assert_not_called()

    def test_a_is_absolute_and_repeated_heartbeats_do_not_toggle(self):
        self.frame(1)
        for _ in range(5):
            self.frame(1)
        self.frame(2)
        for _ in range(5):
            self.frame(2)
        self.frame(1)
        self.assertEqual([call.args for call in self.fpv.set_enabled.call_args_list], [(False,), (True,), (False,)])
        state = namespace["fpv_status"](self.node)
        self.assertTrue(state["remote_controlled"])
        self.assertFalse(state["rc_enabled"])

    def test_disconnect_releases_web_without_starting_or_stopping(self):
        self.frame(2)
        self.fpv.set_enabled.reset_mock()
        self.frame(0)
        self.assertFalse(namespace["fpv_status"](self.node)["remote_controlled"])
        self.fpv.set_enabled.assert_not_called()

    def test_stale_receiver_releases_web_and_reconnect_applies_a(self):
        with patch.object(time, "monotonic", return_value=100.):
            self.frame(2)
        with patch.object(time, "monotonic", return_value=101.):
            self.assertFalse(namespace["fpv_status"](self.node)["remote_controlled"])
            self.frame(1)
        self.fpv.set_enabled.assert_called_with(False)

    def test_invalid_state_and_camera_conflict_do_not_start(self):
        self.frame(3)
        self.camera.status.return_value["enabled"] = True
        self.frame(2)
        self.fpv.set_enabled.assert_not_called()


class FpvHttpTests(unittest.TestCase):
    def setUp(self):
        self.camera = Mock()
        self.camera.status.return_value = {"enabled": False}
        self.fpv = Mock()
        self.fpv.status.return_value = {"enabled": False, "available": True, "message": ""}
        self.fpv.set_enabled.return_value = {"success": True, "enabled": True, "available": True}
        self.ready = Mock(return_value=False)
        self.network = Mock()
        self.node = SimpleNamespace(
            robot="robot-b", share_root=source.parent, web_root=source.parent,
            camera=self.camera, fpv=self.fpv, controller_ready=self.ready, network=self.network,
            _capture_lock=threading.RLock(), get_logger=lambda: Mock(),
            fpv_remote=False,
        )
        self.node.fpv_status = lambda: {**self.fpv.status(), "remote_controlled": self.node.fpv_remote,
                                        "rc_enabled": False if self.node.fpv_remote else None}
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), namespace["handler"](self.node))
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)

    def request(self, suffix, payload=None, headers=None):
        connection = HTTPConnection(*self.server.server_address, timeout=3)
        body = None if payload is None else json.dumps(payload)
        try:
            connection.request("GET" if body is None else "POST", "/api/robots/robot-b/" + suffix,
                               body=body, headers={"X-Go2-Application": "unitree-go2-sdk", **(headers or {})})
            response = connection.getresponse()
            return response.status, json.loads(response.read())
        finally:
            connection.close()

    def test_status_and_capture_work_without_motion_controller(self):
        self.assertEqual(self.request("fpv/status"), (200, self.node.fpv_status()))
        self.assertEqual(self.request("fpv/capture", {"enabled": True})[0], 200)
        self.fpv.set_enabled.assert_called_once_with(True)
        self.assertEqual(self.request("mode", {"mode": "terrain"})[0], 503)

    def test_removed_mode_is_rejected_without_calling_controller(self):
        self.ready.return_value = True
        self.node.mode_client = Mock()
        self.node.set_mode = lambda mode: namespace["set_mode"](self.node, mode)
        for mode in ("run", "heart", "right_flip"):
            with self.subTest(mode=mode):
                self.assertEqual(self.request("mode", {"mode": mode}), (400, {"error": "unknown mode"}))
        self.node.mode_client.wait_for_service.assert_not_called()
        self.node.mode_client.call_async.assert_not_called()

    def test_rejected_post_does_not_corrupt_next_request(self):
        for suffix, headers, expected in (
            ("mode", {}, 503),
            ("fpv/capture", {"X-Go2-Application": "wrong-client"}, 409),
            ("unknown", {}, 404),
        ):
            with self.subTest(suffix=suffix, headers=headers):
                connection = HTTPConnection(*self.server.server_address, timeout=3)
                try:
                    # Exercise an accepted POST first, on the same connection.
                    connection.request("POST", "/api/robots/robot-b/fpv/capture",
                                       json.dumps({"enabled": False}),
                                       {"X-Go2-Application": "unitree-go2-sdk"})
                    response = connection.getresponse()
                    self.assertEqual(response.status, 200)
                    response.read()
                    connection.request("POST", "/api/robots/robot-b/" + suffix,
                                       json.dumps({"mode": "terrain"}),
                                       {"X-Go2-Application": "unitree-go2-sdk", **headers})
                    response = connection.getresponse()
                    self.assertEqual(response.status, expected)
                    response.read()
                    connection.request("GET", "/api/robots/robot-b/fpv/status")
                    response = connection.getresponse()
                    self.assertEqual(response.status, 200)
                    self.assertEqual(json.loads(response.read()), self.node.fpv_status())
                finally:
                    connection.close()

    def test_invalid_body_length_returns_json_error_and_recovers(self):
        for length in ("invalid", "0", "65537"):
            with self.subTest(length=length):
                connection = HTTPConnection(*self.server.server_address, timeout=3)
                try:
                    connection.request("POST", "/api/robots/robot-b/fpv/capture", "{}",
                                       {"X-Go2-Application": "unitree-go2-sdk", "Content-Length": length})
                    response = connection.getresponse()
                    self.assertEqual(response.status, 400)
                    self.assertIn("error", json.loads(response.read()))
                    connection.request("GET", "/api/robots/robot-b/fpv/status")
                    response = connection.getresponse()
                    self.assertEqual(response.status, 200)
                    response.read()
                finally:
                    connection.close()

    def test_disabled_native_status_and_web_request_never_start_pipeline(self):
        self.fpv = FpvProcess("fpv_sky", str(source), 8081)
        self.node.fpv = self.fpv
        self.node.fpv_status = lambda: namespace["fpv_status"](self.node)
        with patch.object(subprocess, "Popen") as start, patch.object(shutil, "which") as probe:
            status, state = self.request("fpv/status")
            self.assertEqual(status, 200)
            self.assertEqual(state["mode"], "native")
            self.assertFalse(state["available"])
            self.assertFalse(state["control_enabled"])
            status, result = self.request("fpv/capture", {"enabled": True})
            self.assertEqual(status, 503)
            self.assertFalse(result["enabled"])
            self.assertEqual(result["message"], "图传暂未启用")
            self.assertEqual(self.request("fpv/capture", {"enabled": False})[0], 200)
            start.assert_not_called()
            probe.assert_not_called()

    def test_legacy_client_cannot_control_sdk_service(self):
        for identity in ("", "unitree-go2-gitlab"):
            status, _body = self.request("fpv/capture", {"enabled": True},
                                         {"X-Go2-Application": identity})
            self.assertEqual(status, 409)
        self.fpv.set_enabled.assert_not_called()

    def test_remote_connected_makes_web_read_only_even_when_a_is_off(self):
        self.node.fpv_remote = True
        state = self.request("fpv/status")[1]
        self.assertTrue(state["remote_controlled"])
        self.assertFalse(state["rc_enabled"])
        self.assertEqual(self.request("fpv/capture", {"enabled": True})[0], 409)
        self.assertEqual(self.request("fpv/capture", {"enabled": False})[0], 409)
        self.fpv.set_enabled.assert_not_called()
        self.node.fpv_remote = False
        self.assertEqual(self.request("fpv/capture", {"enabled": True})[0], 200)

    def test_invalid_capture_payload_does_not_start(self):
        for payload in ({"enabled": 1}, {"enabled": "true"}, {}, []):
            self.assertEqual(self.request("fpv/capture", payload)[0], 400)
        self.fpv.set_enabled.assert_not_called()

    def test_camera_preview_blocks_capture_start_but_allows_stop(self):
        self.camera.status.return_value["enabled"] = True
        self.assertEqual(self.request("fpv/capture", {"enabled": True})[0], 409)
        self.fpv.set_enabled.assert_not_called()
        self.assertEqual(self.request("fpv/capture", {"enabled": False})[0], 200)
        self.fpv.set_enabled.assert_called_once_with(False)

    def test_capture_blocks_camera_preview(self):
        self.ready.return_value = True
        self.fpv.status.return_value["enabled"] = True
        self.assertEqual(self.request("camera/toggle", {"enabled": True})[0], 409)
        self.camera.set_enabled.assert_not_called()

    def test_missing_camera_returns_actionable_failure(self):
        self.ready.return_value = True
        self.camera.set_enabled.return_value = False
        self.camera.status.return_value = {"enabled": False, "message": "未识别到 USB 摄像头"}
        self.assertEqual(self.request("camera/toggle", {"enabled": True}),
                         (503, {"error": "未识别到 USB 摄像头"}))

    def test_failed_capture_returns_failure_state(self):
        self.fpv.set_enabled.return_value = {"success": False, "enabled": False, "message": "missing camera"}
        self.assertEqual(self.request("fpv/capture", {"enabled": True}),
                         (503, self.fpv.set_enabled.return_value))

    def test_wrong_robot_header_does_not_start(self):
        self.assertEqual(self.request("fpv/capture", {"enabled": True}, {"X-Go2-Robot": "go2"})[0], 409)
        self.fpv.set_enabled.assert_not_called()


if __name__ == "__main__":
    unittest.main()
