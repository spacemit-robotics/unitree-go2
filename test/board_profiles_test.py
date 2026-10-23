#!/usr/bin/env python3
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0
"""Evaluate launch actions without executing or opening any hardware."""

import importlib.util
import math
import sys
import tempfile
from pathlib import Path

import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("go2_launch", root / "launch/go2.launch.py")
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


# Evaluate the current source tree, without requiring an installed ROS package.
def source_share(package):
    assert package == "unitree_go2"
    return str(root.resolve())


m.get_package_share_directory = source_share


def resolve(include_config=False, include_domain=False, **overrides):
    context = LaunchContext()
    for action in m.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            context.launch_configurations[action.name] = perform_substitutions(context, action.default_value)
    context.launch_configurations.update(
        board_profiles=str(root / "config/board_profiles.yaml"), sbus="true", web="true",
        local_config=str(root / "test/nonexistent-local.yaml")
    )
    context.launch_configurations.update(overrides)
    actions = m._make_node(context)
    for action in actions:
        if isinstance(action, SetEnvironmentVariable):
            action.execute(context)
    actions = [action for action in actions if isinstance(action, Node)]
    # ROS launch keeps normalized parameters on the Node action until execution.
    result = [evaluate_parameters(context, a._Node__parameters)[-1] for a in actions]
    if include_config:
        config = evaluate_parameters(context, actions[0]._Node__parameters)[0]
        result[0] = {**yaml.safe_load(Path(config).read_text())["go2"]["ros__parameters"], **result[0]}
    return (context.environment["ROS_DOMAIN_ID"], result) if include_domain else result


legacy = resolve()
assert legacy[0]["walking_tilt_protection"] is True
assert resolve(walking_tilt_protection="false")[0]["walking_tilt_protection"] is False
assert legacy[-1]["ap_band"] == "a" and legacy[-1]["ap_channel"] == 0
assert resolve(ap_band="bg", ap_channel="6")[-1]["ap_band"] == "bg"
assert resolve(ap_band="bg", ap_channel="6")[-1]["ap_channel"] == 6
assert Path(legacy[-1]["fpv_executable"]).name == "fpv_sky"
assert Path(legacy[-1]["fpv_config"]).name == "sky.yaml"
assert legacy[-1]["fpv_control_port"] == 8081
assert legacy[-1]["fpv_control_enabled"] is False
assert resolve(fpv_control_enabled="true")[-1]["fpv_control_enabled"] is True
assert "fpv_mode" not in legacy[-1]
assert "fpv_script" not in legacy[-1]
assert "fpv_autostart" not in legacy[-1]

# Temporary synthetic board profiles exercise identity and calibration isolation.
with tempfile.TemporaryDirectory() as temporary:
    local_path = Path(temporary) / "robot.yaml"
    for index in range(2):
        local = {"robot": {"id": f"robot-{index}", "name": f"Go2 {index}", "ros_domain_id": 211 + index,
                           "board_profile": "new_bigcore", "walking_tilt_protection": bool(index),
                           "wifi_connection": "Saved Wi-Fi", "ap_ssid": f"Go2-{index}"},
                 "go2": {"ros__parameters": {"joint_zero_offsets_rad": [0.25 + index] * 12,
                                            "startup_reference_rotor_positions_rad": [1.5 + index] * 12,
                                            "imu_to_body_quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                                            "imu_to_body_matrix": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]}}}
        local_path.write_text(yaml.safe_dump(local))
        domain, nodes = resolve(include_domain=True, include_config=True,
                                local_config=str(local_path), backend="serial")
        assert domain == str(211 + index)
        assert nodes[0]["robot"] == f"robot-{index}"
        assert list(nodes[0]["joint_zero_offsets_rad"]) == [0.25 + index] * 12
        assert list(nodes[0]["startup_reference_rotor_positions_rad"]) == [1.5 + index] * 12
        assert list(nodes[0]["imu_to_body_quaternion_wxyz"]) == [1.0, 0.0, 0.0, 0.0]
        assert nodes[0]["walking_tilt_protection"] is bool(index)
        assert nodes[0]["motor_io_authorized"] is True
        assert nodes[-1]["robot_name"] == f"Go2 {index}"
        assert nodes[-1]["wifi_connection"] == "Saved Wi-Fi"
        assert nodes[-1]["ap_ssid"] == f"Go2-{index}"
        assert resolve(local_config=str(local_path))[0]["motor_io_authorized"] is False
        override = resolve(local_config=str(local_path), walking_tilt_protection="false")[0]
        assert override["walking_tilt_protection"] is False
        assert resolve(include_domain=True, local_config=str(local_path), ros_domain_id="197")[0] == "197"
    # A calibration produced by the tool contains shared defaults too. Launch
    # arguments must still select the actual backend, identity and board.
    local["go2"]["ros__parameters"].update(backend="mock", robot="ignored", board_profile="legacy_usb")
    local_path.write_text(yaml.safe_dump(local))
    node = resolve(local_config=str(local_path), backend="serial")[0]
    assert node["backend"] == "serial" and node["robot"] == "robot-1" and node["board_profile"] == "new_bigcore"
    local["robot"]["board_profile"] = "legacy_usb"
    local["go2"]["ros__parameters"].update(imu_driver="drv_spi_bmi270", imu_device="/dev/iio:device1")
    local_path.write_text(yaml.safe_dump(local))
    node = resolve(local_config=str(local_path))[0]
    assert node["imu_driver"] == "drv_spi_bmi270" and node["imu_device"] == "/dev/iio:device1"
    local["robot"]["board_profile"] = "new_bigcore"
    for invalid in ([], [0.0] * 11, [float("nan")] * 12, [True] * 12):
        local["go2"]["ros__parameters"]["joint_zero_offsets_rad"] = invalid
        local_path.write_text(yaml.safe_dump(local))
        try:
            resolve(local_config=str(local_path), backend="serial")
        except RuntimeError:
            pass
        else:
            raise AssertionError("serial launch must reject missing or invalid local calibration")
try:
    resolve(backend="serial")
except RuntimeError:
    pass
else:
    raise AssertionError("serial launch must require a local config")
assert resolve(include_domain=True, robot="any-new-robot")[0] == "217"
for domain in ("-1", "233", "not-a-number"):
    try:
        resolve(ros_domain_id=domain)
    except (RuntimeError, ValueError):
        pass
    else:
        raise AssertionError("invalid DDS domain must fail")
assert "imu_gpiochip" not in resolve(imu="bmi270")[0]
for robot in ("../robot", "", "invalid robot", "x" * 65):
    if not robot:
        continue  # Empty explicitly selects the board-local identity.
    try:
        resolve(robot=robot)
    except RuntimeError:
        pass
    else:
        raise AssertionError("unknown robot must fail")
assert legacy[0]["board_profile"] == "legacy_usb"
assert legacy[0]["imu_driver"] == "drv_uart_iwt603"
assert "serial_devices" not in legacy[0]  # retain existing robot YAML USB mapping
assert legacy[1]["device"] == "/dev/ttyS5"
assert resolve(imu="bmi270")[0]["imu_device"] == "/dev/bmi270"
assert resolve(imu="bmi270", imu_device="/dev/spidev0.0")[0]["imu_device"] == "/dev/spidev0.0"
mount_keys = ("imu_to_body_matrix", "imu_to_body_quaternion_wxyz", "imu_roll_offset_deg", "imu_pitch_offset_deg")
assert all(k not in legacy[0] for k in mount_keys)
bmi = resolve(imu="bmi270")[0]
assert "imu_gpiochip" not in bmi
assert "imu_gpio_line" not in bmi
assert "imu_gpiochip" not in legacy[0]
assert all(k in bmi for k in mount_keys)
explicit = resolve(imu_driver="drv_spi_bmi270")[0]
assert all(list(explicit[k]) == list(bmi[k]) for k in mount_keys[:2])
q = list(bmi["imu_to_body_quaternion_wxyz"])
matrix = list(bmi["imu_to_body_matrix"])
w, x, y, z = q
inverse_rotation = [
    1 - 2 * (y * y + z * z),
    2 * (x * y + w * z),
    2 * (x * z - w * y),
    2 * (x * y - w * z),
    1 - 2 * (x * x + z * z),
    2 * (y * z + w * x),
    2 * (x * z + w * y),
    2 * (y * z - w * x),
    1 - 2 * (x * x + y * y),
]
assert abs(sum(v * v for v in q) - 1) < 1e-9
assert max(abs(a - b) for a, b in zip(matrix, inverse_rotation)) < 1e-9
# Remove the nominal yaw-axis alignment: generic defaults carry no measured tilt.
W, X, Y, Z = z, y, -x, -w
roll = math.degrees(math.atan2(2 * (W * X + Y * Z), 1 - 2 * (X * X + Y * Y)))
pitch = math.degrees(math.asin(2 * (W * Y - Z * X)))
yaw = math.degrees(math.atan2(2 * (W * Z + X * Y), 1 - 2 * (Y * Y + Z * Z)))
assert abs(roll) < 1e-6
assert abs(pitch) < 1e-6
assert abs(yaw) < 1e-6
assert bmi["imu_roll_offset_deg"] == bmi["imu_pitch_offset_deg"] == 0.0
for board in ("new_smallcore", "new_bigcore", "new_usbmotor"):
    params = resolve(board_profile=board)
    expected_driver = "go2_smallcore" if board == "new_smallcore" else "drv_485_m8010a"
    assert params[0]["motor_driver"] == expected_driver
    assert params[0]["imu_gpiochip"] == ""
    assert params[0]["imu_gpio_line"] == 0
    assert params[0]["imu_device"] == "/dev/iio:device0"
    assert params[1]["device"] == "/dev/ttyS7"
    assert params[0]["imu_roll_offset_deg"] == params[0]["imu_pitch_offset_deg"] == 0.0
    assert params[0]["imu_driver"] == "drv_spi_bmi270"
    assert list(params[0]["imu_to_body_quaternion_wxyz"]) == q
    assert list(params[0]["imu_to_body_matrix"]) == matrix
    expected = ["go2-motor-" + leg for leg in ("rl", "rr", "fl", "fr")] if board == "new_smallcore" else (
        ["/dev/ttyS8", "/dev/ttyS5", "/dev/ttyS6", "/dev/ttyS3"] if board == "new_bigcore" else [
            "/dev/go2/motor-fr", "/dev/go2/motor-fl", "/dev/go2/motor-rr", "/dev/go2/motor-rl",
        ]
    )
    assert list(params[0]["serial_devices"]) == expected
    for wrong in ({"imu": "iwt603"}, {"imu_driver": "drv_uart_iwt603"}, {"motor_driver": "wrong"}):
        try:
            resolve(board_profile=board, **wrong)
        except RuntimeError:
            pass
        else:
            raise AssertionError((board, wrong))
    overridden = resolve(board_profile=board, sbus_device="/dev/test-sbus")
    assert overridden[1]["device"] == "/dev/test-sbus"
# The status light follows the selected input, rather than zero Twist messages.
assert resolve(sbus="true", web="false")[0]["teleop_source"] == "sbus"
assert resolve(sbus="false", web="true")[0]["teleop_source"] == "web"
assert resolve(sbus="true", web="true")[0]["teleop_source"] == "any"
assert resolve(sbus="false", web="false")[0]["teleop_source"] == "none"
assert list(resolve(board_profile="new_bigcore")[0]["led_pwm_devices"]) == [
    "d4022400.pwm", "d4020c00.pwm", "d4022800.pwm", "d4022c00.pwm"]

print("PASS board profiles: wiring, IMU, input selection, PWM lights, overrides")
