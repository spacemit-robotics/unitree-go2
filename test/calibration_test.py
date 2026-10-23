#!/usr/bin/env python3
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0
import copy
import importlib.util
import io
import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import yaml

APP = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("calibrate", APP / "tool/1.calibration/calibrate_robot.py")
CAL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CAL)


class CalibrationTest(unittest.TestCase):
    def setUp(self):
        self.base = yaml.safe_load((APP / "config/go2.yaml").read_text())
        self.pose = yaml.safe_load((APP / "tool/1.calibration/calibration_pose_e.yaml").read_text())
        self.means = [6.4351415, 4.2863, 1.7307105, 1.215882, 3.0341915, 0.8320046,
                      0.8493384, -1.6696545, 26.87127, 9.6983445, 7.062831, -22.18855]

    def test_verified_capture_and_pose_a(self):
        original = copy.deepcopy(self.base)
        params = CAL.generate(self.base, self.pose, self.means)["go2"]["ros__parameters"]
        expected = [-0.975594794, 1.979156222, -2.968368232,
                    -0.249503797, 0.822680472, -2.757097408,
                    0.408367908, 1.799210693, -3.150770181,
                    1.197369783, 0.947208086, -2.764725663]
        for actual, want in zip(params["joint_zero_offsets_rad"], expected):
            self.assertAlmostEqual(actual, want, places=8)
        self.assertEqual(self.base, original)
        self.assertEqual(self.pose["joint_positions_deg"][::3], [2.35, -3.29, 15.71, -19.18])
        for i in range(12):
            zero = params["joint_zero_offsets_rad"][i]
            direction, ratio = CAL.DIRECTIONS[i], CAL.RATIOS[i]
            self.assertAlmostEqual(direction * self.means[i] / ratio + zero,
                                   math.radians(self.pose["joint_positions_deg"][i]), places=8)
            self.assertAlmostEqual(direction * params["startup_reference_rotor_positions_rad"][i]
                                   / ratio + zero, params["startup_reference_joint_positions_rad"][i], places=8)

    def test_integer_turn_invariance(self):
        a = CAL.generate(self.base, self.pose, self.means)["go2"]["ros__parameters"]
        shifted = [q + 2 * math.pi * (i - 5) for i, q in enumerate(self.means)]
        b = CAL.generate(self.base, self.pose, shifted)["go2"]["ros__parameters"]
        for i in range(12):
            self.assertAlmostEqual(b["joint_zero_offsets_rad"][i] - a["joint_zero_offsets_rad"][i],
                                   -CAL.DIRECTIONS[i] * (shifted[i] - self.means[i]) / CAL.RATIOS[i], places=8)
            self.assertAlmostEqual(b["startup_reference_rotor_positions_rad"][i] -
                                   a["startup_reference_rotor_positions_rad"][i],
                                   shifted[i] - self.means[i], places=8)

    def test_local_config_preserves_identity_and_calibration(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "robot.yaml"
            path.write_text((APP / "config/robot.example.yaml").read_text())
            template = CAL.load_config(path)
            self.assertEqual(template["go2"]["ros__parameters"]["joint_zero_offsets_rad"], [0.0] * 12)
            self.assertEqual(template["go2"]["ros__parameters"]["startup_reference_rotor_positions_rad"], [0.0] * 12)
            local = {"robot": {"id": "robot-001", "ros_domain_id": 211, "board_profile": "new_bigcore"},
                     "go2": {"ros__parameters": {"joint_zero_offsets_rad": [0.125] * 12}}}
            path.write_text(yaml.safe_dump(local))
            resolved = CAL.load_config(path)
            self.assertEqual(resolved["robot"], local["robot"])
            self.assertEqual(resolved["go2"]["ros__parameters"]["serial_devices"],
                             ["/dev/ttyS8", "/dev/ttyS5", "/dev/ttyS6", "/dev/ttyS3"])
            result = CAL.generate(resolved, self.pose, self.means)
            self.assertEqual(result["robot"], local["robot"])
            self.assertNotEqual(result["go2"]["ros__parameters"]["joint_zero_offsets_rad"], [0.125] * 12)
            for i in range(12):
                params = result["go2"]["ros__parameters"]
                self.assertAlmostEqual(CAL.DIRECTIONS[i] * params["startup_reference_rotor_positions_rad"][i]
                                       / CAL.RATIOS[i] + params["joint_zero_offsets_rad"][i],
                                       params["startup_reference_joint_positions_rad"][i], places=8)

    def test_three_phase_reference_and_partial_updates(self):
        pose = yaml.safe_load((APP / "tool/1.calibration/calibration_pose_xyz.yaml").read_text())
        degrees = pose["joint_positions_deg"]
        self.assertEqual(degrees[1], degrees[4])
        self.assertEqual(degrees[2], degrees[5])
        self.assertEqual(degrees[7], degrees[10])
        self.assertEqual(degrees[8], degrees[11])
        self.assertEqual(degrees[0], -degrees[3])
        self.assertEqual(degrees[6], -degrees[9])

        original = copy.deepcopy(self.base)
        result = CAL.update_phase(self.base, "poseY", pose, {3: 1.25, 9: -2.5})
        old = original["go2"]["ros__parameters"]
        new = result["go2"]["ros__parameters"]
        self.assertEqual(self.base, original)
        for index in range(12):
            if index not in CAL.PHASES["poseY"]:
                self.assertEqual(new["joint_zero_offsets_rad"][index], old["joint_zero_offsets_rad"][index])
                self.assertEqual(new["startup_reference_rotor_positions_rad"][index],
                                 old["startup_reference_rotor_positions_rad"][index])
        self.assertEqual(set(CAL.PHASES["poseX"]), {1, 2, 4, 5, 7, 8, 10, 11})
        self.assertEqual(set(CAL.PHASES["poseY"]), {3, 9})
        self.assertEqual(set(CAL.PHASES["poseZ"]), {0, 6})

    def test_three_phase_capture_merge(self):
        pose = yaml.safe_load((APP / "tool/1.calibration/calibration_pose_xyz.yaml").read_text())
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            captures = []
            for phase in CAL.PHASES:
                directory = root / phase
                (directory / "sdk").mkdir(parents=True)
                (directory / "phase.txt").write_text(phase + "\n")
                (directory / "samples.txt").write_text("20\n")
                (directory / "base_config.yaml").write_text(yaml.safe_dump(self.base, sort_keys=False))
                (directory / "reference.yaml").write_text(yaml.safe_dump(pose, sort_keys=False))
                for index in CAL.PHASES[phase]:
                    joint = CAL.JOINTS[index]
                    value = self.means[index]
                    (directory / "sdk" / f"{joint[:2]}_id{index % 3}.txt").write_text(
                        f"reply id={index % 3} error=0 q={value}\n" * 20)
                captures.append(directory)
            output = root / "combined.yaml"
            CAL.combine_phases(captures, output)
            merged = yaml.safe_load(output.read_text())
            params = merged["go2"]["ros__parameters"]
            for phase in CAL.PHASES:
                expected = CAL.update_phase(self.base, phase, pose,
                                            CAL.phase_rotor_means(root / phase, phase, 20))
                expected_params = expected["go2"]["ros__parameters"]
                for index in CAL.PHASES[phase]:
                    self.assertEqual(params["joint_zero_offsets_rad"][index],
                                     expected_params["joint_zero_offsets_rad"][index])
                    self.assertEqual(params["startup_reference_rotor_positions_rad"][index],
                                     expected_params["startup_reference_rotor_positions_rad"][index])

    def fake_phase_capture(self, directory, config, reference, _tool, samples, phase):
        (directory / "sdk").mkdir(parents=True)
        (directory / "phase.txt").write_text(phase + "\n")
        (directory / "samples.txt").write_text(str(samples))
        (directory / "base_config.yaml").write_text(yaml.safe_dump(config))
        (directory / "reference.yaml").write_text(yaml.safe_dump(reference))
        for index in CAL.PHASES[phase]:
            path = directory / "sdk" / f"{CAL.JOINTS[index][:2]}_id{index % 3}.txt"
            path.write_text(f"reply id={index % 3} error=0 q={self.means[index]}\n" * samples)

    def test_interactive_countdown_and_confirm_preserve_local_fields(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            base = root / "robot.yaml"
            original = yaml.safe_load((APP / "config/robots/go2air2.yaml").read_text())
            base.write_text(yaml.safe_dump(original))
            base.chmod(0o600)
            capture_root, output = root / "captures", root / "result.yaml"
            events = []

            def answer(prompt):
                self.assertEqual(yaml.safe_load(base.read_text()), original)
                events.append("input")
                return "确认标定" if "确认标定" in prompt else ""

            def capture(*args):
                events.append("capture:" + args[-1])
                self.fake_phase_capture(*args)

            with patch("builtins.input", side_effect=answer), \
                    patch.object(CAL.time, "sleep", side_effect=lambda _seconds: events.append("sleep")) as sleep, \
                    patch.object(CAL, "capture_phase", side_effect=capture), \
                    patch("sys.stdout", new_callable=io.StringIO) as messages:
                CAL.interactive_calibration(capture_root, base, APP / "tool/1.calibration/calibration_pose_xyz.yaml",
                                            root / "tool", 20, output)
            self.assertEqual(events, sum((["input"] + ["sleep"] * 5 + ["capture:" + phase]
                                          for phase in CAL.PHASES), []) + ["input"])
            self.assertEqual(sleep.call_count, 15)
            self.assertTrue(all(call.args == (1,) for call in sleep.call_args_list))
            changed = yaml.safe_load(base.read_text())
            self.assertEqual(changed, yaml.safe_load(output.read_text()))
            self.assertNotEqual(changed, original)
            for key in ("joint_zero_offsets_rad", "startup_reference_rotor_positions_rad"):
                changed["go2"]["ros__parameters"][key] = original["go2"]["ros__parameters"][key]
            self.assertEqual(changed, original)
            self.assertEqual(base.stat().st_mode & 0o777, 0o600)
            self.assertFalse(list(root.glob(".calibration-*")))
            text = messages.getvalue()
            self.assertLess(text.index("可以松手"), text.index("关节           原零偏"))

    def test_interactive_cancel_retry_and_changed_config(self):
        for scenario in ("early_cancel", "decline", "retry", "changed_config", "interrupt"):
            with self.subTest(scenario=scenario), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                base = root / "robot.yaml"
                original = (APP / "config/robots/go2air2.yaml").read_text()
                base.write_text(original)
                output = root / "result.yaml"
                captures = []

                def answer(prompt):
                    if scenario == "early_cancel":
                        return "q"
                    if "确认标定" in prompt:
                        if scenario == "changed_config":
                            base.write_text(original + "\n# changed during capture\n")
                        return "" if scenario == "decline" else "确认标定"
                    return ""

                def capture(*args):
                    captures.append(args[0])
                    self.fake_phase_capture(*args)
                    if scenario == "retry" and len(captures) == 1:
                        path = args[0] / "sdk/FR_id1.txt"
                        path.write_text(path.read_text() + "reply id=1 error=0 q=100\n")

                with patch("builtins.input", side_effect=answer), \
                        patch.object(CAL.time, "sleep") as sleep, \
                        patch.object(CAL, "capture_phase", side_effect=capture), \
                        patch("sys.stdout", new_callable=io.StringIO):
                    if scenario == "interrupt":
                        sleep.side_effect = KeyboardInterrupt
                    def run():
                        CAL.interactive_calibration(root / "captures", base,
                            APP / "tool/1.calibration/calibration_pose_xyz.yaml", root / "tool", 20, output)
                    if scenario == "changed_config":
                        with self.assertRaisesRegex(ValueError, "标定期间已变化"):
                            run()
                    elif scenario == "interrupt":
                        with self.assertRaises(KeyboardInterrupt):
                            run()
                    else:
                        run()
                if scenario == "retry":
                    self.assertEqual([p.name for p in captures], ["poseX", "poseX-2", "poseY", "poseZ"])
                    self.assertEqual(yaml.safe_load(base.read_text()), yaml.safe_load(output.read_text()))
                elif scenario == "changed_config":
                    self.assertTrue(base.read_text().endswith("# changed during capture\n"))
                else:
                    self.assertEqual(base.read_text(), original)
                if scenario in ("early_cancel", "interrupt"):
                    self.assertFalse(captures)
                    self.assertFalse(output.exists())
                self.assertFalse(list(root.glob(".calibration-*")))

    def test_no_arguments_enters_interactive_xyz(self):
        with patch("sys.argv", ["calibrate_robot.py"]), patch.object(CAL, "interactive_calibration") as wizard:
            CAL.main()
        wizard.assert_called_once()
        self.assertEqual(wizard.call_args.args[2], APP / "tool/1.calibration/calibration_pose_xyz.yaml")

    def test_reject_bad_reference_and_hardware(self):
        for value in (float("nan"), float("inf"), "2.35", True):
            bad = copy.deepcopy(self.pose)
            bad["joint_positions_deg"][0] = value
            with self.assertRaises(ValueError):
                CAL.generate(self.base, bad, self.means)
        bad = copy.deepcopy(self.base)
        bad["go2"]["ros__parameters"]["joint_directions"][0] = -1
        with self.assertRaises(ValueError):
            CAL.generate(bad, self.pose, self.means)

    def test_sample_validation(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for i, joint in enumerate(CAL.JOINTS):
                (root / f"{joint[:2]}_id{i % 3}.txt").write_text(
                    f"reply id={i % 3} error=0 q={self.means[i]}\n" * 20)
            self.assertEqual(CAL.rotor_means(root, 20), self.means)
            path = root / "FR_id0.txt"
            for bad in ("reply id=0 error=0 q=nan\n" * 20,
                        "reply id=1 error=0 q=0\n" * 20,
                        "reply id=0 error=1 q=0\n" * 20,
                        "reply id=0 error=0 q=0\n" * 19,
                        "reply id=0 error=0 q=0\n" * 19 + "reply id=0 error=0 q=6.28\n"):
                path.write_text(bad)
                with self.assertRaises(ValueError):
                    CAL.rotor_means(root, 20)


if __name__ == "__main__":
    unittest.main()
