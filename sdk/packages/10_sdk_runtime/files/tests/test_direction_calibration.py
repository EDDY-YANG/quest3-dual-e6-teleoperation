"""Offline coordinate/calibration contract tests; no live input injection."""

import copy
import time
import unittest

import numpy as np

from direction_calibration import DirectionCalibration
from operator_frame import OperatorFrame
from headset_bridge import validate_calibration_action
from relative_pose import incremental_pose_target


class DirectionTests(unittest.TestCase):
    def setUp(self):
        self.calibration = DirectionCalibration()
        self.sample_clock = time.monotonic()
        self.robot = {"valid": True, "connected": True, "robot_mode": 5, "error_status": 0,
                      "user": 0, "tool": 0, "robot_id": "left-fixture", "frame_id": "base-tool0",
                      "host_monotonic_s": time.monotonic()}

    def quest(self, position, seq, *, epoch=0):
        return {"connected": True, "age_ms": 0, "session": "isolated",
                "frame": {"seq": seq, "host_monotonic_s": self.sample_clock + seq*0.01,
                          "reference_space": "local-floor", "reference_epoch": epoch,
                          "visibility": "visible", "input_policy": "continuous-pose-grip-v2",
                          "hands": {"left": {"is_controller": True, "tracking_valid": True,
                                             "grip_held": False,
                                             "gamepad": {"buttons": [{"value": 0}, {"value": 0}]},
                                             "target_ray": {"quaternion_xyzw": [0, 0, 0, 1], "emulated_position": False},
                                             "pose": {"position_m": position,
                                                      "quaternion_xyzw": [0, 0, 0, 1],
                                                      "emulated_position": False}}}}}

    def calibrate(self):
        for action, position, seq in (("begin", [0, 1, 0], 1),
                                      ("capture_x", [0, 1, -.1], 2)):
            self.calibration.apply_action(action, self.quest(position, seq), self.robot)
        quest = self.quest([-.1, 1, -.1], 4)
        self.assertEqual(self.calibration.snapshot(quest, self.robot)["state"], "CALIBRATED")
        self.assertTrue(self.calibration.snapshot(quest, self.robot)["auto_apply"])
        self.calibration.operator.capture(quest, "left")
        return quest

    def test_x_only_auto_applies_and_maps_world_translation_independent_of_tcp(self):
        quest = self.calibrate()
        matrix = self.calibration.matrix_for(quest, self.robot)
        np.testing.assert_allclose(matrix, [[0, 0, -1], [-1, 0, 0], [0, 1, 0]], atol=1e-10)
        before = {"position_m": [0, 0, 0], "quaternion_xyzw": [0, 0, 0, 1]}
        after = {"position_m": [.1, 0, 0], "quaternion_xyzw": [0, 0, 0, 1]}
        for rotation in ([0, 0, 0], [45, -20, 130]):
            target = incremental_pose_target(before, [10, 20, 30] + rotation, after, matrix)
            np.testing.assert_allclose(target[:3], [10, -80, 30], atol=1e-10)
        self.assertTrue(self.calibration.snapshot(quest, self.robot)["current"])

    def test_epoch_or_robot_change_invalidates_mapping(self):
        quest = self.calibrate()
        changed = copy.deepcopy(quest)
        changed["frame"]["reference_epoch"] += 1
        self.assertFalse(self.calibration.snapshot(changed, self.robot)["current"])
        with self.assertRaisesRegex(ValueError, "CALIBRATION_REQUIRED"):
            self.calibration.matrix_for(changed, self.robot)
        with self.assertRaisesRegex(ValueError, "CALIBRATION_REQUIRED"):
            self.calibration.matrix_for(quest, {**self.robot, "robot_id": "another-arm"})

    def test_emulated_stale_or_held_input_is_never_a_calibration_sample(self):
        for kind in ("emulated", "stale", "held"):
            quest = self.quest([0, 1, 0], 1)
            hand = quest["frame"]["hands"]["left"]
            if kind == "emulated":
                hand["pose"]["emulated_position"] = True
            elif kind == "stale":
                quest["age_ms"] = 201
            else:
                hand["grip_held"] = True
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                self.calibration.apply_action("begin", quest, self.robot)

    def test_short_or_vertical_input_is_rejected_without_advancing_stage(self):
        self.calibration.apply_action("begin", self.quest([0, 0, 0], 1), self.robot)
        with self.assertRaisesRegex(ValueError, "10mm"):
            self.calibration.apply_action("capture_x", self.quest([.01, 0, 0], 2), self.robot)
        self.assertEqual(self.calibration.stage, "WAIT_X")
        self.assertIn("至少需要80mm", self.calibration.last_error)
        with self.assertRaisesRegex(ValueError, "0mm"):
            self.calibration.apply_action("capture_x", self.quest([0, .2, 0], 3), self.robot)
        self.assertEqual(self.calibration.snapshot(self.quest([0, .2, 0], 3), self.robot)["live_horizontal_distance_mm"], 0)
        self.calibration.apply_action("capture_x", self.quest([.1, 0, 0], 4), self.robot)
        self.assertEqual(self.calibration.stage, "CALIBRATED")
        self.assertIsNone(self.calibration.last_error)
        self.assertEqual(len(self.calibration.active["samples"]), 2)

    def test_messages_cannot_supply_matrix_or_robot_commands(self):
        valid = {"type": "calibration_action", "id": "c1", "action": "begin"}
        self.assertEqual(validate_calibration_action(valid), ("c1", "begin"))
        for invalid in ({**valid, "matrix": [[1, 0, 0]] * 3},
                        {**valid, "action": "ServoJ"}):
            with self.assertRaises(ValueError):
                validate_calibration_action(invalid)

    def test_vertical_sampling_error_cannot_tilt_or_flip_up_axis(self):
        for action, p, seq in (("begin", [0, 1, 0], 1), ("capture_x", [0, 1.03, -.1], 2)):
            q = self.quest(p, seq)
            self.calibration.apply_action(action, q, self.robot)
        self.calibration.operator.capture(q, "left")
        matrix = np.array(self.calibration.matrix_for(q, self.robot))
        np.testing.assert_allclose(matrix @ [0, 1, 0], [0, 0, 1], atol=1e-12)
        np.testing.assert_allclose(matrix @ [0, 0, -.1], [.1, 0, 0], atol=1e-12)

    def test_retired_y_and_apply_actions_cannot_modify_committed_mapping(self):
        quest = self.calibrate()
        matrix = self.calibration.matrix_for(quest, self.robot)
        revision = self.calibration.revision
        for action in ("capture_y", "apply"):
            with self.assertRaisesRegex(ValueError, "新版只采X"):
                self.calibration.apply_action(action, quest, self.robot)
            self.assertEqual(self.calibration.stage, "CALIBRATED")
            self.assertEqual(self.calibration.revision, revision)
            self.assertEqual(self.calibration.matrix_for(quest, self.robot), matrix)

    def test_front_capture_is_explicit_and_frozen_despite_wrist_rotation(self):
        quest = self.calibrate()
        operator_before = copy.deepcopy(self.calibration.operator.active)
        matrix = self.calibration.matrix_for(quest, self.robot)
        turned = copy.deepcopy(quest)
        h = turned["frame"]["hands"]["left"]
        h["pose"]["quaternion_xyzw"] = [0, 2**-.5, 0, 2**-.5]
        h["target_ray"]["quaternion_xyzw"] = [0, 2**-.5, 0, 2**-.5]
        np.testing.assert_allclose(self.calibration.matrix_for(turned, self.robot), matrix)
        self.assertEqual(self.calibration.operator.active, operator_before)

    def test_forward_uses_target_ray_and_discards_pitch(self):
        quest = self.quest([0, 1, 0], 1)
        quest["frame"]["hands"]["left"]["target_ray"]["quaternion_xyzw"] = [float(np.sin(np.pi/12)), 0, 0, float(np.cos(np.pi/12))]
        # Different grip orientation must not override the platform's pointing ray.
        quest["frame"]["hands"]["left"]["pose"]["quaternion_xyzw"] = [0, 2**-.5, 0, 2**-.5]
        op = OperatorFrame()
        op.capture(quest, "left")
        np.testing.assert_allclose(op.active["forward_xr"], [0, 0, -1], atol=1e-12)
        quest["frame"]["hands"]["left"]["target_ray"]["quaternion_xyzw"] = [2**-.5, 0, 0, 2**-.5]
        with self.assertRaisesRegex(ValueError, "水平方向不足"):
            op.capture(quest, "left")
        np.testing.assert_allclose(op.active["forward_xr"], [0, 0, -1], atol=1e-12)

    def test_dual_base_headings_map_shared_forward_to_different_axes(self):
        quest = self.calibrate()
        right = DirectionCalibration("right", self.calibration.operator)
        robot = {**self.robot, "robot_id": "right-fixture", "frame_id": "right-frame"}
        for action, p, seq in (("begin", [0, 1, 0], 10), ("capture_x", [.1, 1, 0], 11)):
            q = self.quest(p, seq)
            q["frame"]["hands"]["right"] = q["frame"]["hands"]["left"]
            right.apply_action(action, q, robot)
        left_map = self.calibration.snapshot(quest, self.robot)["R_base_from_operator"]
        right_map = right.snapshot(q, robot)["R_base_from_operator"]
        np.testing.assert_allclose(np.array(left_map) @ [.01, 0, 0], [.01, 0, 0], atol=1e-12)
        np.testing.assert_allclose(np.array(right_map) @ [.01, 0, 0], [0, .01, 0], atol=1e-12)
        self.assertAlmostEqual(right.snapshot(q, robot)["operator_forward_yaw_deg"], 90)
        right.apply_action("clear", q, robot)
        self.assertTrue(self.calibration.snapshot(quest, self.robot)["control_ready"])

    def test_missing_ray_and_unrecorded_operator_block_control(self):
        q = self.calibrate()
        self.calibration.operator.active = None
        with self.assertRaisesRegex(ValueError, "OPERATOR_FORWARD_REQUIRED"):
            self.calibration.matrix_for(q, self.robot)
        del q["frame"]["hands"]["left"]["target_ray"]
        with self.assertRaisesRegex(ValueError, "刷新Quest"):
            self.calibration.operator.capture(q, "left")


if __name__ == "__main__":
    unittest.main()
