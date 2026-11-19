"""Synthetic unit fixtures only; none of these inputs represent hardware results."""

import copy
import math
import unittest

from clutch_mapping import DualClutchMapper, FrameCalibration, MappingConfig

IDENTITY = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
Z90 = ((0, -1, 0), (1, 0, 0), (0, 0, 1))


def calibration(matrix=IDENTITY, *, verified=True, epoch=0, session="synthetic-session"):
    return FrameCalibration(
        matrix, "synthetic-test-calibration", verified, session, "local-floor", epoch, "user:0"
    )


def snapshot(grip=0, *, sequence=1, epoch=0, received=100.0):
    hand = {
        "is_controller": True,
        "pose": {
            "position_m": [0, 1, -0.4],
            "quaternion_xyzw": [0, 0, 0, 1],
            "emulated_position": False,
        },
        "gamepad": {
            "connected": True,
            "mapping": "xr-standard",
            "buttons": [{"value": 0}, {"value": grip}],
        },
    }
    return {
        "connected": True,
        "session": "synthetic-session",
        "valid": {"left": True, "right": True},
        "frame": {
            "seq": sequence,
            "reference_space": "local-floor",
            "reference_epoch": epoch,
            "host_monotonic_s": received,
            "visibility": "visible",
            "hands": {"left": copy.deepcopy(hand), "right": copy.deepcopy(hand)},
        },
    }


def robots(received=100.0):
    pose = {
        "valid": True,
        "position_mm": [300, 0, 200],
        "quaternion_xyzw": [0, 0, 0, 1],
        "host_monotonic_s": received,
        "robot_mode": 4,
        "error_status": 0,
        "frame_id": "user:0",
        "user": 0,
        "tool": 0,
    }
    return {
        side: {**copy.deepcopy(pose), "robot_id": f"synthetic-{side}"} for side in ("left", "right")
    }


class ClutchMappingTests(unittest.TestCase):
    def test_grip_gated_idle_without_pose_can_release_and_engage(self):
        mapper = self.mapper()
        idle = snapshot(0)
        idle["frame"]["input_policy"] = "grip-gated-v1"
        for side, hand in idle["frame"]["hands"].items():
            hand.update(tracking_valid=True, grip_held=False, pose=None)
            idle["valid"][side] = False
        self.assertEqual(self.update(mapper, idle)["left"]["state"], "READY")
        held = snapshot(1, sequence=2)
        held["frame"]["input_policy"] = "grip-gated-v1"
        for hand in held["frame"]["hands"].values():
            hand.update(tracking_valid=True, grip_held=True)
        self.assertTrue(self.update(mapper, held)["left"]["target_valid"])
        idle["frame"]["seq"] = 3
        self.assertEqual(self.update(mapper, idle)["left"]["state"], "RELEASED")

    def mapper(self, config=None, left=None):
        return DualClutchMapper({"left": left or calibration(), "right": calibration()}, config)

    def update(self, mapper, quest, robot=None, now=100.1):
        result = mapper.update(quest, robots() if robot is None else robot, now=now)
        self.assertEqual(result["mode"], "NO_SEND")
        self.assertFalse(result["execution_allowed"])
        for arm in result["arms"].values():
            self.assertFalse(arm["execution_allowed"])
        return result["arms"]

    def engage(self, mapper, robot=None):
        self.update(mapper, snapshot(0), robot)
        return self.update(mapper, snapshot(1, sequence=2), robot)

    def test_default_has_no_calibration_and_no_targets(self):
        out = self.update(DualClutchMapper(), snapshot(1))["left"]
        self.assertEqual(out["state"], "NOT_CALIBRATED")
        self.assertIsNone(out["target"])
        self.assertIsNone(out["draft_target"])

    def test_unverified_axis_preset_requires_explicit_preview_opt_in(self):
        preset = calibration(verified=False)
        self.assertEqual(self.engage(self.mapper(left=preset))["left"]["state"], "NOT_CALIBRATED")
        mapper = self.mapper(MappingConfig(allow_unverified_frame_preview=True), left=preset)
        out = self.engage(mapper)["left"]
        self.assertEqual(out["state"], "UNVERIFIED_FRAME_PREVIEW")
        self.assertEqual(out["clutch_state"], "FOLLOWING")
        self.assertFalse(out["calibration_verified"])
        self.assertFalse(out["target_valid"])
        self.assertIsNone(out["target"])
        self.assertEqual(out["draft_target"]["position_mm"], [300, 0, 200])

    def test_startup_held_grip_cannot_start_following(self):
        mapper = self.mapper()
        self.assertEqual(self.update(mapper, snapshot(1))["left"]["state"], "RELEASE_REQUIRED")
        self.assertEqual(
            self.update(mapper, snapshot(0.4, sequence=2))["left"]["state"], "RELEASE_REQUIRED"
        )
        self.assertEqual(self.update(mapper, snapshot(0.2, sequence=3))["left"]["state"], "READY")
        self.assertTrue(self.update(mapper, snapshot(0.7, sequence=4))["left"]["target_valid"])

    def test_metre_to_millimetre_scale_and_noncommuting_rotation_mapping(self):
        mapper = self.mapper(left=calibration(Z90))
        robot = robots()
        s = math.sqrt(0.5)
        robot["left"]["quaternion_xyzw"] = [0, 0, s, s]
        initial = self.engage(mapper, robot)["left"]["target"]
        self.assertEqual(initial["position_mm"], [300, 0, 200])
        frame = snapshot(1, sequence=3)
        frame["frame"]["hands"]["left"]["pose"].update(
            position_m=[0.1, 1, -0.4], quaternion_xyzw=[s, 0, 0, s]
        )
        target = self.update(mapper, frame, robot)["left"]["target"]
        for actual, expected in zip(target["position_mm"], [300, 30, 200]):
            self.assertAlmostEqual(actual, expected)
        # VR X rotation becomes robot Y under Z90; Y90 * anchor Z90 = (.5,.5,.5,.5).
        for actual in target["quaternion_xyzw"]:
            self.assertAlmostEqual(actual, 0.5)

    def test_hysteresis_release_cancels_and_reengagement_uses_actual_robot_pose(self):
        mapper = self.mapper()
        self.engage(mapper)
        frame = snapshot(0.4, sequence=3)
        frame["frame"]["hands"]["left"]["pose"]["position_m"][0] = 0.1
        self.assertEqual(self.update(mapper, frame)["left"]["target"]["position_mm"], [330, 0, 200])
        released = self.update(mapper, snapshot(0.2, sequence=4))["left"]
        self.assertEqual(released["state"], "RELEASED")
        self.assertFalse(released["target_valid"])
        self.assertIsNone(released["target"])
        frame = snapshot(1, sequence=5)
        frame["frame"]["hands"]["left"]["pose"]["position_m"] = [3, 2, 1]
        robot = robots()
        robot["left"]["position_mm"] = [310, 25, 205]
        target = self.update(mapper, frame, robot)["left"]["target"]
        self.assertEqual(target["position_mm"], [310, 25, 205])
        self.assertEqual((target["user"], target["tool"], target["frame_id"]), (0, 0, "user:0"))

    def test_left_failure_does_not_cancel_valid_right_arm(self):
        mapper = self.mapper()
        self.engage(mapper)
        frame = snapshot(1, sequence=3)
        frame["frame"]["hands"]["left"]["pose"]["emulated_position"] = True
        out = self.update(mapper, frame)
        self.assertIsNone(out["left"]["target"])
        self.assertTrue(out["left"]["requires_release"])
        self.assertTrue(out["right"]["target_valid"])

    def test_input_failures_require_valid_release_and_new_press(self):
        mutations = [
            lambda f: f.update(connected=False),
            lambda f: f["frame"].update(visibility="visible-blurred"),
            lambda f: f["frame"].update(host_monotonic_s=99.0),
            lambda f: f["frame"]["hands"].update(left=None),
            lambda f: f["frame"]["hands"]["left"]["pose"].update(emulated_position=True),
            lambda f: f["frame"]["hands"]["left"]["gamepad"].update(connected=False),
            lambda f: f["frame"]["hands"]["left"]["gamepad"].update(mapping=""),
            lambda f: f["frame"]["hands"]["left"]["pose"].update(position_m=[float("nan"), 0, 0]),
        ]
        for mutate in mutations:
            with self.subTest(mutation=mutate):
                mapper = self.mapper()
                self.engage(mapper)
                bad = snapshot(1, sequence=3)
                mutate(bad)
                out = self.update(mapper, bad)["left"]
                self.assertEqual(out["state"], "INPUT_INVALID")
                self.assertIsNone(out["target"])
                self.assertEqual(
                    self.update(mapper, snapshot(1, sequence=4))["left"]["state"],
                    "RELEASE_REQUIRED",
                )
                self.update(mapper, snapshot(0, sequence=5))
                self.assertTrue(
                    self.update(mapper, snapshot(1, sequence=6))["left"]["target_valid"]
                )

    def test_robot_errors_staleness_or_unknown_state_cancel(self):
        changes = [
            {"host_monotonic_s": 99.0},
            {"error_status": 1},
            {"error_status": None},
            {"robot_mode": 9},
            {"robot_mode": 7},
            {"valid": False},
            {"tool": None},
        ]
        for change in changes:
            with self.subTest(change=change):
                mapper = self.mapper()
                self.engage(mapper)
                robot = robots()
                robot["left"].update(change)
                self.assertEqual(
                    self.update(mapper, snapshot(1, sequence=3), robot)["left"]["state"],
                    "ROBOT_INVALID",
                )
                self.assertEqual(
                    self.update(mapper, snapshot(1, sequence=4))["left"]["state"],
                    "RELEASE_REQUIRED",
                )

    def test_origin_reset_invalidates_verified_mapping_and_preview_still_needs_release(self):
        mapper = self.mapper()
        self.engage(mapper)
        out = self.update(mapper, snapshot(1, sequence=3, epoch=1))["left"]
        self.assertEqual(out["state"], "NOT_CALIBRATED")
        self.assertIsNone(out["target"])
        mapper.set_calibration("left", calibration(epoch=1))
        self.assertEqual(
            self.update(mapper, snapshot(1, sequence=4, epoch=1))["left"]["state"],
            "RELEASE_REQUIRED",
        )
        self.update(mapper, snapshot(0, sequence=5, epoch=1))
        self.assertTrue(
            self.update(mapper, snapshot(1, sequence=6, epoch=1))["left"]["target_valid"]
        )
        preview = self.mapper(
            MappingConfig(allow_unverified_frame_preview=True), left=calibration(verified=False)
        )
        self.engage(preview)
        self.assertEqual(
            self.update(preview, snapshot(1, sequence=3, epoch=1))["left"]["state"],
            "RELEASE_REQUIRED",
        )

    def test_new_session_tool_change_and_out_of_order_frames_cannot_resume_held(self):
        mapper = self.mapper()
        self.engage(mapper)
        self.update(mapper, snapshot(1, sequence=5))
        out = self.update(mapper, snapshot(1, sequence=4))["left"]
        self.assertEqual(out["reason"], "OUT_OF_ORDER_QUEST_FRAME")
        self.assertEqual(
            self.update(mapper, snapshot(1, sequence=6))["left"]["state"], "RELEASE_REQUIRED"
        )
        mapper = self.mapper()
        self.engage(mapper)
        robot = robots()
        robot["left"]["tool"] = 1
        self.assertEqual(
            self.update(mapper, snapshot(1, sequence=3), robot)["left"]["state"], "RELEASE_REQUIRED"
        )
        new_session = snapshot(1, sequence=1)
        new_session["session"] = "other-session"
        self.assertEqual(self.update(mapper, new_session)["left"]["state"], "NOT_CALIBRATED")

    def test_repeated_snapshot_and_quaternion_sign_do_not_create_new_anchor(self):
        mapper = self.mapper()
        anchor = self.engage(mapper)["left"]
        frame = snapshot(1, sequence=3)
        frame["frame"]["hands"]["left"]["pose"]["quaternion_xyzw"] = [0, 0, 0, -1]
        first = self.update(mapper, frame)["left"]
        second = self.update(mapper, frame)["left"]
        self.assertEqual(first["generation"], anchor["generation"])
        self.assertEqual(first["target"], second["target"])
        self.assertEqual(first["target"]["quaternion_xyzw"], [0, 0, 0, 1])

    def test_invalid_calibration_and_future_timestamp_fail_closed(self):
        for matrix in (((-1, 0, 0), (0, 1, 0), (0, 0, 1)), ((2, 0, 0), (0, 1, 0), (0, 0, 1))):
            with self.assertRaises(ValueError):
                calibration(matrix)
        with self.assertRaises(ValueError):
            MappingConfig(engage_threshold=0.1, release_threshold=0.2)
        mapper = self.mapper()
        self.engage(mapper)
        self.assertEqual(
            self.update(mapper, snapshot(1, sequence=3, received=101))["left"]["state"],
            "INPUT_INVALID",
        )
        self.assertEqual(
            self.update(mapper, snapshot(1, sequence=4), now=float("nan"))["left"]["state"],
            "INPUT_INVALID",
        )


if __name__ == "__main__":
    unittest.main()
