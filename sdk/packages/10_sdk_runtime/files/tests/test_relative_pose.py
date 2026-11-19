"""Isolated synthetic mapping checks; these are not hardware motion evidence."""

import copy
import math
import unittest

from relative_pose import relative_pose_target, grip_direction_mapping, incremental_pose_target, anchored_position_target, anchored_pose_target
from kinematics_validation import pose_error
from tool_orientation_cone import project_target


def hand(position=(0, 0, 0), quaternion=(0, 0, 0, 1)):
    return {"position_m": list(position), "quaternion_xyzw": list(quaternion)}


class AnchoredPositionTests(unittest.TestCase):
    def test_persistent_orientation_has_no_engagement_jump_and_is_quaternion_sign_invariant(self):
        c = ((0, -1, 0), (1, 0, 0), (0, 0, 1))
        anchor = hand((.2, .9, -.3), (0, math.sin(.2), 0, math.cos(.2)))
        robot = [261, -81, 285, -180, 0, -90]
        target = anchored_pose_target(anchor, robot, anchor, c)
        self.assertLess(max(pose_error(robot, target)), 1e-7)
        changed_sign = copy.deepcopy(anchor)
        changed_sign["quaternion_xyzw"] = [-v for v in anchor["quaternion_xyzw"]]
        self.assertLess(max(pose_error(target, anchored_pose_target(anchor, robot, changed_sign, c))), 1e-7)

    def test_all_three_rotations_persist_and_translation_matches_existing_B(self):
        c = ((0, -1, 0), (1, 0, 0), (0, 0, 1))
        robot = [261, -81, 285, -180, 0, -90]
        for axis in range(3):
            quaternion = [0, 0, 0, math.cos(math.radians(9)/2)]
            quaternion[axis] = math.sin(math.radians(9)/2)
            current = hand((.01, .02, .03), quaternion)
            target = anchored_pose_target(hand(), robot, current, c)
            # Consuming the same current hand and changing measured pose used
            # to erase the rotation. The retained goal has no measured input.
            old = anchored_position_target(hand(), robot, current, [0,0,0,180,0,0], current, c)
            self.assertEqual(target[:3], old[:3])
            self.assertAlmostEqual(pose_error(robot, target)[1], 9, places=6)
            self.assertEqual(target, anchored_pose_target(hand(), robot, current, c))
            neutral = anchored_pose_target(hand(), robot, hand(current["position_m"]), c)
            self.assertEqual(target[:3], neutral[:3])

    def test_cone_projection_preserves_retained_translation_and_does_not_accumulate(self):
        robot = [261, -81, 285, -180, 0, -90]
        c = ((1,0,0),(0,1,0),(0,0,1))
        current = hand((.1, 0, 0), (math.sin(math.radians(70)/2),0,0,math.cos(math.radians(70)/2)))
        first, info = project_target(anchored_pose_target(hand(), robot, current, c))
        self.assertTrue(info["projected"])
        self.assertEqual(first[:3], [361, -81, 285])
        for _ in range(20):
            target, _ = project_target(anchored_pose_target(hand(), robot, current, c))
            self.assertEqual(target, first)

    def test_stationary_hand_retains_goal_despite_actual_robot_lag(self):
        c = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
        anchor, robot_anchor = hand(), [0]*6
        current = hand((.1, 0, 0))
        first = anchored_position_target(anchor, robot_anchor, anchor, [0]*6, current, c)
        second = anchored_position_target(anchor, robot_anchor, current, [60, 0, 0, 0, 0, 0], current, c)
        third = anchored_position_target(anchor, robot_anchor, current, [60, 0, 0, 0, 0, 0], hand((.12, 0, 0)), c)
        self.assertEqual(first[:3], [100, 0, 0])
        self.assertEqual(second[:3], [100, 0, 0])
        self.assertEqual(third[:3], [120, 0, 0])

    def test_fixed_world_translation_and_incremental_rotation_are_independent(self):
        c = ((0, -1, 0), (1, 0, 0), (0, 0, 1))
        previous = hand((.1, 0, 0))
        current = hand((.1, 0, 0), (0, 0, math.sin(math.radians(5)/2), math.cos(math.radians(5)/2)))
        target = anchored_position_target(hand(), [10, 20, 30, 0, 0, 80], previous,
                                          [17, 60, 32, 0, 0, 15], current, c)
        for got, expected in zip(target, [10, 120, 30, 0, 0, 20]):
            self.assertAlmostEqual(got, expected)

    def test_new_grip_reanchors_without_rest_displacement_jump(self):
        c = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
        rested = hand((1.2, 1.4, -.3))
        actual = [60, 20, 30, 40, 10, -20]
        target = anchored_position_target(rested, actual, rested, actual, rested, c)
        for got, expected in zip(target, actual):
            self.assertAlmostEqual(got, expected)


class IncrementalPoseTests(unittest.TestCase):
    def test_rebase_on_current_B_without_previous_goal_debt(self):
        c = grip_direction_mapping(hand(), [0] * 6)
        first = incremental_pose_target(hand(), [10, 20, 30, 0, 0, 0], hand((0.005, 0, 0)), c)
        self.assertEqual(first, [15, 20, 30, 0, 0, 0])
        second = incremental_pose_target(
            hand((0.005, 0, 0)), [12, 20, 30, 0, 0, 0], hand((0.005, 0.002, 0)), c
        )
        self.assertEqual(second, [12, 22, 30, 0, 0, 0])
        self.assertEqual(
            incremental_pose_target(hand(), [7, 8, 9, 10, 20, 30], hand(), c), [7, 8, 9, 10, 20, 30]
        )

    def test_spatial_relative_rotation_uses_fixed_C_and_current_robot_rotation(self):
        s, c0 = math.sin(math.radians(2.5)), math.cos(math.radians(2.5))
        sz, cz = math.sin(math.radians(15)), math.cos(math.radians(15))
        mapping = grip_direction_mapping(hand(), [0, 0, 0, 0, 90, 0])
        previous = hand(quaternion=(0, 0, sz, cz))
        current = hand((0.001, 0, 0), quaternion=(s * cz, -s * sz, c0 * sz, c0 * cz))
        target = incremental_pose_target(previous, [100, 200, 300, 20, 0, 0], current, mapping)
        for value, expected in zip(target, [100, 200, 299, 20, 0, -5]):
            self.assertAlmostEqual(value, expected, places=8)

    def test_mapping_cannot_sneak_in_scale_or_reflection(self):
        for invalid in [((2, 0, 0), (0, 1, 0), (0, 0, 1)), ((-1, 0, 0), (0, 1, 0), (0, 0, 1))]:
            with self.assertRaises(ValueError):
                incremental_pose_target(hand(), [0] * 6, hand((0.001, 0, 0)), invalid)


class RelativePoseTests(unittest.TestCase):
    def assert_vector_close(self, actual, expected):
        self.assertEqual(len(actual), len(expected))
        for value, reference in zip(actual, expected):
            self.assertAlmostEqual(value, reference, places=9)

    def test_engagement_has_no_target_jump_and_keeps_nearest_euler_branch(self):
        anchor = hand((0.4, 1.2, -0.7), (math.sqrt(0.5), 0, 0, math.sqrt(0.5)))
        robot = [300, -20, 150, 181, 100, -270]
        before = copy.deepcopy((anchor, robot))
        self.assert_vector_close(relative_pose_target(anchor, robot, anchor), robot)
        self.assertEqual((anchor, robot), before)

    def test_one_centimetre_is_ten_millimetres_on_all_nonidentity_mapped_axes(self):
        s = math.sqrt(0.5)
        anchor = hand((0.4, 1.2, -0.7), (s, 0, 0, s))
        robot = [300, -20, 150, 0, 0, 90]
        # Rmap = Rz(90) Rx(-90): hand X -> tool +Y, Y -> -Z, Z -> -X.
        for axis, expected_delta in enumerate(((0, 10, 0), (0, 0, -10), (-10, 0, 0))):
            with self.subTest(axis=axis):
                current = copy.deepcopy(anchor)
                current["position_m"][axis] += 0.01
                target = relative_pose_target(anchor, robot, current)
                delta = [target[i] - robot[i] for i in range(3)]
                self.assert_vector_close(delta, expected_delta)
                self.assertAlmostEqual(math.sqrt(sum(x * x for x in delta)), 10)
                self.assert_vector_close(target[3:], robot[3:])

    def test_noncommuting_relative_rotation_and_quaternion_sign_are_preserved(self):
        s = math.sqrt(0.5)
        robot = [300, 0, 200, 0, 0, 90]
        # Rz(90) Rx(-90) Ry(90) = Rx(-90); changing multiplication order differs.
        for anchor_sign in (-1, 1):
            for current_sign in (-1, 1):
                with self.subTest(anchor_sign=anchor_sign, current_sign=current_sign):
                    anchor = hand(quaternion=(anchor_sign * s, 0, 0, anchor_sign * s))
                    current = hand(quaternion=(0, current_sign * s, 0, current_sign * s))
                    self.assert_vector_close(
                        relative_pose_target(anchor, robot, current), [300, 0, 200, -90, 0, 0]
                    )

    def test_wrap_and_both_gimbal_singularities_choose_small_anchor_changes(self):
        angle = math.radians(3) / 2
        current = hand(quaternion=(0, 0, math.sin(angle), math.cos(angle)))
        self.assert_vector_close(
            relative_pose_target(hand(), [0, 0, 0, 0, 0, 179], current),
            [0, 0, 0, 0, 0, 182],
        )
        angle = math.radians(13) / 2
        current = hand(quaternion=(math.sin(angle), 0, 0, math.cos(angle)))
        for sign in (-1, 1):
            with self.subTest(pitch_sign=sign):
                self.assert_vector_close(
                    relative_pose_target(hand(), [0, 0, 0, 35, sign * 90, -40], current),
                    [0, 0, 0, 41.5, sign * 90, -40 - sign * 6.5],
                )

    def test_invalid_measurements_rejected_and_roundoff_only_normalized(self):
        invalid = (
            hand(quaternion=(0, 0, 0, 0)),
            hand(quaternion=(0, 0, 0, 1.01)),
            hand(position=(float("nan"), 0, 0)),
            hand(position=(True, 0, 0)),
            {"position_m": [0, 0, 0]},
        )
        for pose in invalid:
            with self.subTest(pose=pose):
                with self.assertRaises(ValueError):
                    relative_pose_target(hand(), [0] * 6, pose)
                with self.assertRaises(ValueError):
                    relative_pose_target(pose, [0] * 6, hand())
        with self.assertRaises(ValueError):
            relative_pose_target(hand(), [0, 0, 0, 0, float("inf"), 0], hand())
        self.assert_vector_close(
            relative_pose_target(hand(), [0] * 6, hand(quaternion=(0, 0, 0, 1 + 1e-10))),
            [0] * 6,
        )


if __name__ == "__main__":
    unittest.main()
