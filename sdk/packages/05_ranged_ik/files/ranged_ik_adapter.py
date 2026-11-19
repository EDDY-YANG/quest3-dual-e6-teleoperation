"""Per-arm RangedIK adapter preserving the application's mm/degree contract."""

import math
import time
from datetime import datetime
from pathlib import Path
from uuid import uuid4

import numpy as np

from controller_kinematics import pose_matrix, rotation_log
from e6_feedback import euler_xyz_deg_to_xyzw
from kinematics_validation import pose_error
from ranged_ik_backend import NativeRangedIK, RangedIKBackendError
from ranged_ik_model import write_model_bundle
from ranged_ik_profile import vector
from relative_pose import _hand_pose, _near_euler
from tool_orientation_cone import in_cone, project_target, joint_path_check

ORIENTATION_STEP_FRACTIONS = (1.0, 0.5, 0.25, 0.125)
ORIENTATION_RETRY_BUDGET_S = 0.08


def orientation_intermediate(target, measured, fraction):
    """A nearer SO(3) goal; desired XYZ and the caller's full goal are retained."""
    goal, actual = pose_matrix(target), pose_matrix(measured)
    vector = rotation_log(goal[:3, :3] @ actual[:3, :3].T)
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        return list(target)
    x, y, z = vector / angle
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    step = angle*fraction
    rotation = (np.eye(3)+math.sin(step)*skew+(1-math.cos(step))*(skew@skew)) @ actual[:3, :3]
    pose = list(target[:3]) + _near_euler(rotation, target[3:])
    return project_target(pose)[0]


class RangedIKAdapter:
    def __init__(
        self,
        model,
        profile,
        starting_q_deg,
        runtime_dir,
        arm_id="left",
        backend_factory=NativeRangedIK,
    ):
        self.model, self.profile, self.arm_id = model, profile, arm_id
        self.generation = 0
        self._guides = {}
        self._committed_seed = None
        self._check_absolute(starting_q_deg)
        directory = Path(runtime_dir) / (
            datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + f"-rangedik-{arm_id}-" + uuid4().hex[:8]
        )
        self.bundle = write_model_bundle(
            model, profile.joint_limits_deg, starting_q_deg, directory, arm_id
        )
        self.backend = backend_factory(
            profile.library_path, self.bundle["settings"], profile.objective_weights,
            orientation_error_metric=profile.orientation_error_metric,
        )
        try:
            if profile.seed_strategy == "VALIDATED_MULTI_START":
                if not self.backend.has_preview_selection:
                    raise RangedIKBackendError("RANGEDIK_MULTI_START_ABI_MISSING")
                # Numerical initializers only. Their outputs are never executable
                # references: every seed is refined by the original geodesic
                # objective with the original measured-centred bounds/history.
                self._guides["component_guide"] = backend_factory(
                    profile.library_path, self.bundle["settings"], profile.objective_weights,
                    orientation_error_metric="ROTATION_VECTOR_COMPONENTS")
                self._guides["smooth_guide"] = backend_factory(
                    profile.library_path, self.bundle["settings"],
                    {**profile.objective_weights, "rotation": (0.,0.,0.)},
                    orientation_error_metric="GEODESIC_ANGLE")
            errors = []
            queries = [list(starting_q_deg)]
            for i in range(6):
                q = list(starting_q_deg)
                q[i] += 0.5 if q[i] + 0.5 <= profile.joint_limits_deg[i][1] else -0.5
                queries.append(q)
            for q in queries:
                error = pose_error(self.model.fk(q), self.native_fk(q))
                if error[0] > 0.05 or error[1] > 0.05:
                    raise ValueError("RANGEDIK_EXPORTED_MODEL_FK_MISMATCH")
                errors.append(error)
            self.validation = {
                "status": "VALIDATED_AGAINST_COMMISSIONED_MODEL",
                "samples": len(errors),
                "max_position_error_mm": max(e[0] for e in errors),
                "max_rotation_error_deg": max(e[1] for e in errors),
                "physical_acceptance": "NOT_PERFORMED",
            }
            self.reset(starting_q_deg)
        except Exception:
            for guide in self._guides.values():
                guide.close()
            self.backend.close()
            raise

    def _check_absolute(self, q_deg):
        q = vector(q_deg, 6, "joint angles")
        if any(
            v < lo - 1e-7 or v > hi + 1e-7 for v, (lo, hi) in zip(q, self.profile.joint_limits_deg)
        ):
            raise ValueError("RANGEDIK_ABSOLUTE_JOINT_LIMIT")
        return q

    def reset(self, measured_q_deg):
        q = self._check_absolute(measured_q_deg)
        self.backend.reset(q)
        self._committed_seed = list(q)
        for guide in self._guides.values():
            guide.reset(q)
        self.generation += 1

    def validate_joints(self, q_deg):
        return self._check_absolute(q_deg)

    def native_fk(self, q_deg):
        position, quaternion = self.backend.fk(q_deg)
        _, rotation = _hand_pose({"position_m": [0, 0, 0], "quaternion_xyzw": quaternion})
        return position + _near_euler(rotation, [0, 0, 0])

    def task_error(self, requested_pose, candidate_pose):
        goal, candidate = pose_matrix(requested_pose), pose_matrix(candidate_pose)
        ep = goal[:3, :3].T @ (candidate[:3, 3] - goal[:3, 3])
        er = np.degrees(rotation_log(goal[:3, :3].T @ candidate[:3, :3]))
        scaled = np.r_[
            ep / self.profile.position_tolerance_mm, er / self.profile.rotation_tolerance_deg
        ]
        angle = float(np.linalg.norm(er))
        if self.profile.orientation_error_metric == "GEODESIC_ANGLE":
            orientation_within = angle <= self.profile.orientation_tolerance_deg*(1+1e-6)
        else:
            orientation_within = bool(np.max(np.abs(scaled[3:])) <= 1+1e-6)
        return {
            "position_error_goal_frame_mm": ep.tolist(),
            "rotation_vector_error_goal_frame_deg": er.tolist(),
            "rotation_error_angle_deg": angle,
            "orientation_error_metric": self.profile.orientation_error_metric,
            "orientation_tolerance_deg": self.profile.orientation_tolerance_deg,
            "within_orientation_range": orientation_within,
            "within_task_range": bool(np.max(np.abs(scaled[:3])) <= 1.0 + 1e-6 and orientation_within),
            "normalized_error_cost": float(scaled @ scaled),
        }

    def evaluate(self, requested_pose, candidate_q, measured_q):
        q, measured = self._check_absolute(candidate_q), self._check_absolute(measured_q)
        if any(
            abs(a - b) > radius + 1e-6
            for a, b, radius in zip(q, measured, self.profile.joint_search_radius_deg)
        ):
            raise ValueError("RANGEDIK_CANDIDATE_SEARCH_BOUND_EXCEEDED")
        native_pose = self.native_fk(q)
        error = pose_error(self.model.fk(q), native_pose)
        if error[0] > 0.05 or error[1] > 0.05:
            raise ValueError("RANGEDIK_NATIVE_FK_MISMATCH")
        quality = self.task_error(requested_pose, native_pose)
        before = self.task_error(requested_pose, self.model.fk(measured))
        progress = before["normalized_error_cost"] - quality["normalized_error_cost"]
        if quality["within_task_range"]:
            status = "WITHIN_RANGE"
        elif progress > max(1e-9, 1e-4 * before["normalized_error_cost"]):
            status = "APPROACHING"
        else:
            status = "HOLD_NO_PROGRESS"
        selected_q = list(measured if status == "HOLD_NO_PROGRESS" else q)
        return {
            "success": status != "HOLD_NO_PROGRESS",
            "status": status,
            "reason": "RANGEDIK_" + status,
            "q_deg": selected_q,
            "raw_candidate_q_deg": list(q),
            "candidate_pose_mm_deg": native_pose,
            "selected_pose_mm_deg": self.model.fk(selected_q),
            "task_error": quality,
            "center_progress_cost": progress,
            "generation": self.generation,
        }

    def propose(self, target_pose_mm_deg, measured_q_deg, *, command_delta_limit_deg=None):
        target = vector(target_pose_mm_deg, 6, "target pose")
        measured = self._check_absolute(measured_q_deg)
        radii = self.profile.joint_search_radius_deg
        bounds = [
            (max(lo, q - r), min(hi, q + r))
            for (lo, hi), q, r in zip(
                self.profile.joint_limits_deg, measured, radii
            )
        ]
        if self.profile.seed_strategy == "VALIDATED_MULTI_START" and command_delta_limit_deg is not None:
            if (type(command_delta_limit_deg) not in (int,float)
                    or not math.isfinite(command_delta_limit_deg) or command_delta_limit_deg <= 0):
                raise ValueError("Invalid command delta admissibility limit")
            return self._propose_multistart(target, measured, bounds, command_delta_limit_deg)
        started = time.perf_counter()
        continuation = (self.profile.orientation_target_policy == "GRIP_ANCHORED_WORLD"
                        and command_delta_limit_deg is not None)
        if continuation and (type(command_delta_limit_deg) not in (int, float)
                             or not math.isfinite(command_delta_limit_deg) or command_delta_limit_deg <= 0):
            raise ValueError("Invalid command delta admissibility limit")
        actual = self.model.fk(measured)
        attempts, admissible = [], False
        for fraction in (ORIENTATION_STEP_FRACTIONS if continuation else (1.0,)):
            if attempts and time.perf_counter()-started >= ORIENTATION_RETRY_BUDGET_S:
                break
            solver_goal = orientation_intermediate(target, actual, fraction) if fraction < 1 else target
            candidate = self.backend.propose(
                solver_goal[:3], euler_xyz_deg_to_xyzw(solver_goal[3:]),
                self.profile.position_tolerance_mm + self.profile.rotation_tolerance_deg,
                measured, bounds,
            )
            # Always judge progress against the FULL retained pose.
            result = self.evaluate(target, candidate, measured)
            if not continuation:
                break
            rejection = []
            if max(abs(a-b) for a,b in zip(result["q_deg"], measured)) > command_delta_limit_deg+1e-6:
                rejection.append("JOINT_SEND_BOUND")
            if not in_cone(result["selected_pose_mm_deg"]):
                rejection.append("CANDIDATE_CONE_45")
            path = joint_path_check(self.model, measured, result["q_deg"])
            if not path["accepted"]:
                rejection.append(path["reason"])
            if result["status"] == "HOLD_NO_PROGRESS":
                rejection.append("NO_FULL_GOAL_PROGRESS")
            attempts.append({"orientation_fraction": fraction, "solver_goal_pose_mm_deg": list(solver_goal),
                             "full_goal_task_error": result["task_error"], "rejections": rejection,
                             "solve_ms": time.perf_counter()*1000-started*1000})
            if not rejection:
                admissible = True
                break
        if continuation:
            result["continuation"] = {"attempts": attempts, "output_admissible": admissible,
                                      "position_goal_unchanged": True, "full_target_retained": True,
                                      "retry_budget_ms": ORIENTATION_RETRY_BUDGET_S*1000}
            if not admissible:
                # The caller must not send or commit this rejected preview.
                result["success"] = False
                result["status"] = "REJECTED_NO_ADMISSIBLE_STEP"
                result["reason"] = "RANGEDIK_NO_ADMISSIBLE_ORIENTATION_STEP"
        result["search_bounds_deg"] = [list(pair) for pair in bounds]
        result["native_solver"] = dict(getattr(self.backend, "last_stats", {}))
        result["solve_ms"] = (time.perf_counter() - started) * 1000
        return result

    def _propose_multistart(self, target, measured, bounds, command_limit):
        started = time.perf_counter()
        deadline = started + ORIENTATION_RETRY_BUDGET_S
        actual = self.model.fk(measured)
        attempts, guide_calls = [], []
        best, last, send_bound_seed = None, None, None
        self.backend.begin_preview_batch()

        def remaining_ms():
            return min(80., max(0., (deadline-time.perf_counter())*1000))

        def score(candidate):
            quality = candidate["task_error"]
            return (not quality["within_task_range"], quality["normalized_error_cost"])

        def solve(seed, goal, fraction, source, search_bounds=None):
            nonlocal best, last, send_bound_seed
            budget = remaining_ms()
            if budget < 1:
                return
            # This clamps a NUMERICAL INITIAL GUESS only; no command is clipped.
            working_bounds = bounds if search_bounds is None else search_bounds
            initial = [max(lo,min(hi,float(v))) for v,(lo,hi) in zip(seed,working_bounds)]
            try:
                q = self.backend.propose(
                    goal[:3], euler_xyz_deg_to_xyzw(goal[3:]),
                    self.profile.position_tolerance_mm+self.profile.rotation_tolerance_deg,
                    initial, working_bounds, time_budget_ms=budget)
            except RangedIKBackendError as exc:
                if best is None:
                    raise
                attempts.append({"orientation_fraction":fraction,"seed_source":source,
                    "initial_guess_deg":initial,"solver_goal_pose_mm_deg":list(goal),
                    "joint_search_bounds_deg":[list(pair) for pair in working_bounds],
                    "rejections":["NATIVE_REFINEMENT_ERROR"],"error":str(exc),
                    "solve_ms":(time.perf_counter()-started)*1000})
                return
            result = self.evaluate(target,q,measured)
            result["native_preview_id"] = self.backend.last_preview_id
            result["native_solver"] = dict(self.backend.last_stats)
            result["seed_source"] = source
            result["orientation_fraction"] = fraction
            rejection = []
            if max(abs(a-b) for a,b in zip(result["q_deg"],measured)) > command_limit+1e-6:
                rejection.append("JOINT_SEND_BOUND")
            if not in_cone(result["selected_pose_mm_deg"]):
                rejection.append("CANDIDATE_CONE_45")
            path = joint_path_check(self.model,measured,result["q_deg"])
            if not path["accepted"]:
                rejection.append(path["reason"])
            if result["status"] == "HOLD_NO_PROGRESS":
                rejection.append("NO_FULL_GOAL_PROGRESS")
            attempts.append({"orientation_fraction":fraction,"seed_source":source,
                "initial_guess_deg":initial,"solver_goal_pose_mm_deg":list(goal),
                "joint_search_bounds_deg":[list(pair) for pair in working_bounds],
                "full_goal_task_error":result["task_error"],"rejections":rejection,
                "native_preview_id":result["native_preview_id"],"native_solver":result["native_solver"],
                "solve_ms":(time.perf_counter()-started)*1000})
            last = result
            if fraction == 1 and "JOINT_SEND_BOUND" in rejection and (
                    send_bound_seed is None or score(result)<score(send_bound_seed)):
                send_bound_seed = result
            if not rejection and (best is None or score(result) < score(best)):
                best = result

        for fraction in ORIENTATION_STEP_FRACTIONS:
            if remaining_ms() < 1:
                break
            goal = list(target) if fraction == 1 else orientation_intermediate(target,actual,fraction)
            solve(measured,goal,fraction,"measured")
            if best is not None:
                break
        # Secure the existing v6 staged reference first. Additional work may
        # improve it, but budget exhaustion cannot discard this valid fallback.
        baseline = best
        admission_bounds = [(max(lo,q-command_limit),min(hi,q+command_limit))
                            for (lo,hi),q in zip(bounds,measured)]
        if send_bound_seed is not None and (best is None or not best["task_error"]["within_task_range"]):
            # Main IK search remains +/-60deg. This additional native subproblem
            # incorporates the EXISTING +/-50deg send gate; it is not joint clipping.
            solve(send_bound_seed["q_deg"],list(target),1.0,"send_bound_refinement",admission_bounds)
        if best is None or not best["task_error"]["within_task_range"]:
            goal, fraction = list(target), 1.0
            if self._committed_seed is not None and max(abs(a-b) for a,b in zip(self._committed_seed,measured)) > 1e-6:
                solve(self._committed_seed,goal,fraction,"last_committed")
            for name, guide in self._guides.items():
                if best is not None and best["task_error"]["within_task_range"]:
                    break
                budget = remaining_ms()
                if budget < 2:
                    break
                guide.reset(measured)
                guide_started = time.perf_counter()
                try:
                    initial = guide.propose(
                        goal[:3],euler_xyz_deg_to_xyzw(goal[3:]),
                        self.profile.position_tolerance_mm+self.profile.rotation_tolerance_deg,
                        measured,bounds,time_budget_ms=budget/2)
                    guide_calls.append({"source":name,"orientation_fraction":fraction,
                        "initial_guess_deg":initial,"elapsed_ms":(time.perf_counter()-guide_started)*1000})
                except RangedIKBackendError as exc:
                    guide_calls.append({"source":name,"error":str(exc)})
                    continue
                solve(initial,goal,fraction,name+"_refined")
            if best is not None and not best["task_error"]["within_task_range"] and remaining_ms() >= 2:
                # Re-enter the gentler component landscape from the best
                # admissible strict-geodesic result, then refine geodesically.
                guide = self._guides["component_guide"]
                guide.reset(measured)
                try:
                    initial = guide.propose(
                        target[:3],euler_xyz_deg_to_xyzw(target[3:]),
                        self.profile.position_tolerance_mm+self.profile.rotation_tolerance_deg,
                        best["q_deg"],bounds,time_budget_ms=remaining_ms()/2)
                    guide_calls.append({"source":"component_from_best","initial_guess_deg":initial})
                    solve(initial,list(target),1.0,"component_from_best_refined")
                except RangedIKBackendError as exc:
                    guide_calls.append({"source":"component_from_best","error":str(exc)})
        result = best if best is not None else last
        if result is None:
            raise RangedIKBackendError("RANGEDIK_NO_PREVIEW_WITHIN_TIME_BUDGET")
        if best is not None:
            self.backend.select_preview(best["native_preview_id"])
        else:
            result["success"] = False
            result["status"] = "REJECTED_NO_ADMISSIBLE_STEP"
            result["reason"] = "RANGEDIK_NO_ADMISSIBLE_ORIENTATION_STEP"
        result["continuation"] = {
            "attempts":attempts,"guide_calls":guide_calls,"output_admissible":best is not None,
            "position_goal_unchanged":True,"full_target_retained":True,
            "retry_budget_ms":ORIENTATION_RETRY_BUDGET_S*1000,
            "selected_preview_id":best["native_preview_id"] if best else None,
            "selected_seed_source":best["seed_source"] if best else None,
            "selected_orientation_fraction":best["orientation_fraction"] if best else None,
            "measured_joint_bounds_deg":[list(pair) for pair in bounds],
            "admission_refinement_bounds_deg":[list(pair) for pair in admission_bounds],
            "time_budget_exhausted":remaining_ms()<1,
            "baseline_admissible":baseline is not None,
            "baseline_preview_id":baseline["native_preview_id"] if baseline else None,
            "baseline_task_error":baseline["task_error"] if baseline else None,
        }
        result["search_bounds_deg"] = [list(pair) for pair in bounds]
        result["solve_ms"] = (time.perf_counter()-started)*1000
        return result

    def commit(self, result):
        if (result.get("continuation") or {}).get("output_admissible") is False:
            raise ValueError("RANGEDIK_REJECTED_PROPOSAL_CANNOT_COMMIT")
        if result.get("generation") != self.generation:
            raise ValueError("RANGEDIK_STALE_PROPOSAL")
        if self.profile.seed_strategy == "VALIDATED_MULTI_START" and (
                result.get("native_preview_id") is None or result["native_preview_id"] != self.backend.last_preview_id):
            raise ValueError("RANGEDIK_STALE_PREVIEW_SELECTION")
        if result["status"] != "HOLD_NO_PROGRESS":
            self.backend.commit(self._check_absolute(result["q_deg"]))
            self._committed_seed = list(result["q_deg"])
        self.generation += 1

    def snapshot(self):
        return {
            **self.profile.snapshot(),
            "arm_id": self.arm_id,
            "validation": self.validation,
            "native_library_sha256": getattr(self.backend, "library_sha256", None),
            "upstream_commit": getattr(self.backend, "upstream_commit", None),
            "adapter": "CALLER_BUFFER_PREVIEW_COMMIT_WITH_CONFIGURED_CENTER_OBJECTIVES",
            "orientation_center_weight": getattr(self.backend, "orientation_center_weight", 0.0),
            "native_orientation_error_metric": self.backend.orientation_error_metric,
            "native_preview_selection": self.backend.has_preview_selection,
            "seed_search": {"strategy":self.profile.seed_strategy,"budget_ms":ORIENTATION_RETRY_BUDGET_S*1000,
                            "guide_outputs_executable":False,"history_changes_only_after_ack_commit":True},
            "orientation_step_fractions": list(ORIENTATION_STEP_FRACTIONS) if self.profile.orientation_target_policy == "GRIP_ANCHORED_WORLD" else None,
            "orientation_retry_budget_ms": ORIENTATION_RETRY_BUDGET_S*1000,
        }

    def close(self):
        for guide in self._guides.values():
            guide.close()
        self.backend.close()
