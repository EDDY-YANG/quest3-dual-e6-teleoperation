"""Exercise real native RangedIK with saved FK/log data. Never contacts hardware."""

import argparse
import hashlib
import json
from collections import Counter
from datetime import datetime
from pathlib import Path

import numpy as np

from controller_kinematics import identify
from kinematics_validation import pose_error
from ranged_ik_adapter import RangedIKAdapter
from ranged_ik_profile import RangedIKProfile

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--profile", type=Path, default=ROOT / "profiles/rangedik_unloaded_wide.json"
    )
    parser.add_argument("--episode", type=Path, action="append", default=[])
    parser.add_argument("--fk-samples", type=Path, required=True,
                        help="Your own controller FK calibration and held-out samples JSON")
    args = parser.parse_args()
    data = json.loads(args.fk_samples.read_text(encoding="utf-8-sig"))
    model = identify(data["samples"])
    profile = RangedIKProfile.load(args.profile)
    stamp = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    output = ROOT / "runtime" / (stamp + "-rangedik-offline-check")
    output.mkdir(parents=True, exist_ok=False)
    seed = data["validation"][0]["q_deg"]
    adapter = RangedIKAdapter(model, profile, seed, output)
    report = {
        "scope": "REAL_NATIVE_SOLVER_OFFLINE_ONLY_NO_NETWORK_OR_ROBOT_MOTION",
        "profile": profile.snapshot(),
        "native": adapter.snapshot(),
        "fk_checks": [],
        "generated_goal_checks": [],
        "replayed_windows": [],
    }
    try:
        for sample in data["validation"]:
            error = pose_error(sample["pose_mm_deg"], adapter.native_fk(sample["q_deg"]))
            report["fk_checks"].append(
                {
                    "q_deg": sample["q_deg"],
                    "position_error_mm": error[0],
                    "rotation_error_deg": error[1],
                }
            )
            if error[0] > 0.05 or error[1] > 0.05:
                raise RuntimeError("Native model disagrees with saved controller FK")
        for axis in range(6):
            for sign in (-1, 1):
                adapter.reset(seed)
                goal = model.fk(seed)
                goal[axis] += sign * (20 if axis < 3 else 15)
                result = adapter.propose(goal, seed)
                report["generated_goal_checks"].append(
                    {"axis": axis, "sign": sign, "target_pose_mm_deg": goal, "result": result}
                )
                if result["status"] not in ("WITHIN_RANGE", "APPROACHING"):
                    raise RuntimeError("No useful progress on a local unloaded commissioning goal")
        for episode in args.episode:
            source = episode if episode.is_file() else episode / "events.jsonl"
            original = source.read_bytes()
            records = [json.loads(line) for line in original.decode("utf-8").splitlines() if line]
            events = [
                r for r in records if not r.get("pre_roll") and r["kind"] == "controller_event"
            ]
            initialized = False
            for row in events:
                event = row["data"]
                if event.get("kind") == "GRIP_ENGAGED":
                    initialized = False
                if event.get("kind") != "IK_SOLUTION":
                    continue
                q = event["measured_seed_deg"]
                if not initialized:
                    adapter.reset(q)
                    initialized = True
                result = adapter.propose(event["requested_pose_mm_deg"], q)
                report["replayed_windows"].append(
                    {
                        "episode": source.parent.name,
                        "pc_time": row["pc_time"],
                        "measured_seed_deg": q,
                        "target_pose_mm_deg": event["requested_pose_mm_deg"],
                        "original_solver_success": event["local_result"]["success"],
                        "result": result,
                        "history_commit": "OFFLINE_CANDIDATE_REPLAY_NOT_PHYSICAL_ACK",
                    }
                )
                adapter.commit(result)
            if hashlib.sha256(source.read_bytes()).digest() != hashlib.sha256(original).digest():
                raise RuntimeError("Source log changed during replay")
    finally:
        report["statuses"] = dict(
            Counter(r["result"]["status"] for r in report["replayed_windows"])
        )
        timings = [
            r["result"]["solve_ms"]
            for r in report["generated_goal_checks"] + report["replayed_windows"]
        ]
        report["solver_latency_ms"] = (
            dict(
                zip(
                    ("min", "median", "p95", "max"),
                    np.percentile(timings, [0, 50, 95, 100]).tolist(),
                )
            )
            if timings
            else {}
        )
        (output / "report.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8"
        )
        adapter.close()
    print(
        json.dumps(
            {
                "report": str(output / "report.json"),
                "fk_checks": len(report["fk_checks"]),
                "generated_goals": len(report["generated_goal_checks"]),
                "replayed_windows": len(report["replayed_windows"]),
                "statuses": report["statuses"],
                "solver_latency_ms": report["solver_latency_ms"],
            },
            ensure_ascii=True,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
