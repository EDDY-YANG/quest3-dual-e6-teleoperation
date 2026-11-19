"""Offline audit of saved measured inputs, IK replies and motion/stop evidence.

Never connects to a robot or service. Raw events.jsonl files remain unchanged.
"""

import argparse
import hashlib
import json
import math
import re
from datetime import datetime, timedelta, timezone
from pathlib import Path

from e6_feedback import euler_xyz_deg_to_xyzw
from relative_pose import relative_pose_target
from teleop_recording import analyze_log


def angle_between(first, second):
    dot = abs(sum(x * y for x, y in zip(first, second)))
    return math.degrees(2 * math.acos(min(1.0, dot)))


def audit(folder):
    source = folder / "events.jsonl"
    rows = [json.loads(line) for line in source.read_text(encoding="utf-8").splitlines() if line]
    active = [row for row in rows if not row.get("pre_roll")]
    anchors = [
        r["data"]
        for r in active
        if r["kind"] == "controller_event" and r["data"].get("kind") == "GRIP_ENGAGED"
    ]
    feedback = [r for r in active if r["kind"] == "robot_feedback" and r["data"].get("valid")]
    frames = [r for r in active if r["kind"] == "quest_frame"]
    servo = [
        r for r in active if r["kind"] == "command_return" and r["data"].get("method") == "ServoJ"
    ]
    stops = [
        r["data"]["evidence"]
        for r in active
        if r["kind"] == "controller_event" and r["data"].get("kind") == "STOP_RESULT"
    ]
    result = {
        "episode": folder.name,
        "source": str(source),
        "raw_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "servo_ack_count": sum(r["data"].get("reply_error_id") == 0 for r in servo),
        "stop_reasons": [s.get("reason") for s in stops],
        "stops_feedback_confirmed": all(s.get("feedback_confirmed") for s in stops),
        "robot_samples": len(feedback),
        "accepted_quest_frames": len(frames),
        "ik": [],
    }
    if feedback:
        points = [r["data"]["position_mm"] for r in feedback if r["data"].get("position_mm")]
        result.update(
            first_xyz_mm=points[0],
            last_xyz_mm=points[-1],
            observed_endpoint_change_mm=math.dist(points[0], points[-1]),
        )
    for row in active:
        data = row.get("data") or {}
        if row["kind"] != "command_return" or data.get("method") != "InverseKin":
            continue
        match = re.fullmatch(r"\s*0,\{([^{}]*)\},InverseKin\([^;]*\);\s*", data["raw_reply"])
        if not match:
            result["ik"].append(
                {
                    "at": row["pc_time"],
                    "raw_reply": data["raw_reply"],
                    "parse": "NOT_SUCCESSFUL_SIX_ANGLE_REPLY",
                }
            )
            continue
        solved = [float(x) for x in match[1].split(",")]
        seed = [float(x) for x in data["kwargs"]["JointNear"].strip("{}").split(",")]
        delta = [q - near for q, near in zip(solved, seed)]
        if len(delta) != 6:
            raise ValueError("IK reply or seed was not six-dimensional")
        item = {
            "at": row["pc_time"],
            "target_pose_mm_deg": data["args"],
            "ik_joint_deg": solved,
            "measured_seed_joint_deg": seed,
            "delta_from_measured_seed_deg": delta,
            "max_delta_from_seed_deg": max(map(abs, delta)),
            "largest_delta_joint": 1 + max(range(6), key=lambda k: abs(delta[k])),
            "wrist_joint5_seed_deg": seed[4],
            "raw_reply": data["raw_reply"],
        }
        # Search actual saved anchors/frames; no generated pose is inserted into live input.
        best = None
        for anchor in anchors:
            robot_anchor = anchor["robot_pose_mm_deg"]
            for frame in frames:
                lag = (row["pc_perf_ns"] - frame["pc_perf_ns"]) / 1e6
                if not 0 <= lag <= 300:
                    continue
                hand = (frame["data"].get("hands") or {}).get("left") or {}
                pose = hand.get("pose")
                if not pose:
                    continue
                mapped = relative_pose_target(anchor["hand_pose"], robot_anchor, pose)
                error = max(abs(a - b) for a, b in zip(mapped, data["args"]))
                if best is None or error < best[0]:
                    best = (error, anchor, frame, pose, lag)
        if best:
            error, anchor, frame, pose, lag = best
            robot_anchor = anchor["robot_pose_mm_deg"]
            item.update(
                mapping_replay_max_numeric_difference=error,
                matched_input_time=frame["pc_time"],
                receive_to_ik_reply_ms=lag,
                tracked=frame["data"]["hands"]["left"].get("tracking_valid"),
                visibility=frame["data"].get("visibility"),
                hand_displacement_mm=1000
                * math.dist(pose["position_m"], anchor["hand_pose"]["position_m"]),
                target_displacement_from_grip_mm=math.dist(data["args"][:3], robot_anchor[:3]),
                target_rotation_from_grip_deg=angle_between(
                    euler_xyz_deg_to_xyzw(robot_anchor[3:]), euler_xyz_deg_to_xyzw(data["args"][3:])
                ),
            )
        result["ik"].append(item)
    return result


def plot(report, path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties

    font = FontProperties(fname="C:/Windows/Fonts/msyh.ttc")
    points = [
        x for episode in report["episodes"] for x in episode["ik"] if "max_delta_from_seed_deg" in x
    ]
    fig, ax = plt.subplots(figsize=(12.8, 6.2))
    values = [p["max_delta_from_seed_deg"] for p in points]
    bars = ax.bar(
        range(len(points)), values, color=["#c84c50" if v > 10 else "#288968" for v in values]
    )
    ax.axhline(10, color="#d17f0d", ls="--", label="10° command-delta threshold")
    ax.set_xticks(range(len(points)), [p["at"][11:23] for p in points], rotation=30, ha="right")
    ax.set_ylabel("IK目标相对实测就近参考的最大关节变化 / °", fontproperties=font)
    ax.set_title("真实记录：小幅手柄动作可对应很大的IK关节变化", fontproperties=font, fontsize=16)
    for bar, point in zip(bars, points):
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            max(bar.get_height() + 2, 16),
            f"J{point['largest_delta_joint']} {point['max_delta_from_seed_deg']:.1f}°\n{point.get('hand_displacement_mm', 0):.2f} mm / {point.get('target_rotation_from_grip_deg', 0):.2f}°",
            ha="center",
            fontsize=9,
            bbox={"facecolor": "white", "alpha": 0.9, "edgecolor": "none", "pad": 1},
        )
    ax.set_ylim(0, max(values) * 1.22)
    ax.grid(axis="y", alpha=0.2)
    ax.legend(loc="upper left")
    fig.text(
        0.04,
        0.015,
        "绿色：该IK差值小于10°，记录中有对应ServoJ ACK；不表示精确到位。红色：新目标被10°闸拒发。",
        fontproperties=font,
        fontsize=10,
    )
    fig.tight_layout(rect=(0, 0.07, 1, 1))
    fig.savefig(path, dpi=170)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--prefix", default="2026-11-04-18-4")
    parser.add_argument("--plot", action="store_true")
    parser.add_argument("--refresh-summaries", action="store_true")
    args = parser.parse_args()
    stamp = datetime.now(timezone(timedelta(hours=8))).strftime("%Y-%m-%d-%H-%M-%S")
    output = args.root / (stamp + "-ik-jump-analysis")
    output.mkdir()
    folders = sorted(
        p for p in args.root.glob(args.prefix + "*-left") if (p / "events.jsonl").is_file()
    )
    report = {
        "scope": "OFFLINE_REAL_LOG_AUDIT_NO_ROBOT_COMMANDS",
        "episodes": [audit(p) for p in folders],
        "caveat": "Joint differences use the actual measured seed before IK. The send guard re-reads actual joints after IK; archived feedback is sampled at 20 Hz, not every controller packet.",
    }
    (output / "analysis.json").write_text(
        json.dumps(report, ensure_ascii=True, indent=2), encoding="utf-8"
    )
    if args.refresh_summaries:
        for folder in folders:
            old = folder / "analysis.json"
            if old.is_file():
                (folder / (stamp + "-analysis-before-audit.json")).write_bytes(old.read_bytes())
            old.write_text(
                json.dumps(analyze_log(folder / "events.jsonl"), ensure_ascii=True, indent=2),
                encoding="utf-8",
            )
    if args.plot:
        plot(report, output / "joint-jumps.png")
    print(
        json.dumps(
            {
                "output": str(output),
                "episodes": len(folders),
                "servo_acks": sum(p["servo_ack_count"] for p in report["episodes"]),
                "guard_failures": sum(
                    "IK_JOINT_STEP_EXCEEDS_10_DEG" in p["stop_reasons"] for p in report["episodes"]
                ),
            },
            ensure_ascii=True,
        )
    )


if __name__ == "__main__":
    main()
