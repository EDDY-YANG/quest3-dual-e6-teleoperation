"""Read-only observational comparison of genuine Quest/E6 recording files.

No robot/SDK imports or network access. No reconstruction of missing feedback.
See methods.md for exactly what the tracking-gap and speed metrics mean.
"""
import argparse
import bisect
from collections import Counter, defaultdict
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import sys

import numpy as np


GROUPS = {
    "ik": "Local DLS IK (no QP)",
    "ik_qp": "Local DLS IK + QP",
    "ranged_v7": "RangedIK v7 direct",
}
MAX_GAP_S = 0.2
NORMAL_STOP_REASONS = {"GRIP_RELEASED", "OPERATOR_DISARMED", "SERVICE_CLEANUP"}


def forbid_network(event, args):
    if event in ("socket.connect", "socket.connect_ex", "socket.sendto"):
        raise RuntimeError("This statistics program forbids network and robot I/O")


sys.addaudithook(forbid_network)


def finite_vector(value, n):
    return isinstance(value, (list, tuple)) and len(value) == n and all(
        type(x) in (int, float) and math.isfinite(x) for x in value)


def numeric(value):
    return type(value) in (int, float) and math.isfinite(value)


def clean_reason(value):
    text = str(value)
    text = re.sub(r"\b(?:\d{1,3}\.){3}\d{1,3}\b", "[ip]", text)
    text = re.sub(r"\b(?:[0-9A-Fa-f]{2}[:-]){5}[0-9A-Fa-f]{2}\b", "[device-id]", text)
    return re.sub(r"[A-Za-z]:[\\/][^\n;]+", "[local path]", text)


def group_key(snapshot):
    limits, native = snapshot.get("limits") or {}, snapshot.get("ranged_ik") or {}
    solver, reference = limits.get("ik_solver"), limits.get("motion_reference")
    profile = native.get("profile_id")
    if solver == "LOCAL_BOUNDED_DLS_CONTROLLER_FK_VERIFIED":
        if reference == "FILTERED_VELOCITY_ACCELERATION_BOX_QP":
            return "ik_qp"
        if reference is None and not native:
            return "ik"
    if reference == "RANGEDIK_DIRECT" and profile == "e6_world_pose_geodesic_warm_v7":
        return "ranged_v7"
    return None


def identify_group(path):
    for line in path.open("rb"):
        row = json.loads(line)
        if row.get("pre_roll") or row.get("kind") != "control_snapshot":
            continue
        d = row.get("data") or {}
        if (d.get("limits") or {}).get("ik_solver"):
            native = d.get("ranged_ik") or {}
            return group_key(d), {
                "solver": d["limits"].get("ik_solver"),
                "motion_reference": d["limits"].get("motion_reference"),
                "profile_id": native.get("profile_id"),
                "profile_sha256": native.get("profile_sha256"),
                "position_target_policy": native.get("position_target_policy")
                    or d["limits"].get("position_target_policy") or "MEASURED_INCREMENTAL",
                "orientation_target_policy": native.get("orientation_target_policy")
                    or d["limits"].get("orientation_target_policy") or "MEASURED_INCREMENTAL",
                "recorded_parameters": {name:d["limits"].get(name) for name in (
                    "coordinate_mapping", "control_mode", "joint_step_limit_deg",
                    "tracking_window_deg", "reference_max_velocity_deg_s",
                    "reference_max_acceleration_deg_s2", "reference_max_tracking_error_deg",
                    "reference_goal_filter_tau_s", "reference_tracking_tau_s",
                    "servo_target_duration_s", "quest_motion_max_age_ms")},
            }
    return None, {"solver": "UNLABELLED_OR_LEGACY_VENDOR_IK"}


def rot_xyz_deg(angles):
    x, y, z = np.radians(angles)
    cx, sx, cy, sy, cz, sz = math.cos(x), math.sin(x), math.cos(y), math.sin(y), math.cos(z), math.sin(z)
    return np.array([[cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx],
                     [sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx],
                     [-sy, cy*sx, cy*cx]])


def pose_gap(target, actual):
    position = float(np.linalg.norm(np.asarray(target[:3])-np.asarray(actual[:3])))
    trace = np.trace(rot_xyz_deg(target[3:]).T @ rot_xyz_deg(actual[3:]))
    angle = math.degrees(math.acos(float(np.clip((trace-1)/2, -1, 1))))
    return position, angle


def latest_before(items, times, t, *, strict=False):
    index = (bisect.bisect_left(times, t) if strict else bisect.bisect_right(times, t))-1
    return None if index < 0 else items[index]


def statistics(values):
    a = np.asarray(values, dtype=float)
    if not len(a):
        return {"n": 0, "mean": None, "rmse": None, "p50": None, "p95": None, "max": None}
    return {"n": len(a), "mean": float(a.mean()), "rmse": float(np.sqrt(np.mean(a*a))),
            "p50": float(np.percentile(a, 50)), "p95": float(np.percentile(a, 95)), "max": float(a.max())}


def speed_between(a, b):
    dt = b["t"]-a["t"]
    if a["generation"] != b["generation"] or not 0 < dt <= MAX_GAP_S:
        return None
    return float(np.linalg.norm(np.asarray(b["pose"][:3])-np.asarray(a["pose"][:3]))/dt), dt


def read_record(path, expected_group):
    feedback, events, event_keys, offsets, groups = {}, [], set(), [], set()
    counters = Counter()
    drops, writer_error = None, None
    digest = hashlib.sha256()
    start_stat = path.stat()
    for raw in path.open("rb"):
        digest.update(raw)
        row = json.loads(raw)
        if row.get("pre_roll"):
            counters["pre_roll_rows_excluded"] += 1
            continue
        counters["non_preroll_rows"] += 1
        kind, d = row.get("kind"), row.get("data") or {}
        if kind == "control_snapshot":
            tag = group_key(d)
            if tag:
                groups.add(tag)
        elif kind == "robot_feedback":
            counters["feedback_rows"] += 1
            t = d.get("host_monotonic_s")
            if not (d.get("valid") is True and d.get("connected") is True
                    and numeric(t) and finite_vector(d.get("pose_mm_deg"), 6)):
                counters["invalid_feedback_rows_excluded"] += 1
                continue
            gen = d.get("connection_generation")
            key = (gen, t)
            if key in feedback:
                counters["duplicate_feedback_timestamps"] += 1
            feedback[key] = {"t": t, "generation": gen, "pose": d["pose_mm_deg"],
                             "mode": d.get("robot_mode"), "frame": d.get("frame_id")}
            if numeric(d.get("feedback_age_ms")) and numeric(row.get("pc_perf_ns")):
                offsets.append(row["pc_perf_ns"]/1e9-t-d["feedback_age_ms"]/1000)
        elif kind == "controller_event" and numeric(d.get("host_monotonic_s")):
            key = json.dumps(d, sort_keys=True)
            if key not in event_keys:
                event_keys.add(key)
                events.append(d)
        elif kind == "action_rejected":
            counters["action_rejected_events"] += 1
        elif kind == "command_exception":
            counters["command_exception_events"] += 1
        elif kind == "session_end":
            drops = d.get("dropped_records")
            counters["session_end_rows"] += 1
    after = path.stat()
    if after.st_size != start_stat.st_size or after.st_mtime_ns != start_stat.st_mtime_ns:
        raise RuntimeError("Source changed while reading: " + path.parent.name)
    if groups != {expected_group}:
        raise ValueError("MIXED_OR_UNKNOWN_ALGORITHM_WITHIN_FILE")
    ap = path.with_name("analysis.json")
    if ap.exists():
        analysis = json.loads(ap.read_text(encoding="utf-8"))
        writer_error = analysis.get("writer_error")
    return sorted(feedback.values(), key=lambda x:x["t"]), sorted(events, key=lambda x:x["host_monotonic_s"]), {
        **counters, "source_sha256": digest.hexdigest(), "source_bytes": start_stat.st_size,
        "dropped_records": drops, "writer_error": clean_reason(writer_error) if writer_error else None,
        "perf_counter_minus_monotonic_offset_s_median": float(np.median(offsets)) if offsets else None,
    }


def measure_record(folder, key, version):
    feedback, events, audit = read_record(folder/"events.jsonl", key)
    ft = [f["t"] for f in feedback]
    grips = [e for e in events if e["kind"] == "GRIP_ENGAGED"]
    stops = sorted([e["evidence"] for e in events if e["kind"] == "STOP_RESULT"
                    and numeric((e.get("evidence") or {}).get("requested_at"))], key=lambda e:e["requested_at"])
    acks = [e for e in events if e["kind"] == "TARGET_ACK" and finite_vector(e.get("pose_mm_deg"), 6)
            and finite_vector(e.get("q_deg"), 6)]
    goals = [e for e in events if e["kind"] == "INCREMENT_WINDOW" and e.get("decision") == "PLANNED"
             and finite_vector(e.get("target_pose_mm_deg"), 6)]
    goal_source = "PLANNED_INCREMENT_WINDOW_POST_CONSTRAINT_TARGET"
    if not goals:
        goals = [{**e, "target_pose_mm_deg": e["requested_pose_mm_deg"]} for e in events
                 if e["kind"] == "IK_SOLUTION" and finite_vector(e.get("requested_pose_mm_deg"), 6)]
        goal_source = "IK_SOLUTION_REQUEST" if goals else "UNAVAILABLE"
    data = defaultdict(list)
    segments = []
    excluded = Counter()
    for i, grip in enumerate(grips):
        begin = grip["host_monotonic_s"]
        next_grip = grips[i+1]["host_monotonic_s"] if i+1<len(grips) else math.inf
        end_event = next((s for s in stops if begin < s["requested_at"] <= next_grip), None)
        if end_event is None:
            excluded["unclosed_grip_segments"] += 1
            continue
        end = end_event["requested_at"]
        anchor = latest_before(feedback, ft, begin)
        if anchor is None or begin-anchor["t"] > MAX_GAP_S:
            excluded["missing_fresh_anchor_feedback"] += 1
            continue
        segment_feedback = [f for f in feedback if begin <= f["t"] < end]
        ff = [f for f in segment_feedback if f["generation"]==anchor["generation"] and f["frame"]==anchor["frame"]]
        excluded["connection_or_frame_mismatch_feedback"] += len(segment_feedback)-len(ff)
        aa = [e for e in acks if begin <= e["host_monotonic_s"] < end]
        gg = [e for e in goals if begin <= e["host_monotonic_s"] < end]
        at, gt = [e["host_monotonic_s"] for e in aa], [e["host_monotonic_s"] for e in gg]
        valid_speed_duration = 0.
        for j, feedback_row in enumerate(ff):
            t, pose = feedback_row["t"], feedback_row["pose"]
            # Coarse Windows monotonic ticks cannot establish within-tick order.
            # Do not associate a target whose timestamp equals the feedback tick.
            ack = latest_before(aa, at, t, strict=True)
            goal = latest_before(gg, gt, t, strict=True)
            if ack:
                ep, er = pose_gap(ack["pose_mm_deg"], pose)
                data["reference_position_mm"].append(ep)
                data["reference_orientation_deg"].append(er)
            if goal:
                ep, er = pose_gap(goal["target_pose_mm_deg"], pose)
                data["requested_position_mm"].append(ep)
                data["requested_orientation_deg"].append(er)
            if j:
                result = speed_between(ff[j-1], feedback_row)
                if result is None:
                    excluded["invalid_speed_intervals"] += 1
                else:
                    speed, dt = result
                    data["tcp_speed_mm_s"].append(speed)
                    data["tcp_speed_interval_s"].append(dt)
                    valid_speed_duration += dt
        for prev, current in zip(aa, aa[1:]):
            dt = current["host_monotonic_s"]-prev["host_monotonic_s"]
            if dt <= 0:
                excluded["nonpositive_ack_interval"] += 1
                continue
            data["command_joint_step_deg"].append(float(np.max(np.abs(np.asarray(current["q_deg"])-prev["q_deg"]))))
        segments.append({"grip_segment":i+1, "duration_s":end-begin,
                         "valid_feedback_count":len(ff), "accepted_target_count":len(aa),
                         "requested_goal_count":len(gg), "valid_speed_duration_s":valid_speed_duration,
                         "end_reason":clean_reason(end_event.get("reason")),
                         "stop_feedback_confirmed":end_event.get("feedback_confirmed") is True})
    stop_reasons = Counter(clean_reason(s.get("reason")) for s in stops)
    abnormal = sum(n for reason,n in stop_reasons.items() if reason not in NORMAL_STOP_REASONS)
    report = {"record_id":folder.name, "date":folder.name[:10], "side":"right" if "-right" in folder.name else "left",
              "algorithm":key, "algorithm_label":GROUPS[key], **version, "audit":audit,
              "recorded_grip_segments":len(grips), "closed_grip_segments_used":len(segments),
              "accepted_target_events":len(acks), "grip_duration_s":sum(s["duration_s"] for s in segments),
              "valid_speed_duration_s":sum(s["valid_speed_duration_s"] for s in segments),
              "requested_goal_source":goal_source, "stop_reason_counts":dict(stop_reasons),
              "non_normal_stop_events":abnormal, "exclusions":dict(excluded), "segments":segments,
              "metrics":{k:statistics(v) for k,v in data.items() if not k.endswith("interval_s")}}
    return report, data


def self_check():
    assert abs(pose_gap([0,0,0,0,0,0],[3,4,0,0,0,0])[0]-5)<1e-12
    assert abs(pose_gap([0,0,0,0,0,179],[0,0,0,0,0,-179])[1]-2)<1e-10
    a={"t":1.,"generation":1,"pose":[0,0,0]};b={"t":1.1,"generation":1,"pose":[3,4,0]}
    assert abs(speed_between(a,b)[0]-50)<1e-9
    assert speed_between(a,{**b,"generation":2}) is None
    assert speed_between(a,{**b,"t":1.}) is None
    assert speed_between(a,{**b,"t":1.3}) is None
    assert latest_before(["past","future"],[1.,3.],2.)=="past"
    assert latest_before(["future"],[3.],2.) is None
    assert latest_before(["same_tick"],[2.],2.,strict=True) is None
    assert abs(statistics([3,4])["rmse"]-math.sqrt(12.5))<1e-12
    assert statistics([])["rmse"] is None
    return {"checks_passed":11,"scope":"MATHEMATICAL_SELF_CHECKS_ONLY_NOT_HARDWARE_DATA"}


def write_csv(path, rows):
    keys=list(dict.fromkeys(k for row in rows for k in row))
    with path.open("w",encoding="utf-8",newline="") as stream:
        writer=csv.DictWriter(stream,fieldnames=keys);writer.writeheader();writer.writerows(rows)


def aggregate_groups(sessions, pool, side=None):
    groups=[]
    for key,label in GROUPS.items():
        matching=[r for r in sessions if r["algorithm"]==key and (side is None or r["side"]==side)]
        duration=sum(r["grip_duration_s"] for r in matching)
        row={"algorithm":key,"label":label,"headset_arm_record_files":len(matching),
             "record_files_with_accepted_target":sum(r["accepted_target_events"]>0 for r in matching),
             "record_files_with_tracking_samples":sum(r["metrics"].get("reference_position_mm",{}).get("n",0)>0 for r in matching),
             "dates":sorted({r["date"] for r in matching}),"sides":sorted({r["side"] for r in matching}),
             "accepted_target_events":sum(r["accepted_target_events"] for r in matching),
             "closed_grip_segments_used":sum(r["closed_grip_segments_used"] for r in matching),
             "grip_duration_s":duration,"valid_speed_duration_s":sum(r["valid_speed_duration_s"] for r in matching),
             "non_normal_stop_events":sum(r["non_normal_stop_events"] for r in matching),
             "recorded_queue_drops":sum(r["audit"]["dropped_records"] or 0 for r in matching),
             "records_with_unknown_drop_count":sum(r["audit"]["dropped_records"] is None for r in matching),
             "records_with_writer_error":sum(bool(r["audit"]["writer_error"]) for r in matching),
             "action_rejected_events":sum(r["audit"].get("action_rejected_events",0) for r in matching),
             "recorded_parameter_variants":[{"parameters":json.loads(parameters),"arm_record_count":count}
                 for parameters,count in Counter(json.dumps(r["recorded_parameters"],sort_keys=True)
                                                  for r in matching).items()],
             "metrics":{name:statistics(values) for name,values in pool[key].items() if not name.endswith("interval_s")}}
        groups.append(row)
    return groups


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recordings",type=Path,required=True)
    parser.add_argument("--output",type=Path,default=Path(__file__).resolve().parent/"results")
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    checked=self_check();sessions=[];excluded=[];pool={k:defaultdict(list) for k in GROUPS}
    left_pool={k:defaultdict(list) for k in GROUPS}
    right_pool={k:defaultdict(list) for k in GROUPS}
    for folder in sorted(args.recordings.iterdir()):
        if not folder.is_dir() or not (folder/"events.jsonl").is_file():continue
        mp=folder/"metadata.json"
        meta=json.loads(mp.read_text(encoding="utf-8")) if mp.exists() else {}
        if meta.get("source")!="paired_headset":
            excluded.append({"record_id":folder.name,"reason":"NOT_HEADSET_ARM_RECORD","source":meta.get("source")});continue
        key,version=identify_group(folder/"events.jsonl")
        if key is None:
            excluded.append({"record_id":folder.name,"reason":"OUTSIDE_THREE_VERIFIED_ALGORITHM_GROUPS",**version});continue
        report,data=measure_record(folder,key,version)
        report["controller_source_sha256"]=meta.get("disk_source_sha256_at_start",{}).get("left_z_trial.py")
        sessions.append(report)
        for name,values in data.items():
            pool[key][name].extend(values)
            (left_pool if report["side"]=="left" else right_pool)[key][name].extend(values)
        print(folder.name,key,"feedback",report["metrics"].get("reference_position_mm",{}).get("n",0),flush=True)
    groups=aggregate_groups(sessions,left_pool,"left")
    all_arm_groups=aggregate_groups(sessions,pool)
    right_only_groups=aggregate_groups(sessions,right_pool,"right")
    result={"schema":2,"generated_at_utc":datetime.now(timezone.utc).isoformat(),
            "scope":"OBSERVATIONAL_HARDWARE_LOG_ANALYSIS_NOT_MATCHED_TRAJECTORY_BENCHMARK",
            "video_alignment":"STAGE_LEVEL_ONLY_NOT_EXACT_VIDEO_SYNCHRONIZATION",
            "hybrid_dls_hardware_evidence":False,"self_checks":checked,
            "primary_comparison_scope":"LEFT_ARM_RECORDS_ONLY", "groups":groups,
            "all_arm_groups":all_arm_groups,"right_only_groups":right_only_groups}
    (args.output/"summary.json").write_text(json.dumps(result,indent=2,allow_nan=False),encoding="utf-8")
    (args.output/"sessions.json").write_text(json.dumps(sessions,indent=2,allow_nan=False),encoding="utf-8")
    (args.output/"excluded_records.json").write_text(json.dumps(excluded,indent=2,allow_nan=False),encoding="utf-8")
    aggregate=[]
    for row in groups:
        metric=lambda name,stat:row["metrics"].get(name,{}).get(stat)
        aggregate.append({"algorithm":row["algorithm"],"label":row["label"],"dates":";".join(row["dates"]),
                          "arm_records":row["headset_arm_record_files"],"tracking_arm_records":row["record_files_with_tracking_samples"],
                          "grip_segments":row["closed_grip_segments_used"],"grip_duration_s":row["grip_duration_s"],
                          "reference_position_rmse_mm":metric("reference_position_mm","rmse"),
                          "reference_position_p95_mm":metric("reference_position_mm","p95"),
                          "reference_orientation_p95_deg":metric("reference_orientation_deg","p95"),
                          "requested_position_rmse_mm":metric("requested_position_mm","rmse"),
                          "requested_position_p95_mm":metric("requested_position_mm","p95"),
                          "requested_orientation_p95_deg":metric("requested_orientation_deg","p95"),
                          "actual_tcp_speed_p50_mm_s":metric("tcp_speed_mm_s","p50"),
                          "actual_tcp_speed_p95_mm_s":metric("tcp_speed_mm_s","p95"),
                          "command_joint_step_p95_deg":metric("command_joint_step_deg","p95"),
                          "command_joint_step_max_deg":metric("command_joint_step_deg","max"),
                          "tracking_samples":metric("reference_position_mm","n"),
                          "speed_intervals":metric("tcp_speed_mm_s","n"),
                          "command_step_pairs":metric("command_joint_step_deg","n"),
                          "non_normal_arm_stop_events":row["non_normal_stop_events"],
                          "queue_drops":row["recorded_queue_drops"]})
    write_csv(args.output/"stage_metrics.csv",aggregate)
    session_rows=[]
    for r in sessions:
        row={k:r[k] for k in ["record_id","algorithm","date","side","accepted_target_events","closed_grip_segments_used","grip_duration_s","valid_speed_duration_s","non_normal_stop_events"]}
        row.update({k:r["audit"].get(k) for k in ["source_sha256","source_bytes","dropped_records","writer_error","action_rejected_events","command_exception_events","perf_counter_minus_monotonic_offset_s_median"]})
        row["controller_source_sha256"]=r["controller_source_sha256"]
        for metric,stats in r["metrics"].items():
            for field in ("n","mean","rmse","p50","p95","max"):row[metric+"_"+field]=stats[field]
        session_rows.append(row)
    write_csv(args.output/"per_record_metrics.csv",session_rows)
    # Unlabelled numerical distributions support publication plots without
    # exposing network identifiers or original trajectory coordinates.
    arrays={key+"__"+metric:np.asarray(values) for key,metrics in left_pool.items()
            for metric,values in metrics.items() if not metric.endswith("interval_s")}
    np.savez_compressed(args.output/"metric_distributions.npz",**arrays)
    all_arrays={key+"__"+metric:np.asarray(values) for key,metrics in pool.items()
                for metric,values in metrics.items() if not metric.endswith("interval_s")}
    np.savez_compressed(args.output/"metric_distributions_all_arms.npz",**all_arrays)
    print(json.dumps({"included_arm_records":len(sessions),"excluded_records":len(excluded),"metrics":aggregate},indent=2))


if __name__=="__main__":
    main()
