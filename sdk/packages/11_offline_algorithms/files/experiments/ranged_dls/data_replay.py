"""Causal replay and provenance for actual Quest/E6 recordings."""
import offline_runtime as env
import bisect,json
from dataclasses import dataclass
import numpy as np
from controller_kinematics import pose_matrix
from relative_pose import incremental_pose_target,anchored_position_target,grip_direction_mapping
from kinematics_validation import pose_error

def time_of(row):
    if "host_monotonic_s" in row["data"]:
        return float(row["data"]["host_monotonic_s"])
    return float(row["pc_perf_ns"])/1e9

class Signal:
    def __init__(self,rows,key=time_of):
        self.rows=sorted(rows,key=key)
        self.times=np.array([key(r) for r in self.rows])
    def at(self,t):
        i=bisect.bisect_right(self.times,t)-1
        return self.rows[i] if i>=0 else None

@dataclass
class Episode:
    name:str
    rows:list
    grip:dict
    start:float
    stop:float
    mapping:np.ndarray
    hand:Signal
    feedback:Signal
    targets:Signal
    acks:Signal
    windows:list
    q0:np.ndarray
    audit:dict

def load_episode(name,model):
    path=env.episode_directory(name)/"events.jsonl"
    rows=[json.loads(x) for x in path.read_text(encoding="utf-8").splitlines() if x]
    ev=[r for r in rows if r["kind"]=="controller_event" and not r.get("pre_roll")]
    grips=[r for r in ev if r["data"].get("kind")=="GRIP_ENGAGED"]
    if len(grips)!=1:raise ValueError("First prototype requires exactly one recorded grip per episode")
    grip=grips[0];start=time_of(grip)
    stops=[float(r["data"]["evidence"]["requested_at"]) for r in ev
           if r["data"].get("kind")=="STOP_RESULT" and r["data"].get("evidence",{}).get("requested_at",0)>=start]
    stop=min(stops) if stops else max(time_of(r) for r in ev)
    windows=[r for r in ev if r["data"].get("kind")=="INCREMENT_WINDOW" and "target_pose_mm_deg" in r["data"]]
    if not windows:raise ValueError("No mapped hand input windows")
    mapping=np.array(windows[0]["data"]["direction_mapping"],float)
    if not np.allclose(mapping@mapping.T,np.eye(3),atol=1e-6) or np.linalg.det(mapping)<.999:
        raise ValueError("Invalid direction rotation")
    hand=Signal([r for r in rows if r["kind"]=="quest_frame"])
    feedback=Signal([r for r in rows if r["kind"]=="robot_feedback" and r["data"].get("valid")])
    first=feedback.at(start)
    if first is None or start-time_of(first)>.2:raise ValueError("No fresh historical initial feedback")
    q0=np.array(first["data"]["joint_position_deg"],float)
    targets=Signal([r for r in ev if r["data"].get("kind")=="IK_SOLUTION"])
    acks=Signal([r for r in ev if r["data"].get("kind")=="TARGET_ACK"])
    mapping_errors=[];matched_seq=0
    seqs={r["data"]["seq"]:r for r in hand.rows}
    for row in windows:
        w=row["data"]
        if w.get("position_target_policy")=="GRIP_ANCHORED_WORLD":
            rebuilt=anchored_position_target(w["hand_anchor"],w["robot_anchor_pose_mm_deg"],
                        w["hand_previous"],w["robot_measured_pose_mm_deg"],w["hand_current"],w["direction_mapping"])
        else:
            rebuilt=incremental_pose_target(w["hand_previous"],w["robot_measured_pose_mm_deg"],
                                              w["hand_current"],w["direction_mapping"])
        mapping_errors.append(pose_error(rebuilt,w["target_pose_mm_deg"]))
        raw=seqs.get(w.get("seq_to"))
        if raw and raw["data"].get("hands",{}).get("left",{}).get("pose"):
            hp=raw["data"]["hands"]["left"]["pose"]
            if np.allclose(hp["position_m"],w["hand_current"]["position_m"],atol=1e-9):
                matched_seq+=1
    if max(p for p,a in mapping_errors)>1e-5 or max(a for p,a in mapping_errors)>.001:
        raise AssertionError("Recorded hand-to-target mapping cannot be reproduced")
    fbchecks=[pose_error(model.fk(r["data"]["joint_position_deg"]),r["data"]["pose_mm_deg"])
              for r in feedback.rows if start<=time_of(r)<=stop]
    latencies=[]
    window_by_pose={tuple(r["data"]["target_pose_mm_deg"]):r["data"] for r in windows}
    for row in acks.rows:
        w=window_by_pose.get(tuple(row["data"]["requested_pose_mm_deg"]))
        if w is not None:
            delta=(time_of(row)-w["input_received_at"])*1000
            if delta>=0:latencies.append(delta)
    command_latency=[r["data"]["duration_ms"] for r in rows
                     if r["kind"]=="command_return" and r["data"].get("method")=="ServoJ" and not r.get("pre_roll")]
    audit={"source":str(path),"source_sha256":env.sha256(path),"historical_only":True,
       "grip_pc_time":grip["pc_time"],"duration_s":stop-start,"quest_frames":len(hand.rows),
       "robot_feedback_frames":len(feedback.rows),"target_requests":len(targets.rows),
       "target_ack_count":len(acks.rows),"mapped_windows":len(windows),
       "raw_hand_sequence_matches":matched_seq,
       "mapping_max_position_error_mm":max(p for p,a in mapping_errors),
       "mapping_max_orientation_error_deg":max(a for p,a in mapping_errors),
       "model_feedback_max_position_error_mm":max(p for p,a in fbchecks),
       "model_feedback_max_orientation_error_deg":max(a for p,a in fbchecks),
       "model_feedback_outside_005_count":sum(p>.05 or a>.05 for p,a in fbchecks),
       "input_receive_to_ack_ms":latencies,"servoj_request_to_ack_ms":command_latency,
       "frame":"User0/Tool0; base-frame coordinates; original recorded calibration retained",
       "clock":"host_monotonic_s, causal zero-order hold; no future samples",
       "stop_reason":next((r["data"]["evidence"].get("reason") for r in ev if r["data"].get("kind")=="STOP_RESULT"),None)}
    return Episode(name,rows,grip,start,stop,mapping,hand,feedback,targets,acks,windows,q0,audit)
