"""Independently audit saved reference trajectories and source provenance."""
import offline_runtime as env
import argparse,csv,json
from pathlib import Path
import numpy as np
from controller_kinematics import PoseModel
from kinematics_validation import pose_error
from ranged_ik_profile import RangedIKProfile
from data_replay import load_episode,time_of
from controllers import tilt_deg
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("output",type=Path)
    p.add_argument("--model",type=Path,required=True);p.add_argument("--recordings",type=Path,required=True)
    args=p.parse_args();env.RECORDINGS=args.recordings.resolve()
    output=args.output.resolve();r=json.loads((output/"summary.json").read_text(encoding="utf-8"))
    model=PoseModel.from_dict(json.loads(args.model.read_text(encoding="utf-8-sig"))["model"])
    limits=np.array(RangedIKProfile.load(env.PROFILE).joint_limits_deg)
    checks={"all_replays_completed":True,"down_segments_certified":True,
            "velocity_acceleration_bounds":True,"absolute_joint_limits":True,
            "causal_input_only":True,"initial_fk_matches_historical":True}
    initials=[];details=[];metrics_rows=[];counts={}
    for name,episode in r["episodes"].items():
        ep=load_episode(name,model)
        initial=ep.feedback.at(ep.start)["data"]
        err=pose_error(model.fk(ep.q0),initial["pose_mm_deg"])
        initials.append({"episode":name,"position_error_mm":err[0],"orientation_error_deg":err[1]})
        checks["initial_fk_matches_historical"] &= err[0]<=.05 and err[1]<=.05
        seq_time={row["data"]["seq"]:time_of(row) for row in ep.hand.rows}
        for k in ("quest_frames","robot_feedback_frames","target_requests","target_ack_count",
                  "mapped_windows","raw_hand_sequence_matches","model_feedback_outside_005_count"):
            counts[k]=counts.get(k,0)+episode["data_audit"][k]
        h=episode["historical_metrics"]
        metrics_rows.append({"episode":name,"condition":"historical_actual","evidence":"measured original robot",
            "position_rmse_mm":h["target_to_actual_error_mm"]["rms"],
            "position_p95_mm":h["target_to_actual_error_mm"]["p95"],
            "tilt_max_deg":h["tool_axis_tilt_deg"]["max"]})
        for condition,m in episode["runs"].items():
            trace=np.load(output/name/condition/"trace.npz")
            checks["all_replays_completed"] &= m["completed"]
            checks["absolute_joint_limits"] &= bool(np.all(trace["q_deg"]>=limits[:,0]-1e-6) and np.all(trace["q_deg"]<=limits[:,1]+1e-6))
            for t,seq in zip(trace["t"],trace["source_seq"]):
                if seq<0:continue
                checks["causal_input_only"] &= seq in seq_time and seq_time[int(seq)]<=ep.start+float(t)+1e-8
            if condition.endswith("__hybrid"):
                dt=np.diff(trace["t"])
                vel=np.diff(trace["q_deg"],axis=0)/dt[:,None]
                acc=np.diff(np.vstack((np.zeros(6),vel)),axis=0)/dt[:,None]
                vmax=float(np.max(abs(vel)));amax=float(np.max(abs(acc)))
                checks["velocity_acceleration_bounds"] &= vmax<=60+1e-5 and amax<=120+1e-3
                detail={"episode":name,"condition":condition,"speed_max_deg_s":vmax,"acceleration_max_deg_s2":amax}
                if condition.startswith("downward"):
                    fresh_tilt=max(tilt_deg(model,np.radians(q)) for q in trace["q_deg"])
                    bound=float(np.max(trace["segment_tilt_upper_deg"]))
                    detail.update(recomputed_tilt_max_deg=fresh_tilt,stop_envelope_bound_max_deg=bound)
                    checks["down_segments_certified"] &= fresh_tilt<=5+1e-6 and bound<=5+1e-6
                details.append(detail)
            metrics_rows.append({"episode":name,"condition":condition,"evidence":"nominal model reference",
                "position_rmse_mm":m["position_reference_error_mm"]["rms"],
                "position_p95_mm":m["position_reference_error_mm"]["p95"],
                "tilt_max_deg":m["tool_axis_tilt_deg"]["max"],"command_step_max_deg":m["max_command_step_deg"],
                "reference_speed_max_deg_s":m["joint_reference_speed_deg_s"].get("max"),
                "reference_acceleration_max_deg_s2":m["joint_reference_acceleration_deg_s2"].get("max"),
                "compute_p95_ms":m["cycle_compute_ms"]["p95"],"compute_max_ms":m["cycle_compute_ms"]["max"],
                "cycles_over_20ms":m["cycle_over_20ms_count"],"reference_rejections":m["reference_rejections"],
                "completed":m["completed"]})
    for path,expected in r["source_hashes"].items():
        if env.sha256(path)!=expected:raise AssertionError("Source changed: "+path)
    if not all(checks.values()):raise AssertionError(checks)
    verification={"checks":checks,"counts":counts,"initial_pose_checks":initials,"independent_trace_checks":details,
          "source_hashes_unchanged":True,"network_attempts":env.NETWORK_ATTEMPTS,
          "verifier_sha256":env.sha256(__file__),
          "scope":"Nominal kinematic reference validation, not physical robot acceptance",
          "acceptance":"MEASURED_OFFLINE_CONSTRAINT_CHECKS_PASS"}
    (output/"verification.json").write_text(json.dumps(verification,ensure_ascii=False,indent=2),encoding="utf-8")
    fields=list(dict.fromkeys(k for row in metrics_rows for k in row))
    with (output/"comparison_metrics.csv").open("w",encoding="utf-8-sig",newline="") as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(metrics_rows)
    print(json.dumps(verification,ensure_ascii=False,indent=2))
if __name__=="__main__":main()
