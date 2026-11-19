"""Offline E6 kinematics and DLS study. No robot/network interfaces are imported.

Historical feedback stays historical. Synthetic configurations are FK queries only.
Run from any directory with the existing project Python environment.
"""
import sys
sys.dont_write_bytecode = True
import argparse
import hashlib
import itertools
import json
import math
import time
from datetime import datetime, timezone, timedelta
from pathlib import Path

NETWORK_ATTEMPTS = []
def deny_network(event, args):
    if event in ("socket.connect", "socket.connect_ex", "socket.sendto"):
        NETWORK_ATTEMPTS.append(event)
        raise RuntimeError("This study is offline; network access is forbidden")
sys.addaudithook(deny_network)

import numpy as np

PROJECT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT))
from controller_kinematics import PoseModel, pose_matrix, rotation_log
from kinematics_validation import pose_error

PROFILE_PATH = PROJECT / "profiles/offline_hybrid_v4.json"
LENGTH_M = 0.100

def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def skew(v):
    x,y,z = v
    return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])

def jacobian_si(model, q):
    j = model.jacobian(q) * (180 / math.pi)  # per-degree -> per-radian
    j[:3] /= 1000  # mm -> m; angular rows are already rad/rad
    return j

def tangent_basis(axis):
    ref = np.eye(3)[int(np.argmin(np.abs(axis)))]
    b1 = np.cross(axis, ref)
    b1 /= np.linalg.norm(b1)
    return np.column_stack((b1, np.cross(axis, b1)))

def task_jacobians(model, q, length=LENGTH_M):
    j = jacobian_si(model, q)
    a = model._forward(q)[:3,2]
    b = tangent_basis(a)
    full = np.vstack((j[:3] / length, j[3:]))
    axis = np.vstack((j[:3] / length, b.T @ (-skew(a)) @ j[3:]))
    return j, full, axis

def spectrum(matrix):
    u,s,vt = np.linalg.svd(matrix, full_matrices=False)
    tol = max(matrix.shape) * s[0] * np.finfo(float).eps
    return {"singular_values":s.tolist(), "sigma_min":float(s[-1]),
            "condition_number":float(s[0]/s[-1]) if s[-1]>tol else None,
            "numerical_rank":int(np.sum(s>tol)), "rank_tolerance":float(tol)}

def diagnostic(model, q, provenance):
    q=np.array(q,dtype=float)
    j,a,axis=task_jacobians(model,q)
    u,s,vt=np.linalg.svd(a)
    weak=u[:,-1].copy()
    joints=vt[-1].copy()
    if weak[np.argmax(np.abs(weak))]<0:
        weak=-weak; joints=-joints
    axes=j[3:]
    wrist_angle=math.degrees(math.acos(float(np.clip(abs(axes[:,3]@axes[:,5]),0,1))))
    tool_axis=model._forward(q)[:3,2]
    tilt=math.degrees(math.acos(float(np.clip(tool_axis@np.array([0,0,-1]),-1,1))))
    probes=[]
    for i,label in enumerate(("base_X","base_Y","base_Z","base_Rx","base_Ry","base_Rz")):
        target=np.zeros(6);target[i]=.010 if i<3 else math.radians(1)
        scaled=target.copy();scaled[:3]/=LENGTH_M
        rate=np.linalg.pinv(a,rcond=1e-10)@scaled
        achieved=j@rate
        residual=float(np.linalg.norm(a@rate-scaled))
        probes.append({"direction":label,
                       "requested_linear_m_s":target[:3].tolist(),
                       "requested_angular_rad_s":target[3:].tolist(),
                       "least_squares_joint_rates_deg_s":np.degrees(rate).tolist(),
                       "peak_abs_joint_rate_deg_s":float(np.max(np.abs(np.degrees(rate)))),
                       "normalized_residual":residual,
                       "normalized_unachievable_fraction":residual/float(np.linalg.norm(scaled)),
                       "exact_local_solution_exists_at_numeric_tolerance":residual<1e-9,
                       "achieved_linear_m_s":achieved[:3].tolist(),
                       "achieved_angular_rad_s":achieved[3:].tolist(),
                       "meaning":"mathematical probe, holding other five twist components zero; not a command"})
    return {"provenance":provenance,"q_deg":q.tolist(),"pose_mm_deg":model.fk(q),
            "normalization":{"q":"rad","linear":"m","angular":"rad","characteristic_length_m":LENGTH_M,
                             "task_velocity":"[v/L, omega]","joint_metric":"equal-weight joint-rate 2-norm"},
            "full_pose_6d":spectrum(a),
            "position_plus_tool_z_direction_5d":spectrum(axis),
            "position_only_3d":spectrum(j[:3]/LENGTH_M),
            "tool_z_base":tool_axis.tolist(),"tool_z_tilt_from_base_minus_z_deg":tilt,
            "j4_j6_acute_axis_angle_deg":wrist_angle,
            "weakest_full_pose_mode":{"scaled_twist_v_over_L_omega":weak.tolist(),
                   "linear_part_direction_base":(weak[:3]/np.linalg.norm(weak[:3])).tolist(),
                   "joint_combination":joints.tolist(),
                   "meaning":"coupled linear/angular direction; sign is arbitrary; not a pure X/Y/Z movement"},
            "pure_axis_velocity_probes":probes,
            "length_sensitivity":[{"length_m":length,**spectrum(task_jacobians(model,q,length)[1])}
                                  for length in (.05,.1,.2)],
            "limitations":["no certified safety threshold","no collision model",
                           "5D task frees spin about current tool Z; current axis may not point down"]}

def numerical_jacobian_check(model, q):
    eps=1e-4  # degree, deliberately separate FK finite differences
    q=np.array(q,float);numeric=np.zeros((6,6))
    for i in range(6):
        qp=q.copy();qm=q.copy();qp[i]+=eps;qm[i]-=eps
        tp=model._forward(qp);tm=model._forward(qm)
        numeric[:3,i]=(tp[:3,3]-tm[:3,3])/(2*eps)
        numeric[3:,i]=rotation_log(tp[:3,:3]@tm[:3,:3].T)/(2*eps)
    err=float(np.max(np.abs(numeric-model.jacobian(q))))
    if err>1e-6: raise AssertionError(("Jacobian finite difference mismatch",err))
    return err

def box_counterexample(model, q):
    """Two feasible configurations do not define a feasible Cartesian product."""
    goal=pose_matrix(model.fk(q))
    def quality(candidate):
        t=pose_matrix(model.fk(candidate))
        ep=goal[:3,:3].T@(t[:3,3]-goal[:3,3])
        er=np.degrees(rotation_log(goal[:3,:3].T@t[:3,:3]))
        return {"position_components_goal_mm":ep.tolist(),"position_norm_mm":float(np.linalg.norm(ep)),
                "rotation_components_goal_deg":er.tolist(),
                "feasible_in_example_task":bool(np.max(np.abs(ep))<=1 and np.max(np.abs(er))<=30)}
    _,a,_=task_jacobians(model,q)
    direction=np.linalg.svd(a)[2][-1]
    direction/=np.max(np.abs(direction))
    pair=None
    for amplitude in (.25,.5,1,2,4,8):
        qa=np.array(q)+amplitude*direction;qb=np.array(q)-amplitude*direction
        if quality(qa)["feasible_in_example_task"] and quality(qb)["feasible_in_example_task"]:
            pair=(amplitude,qa,qb)
    if pair is None: return {"found":False}
    amplitude,qa,qb=pair
    lo=np.minimum(qa,qb);hi=np.maximum(qa,qb)
    corners=[]
    for bits in itertools.product((0,1),repeat=6):
        candidate=np.where(bits,hi,lo)
        corners.append({"q_deg":candidate.tolist(),**quality(candidate)})
    worst=max(corners,key=lambda x:x["position_norm_mm"])
    return {"found":any(not x["feasible_in_example_task"] for x in corners),
            "scope":"synthetic FK queries around a historical measured configuration; no physical movement",
            "task":"1 mm per goal-frame position component, 30 deg per rotation-vector component",
            "amplitude_deg":amplitude,
            "endpoint_A":{"q_deg":qa.tolist(),**quality(qa)},
            "endpoint_B":{"q_deg":qb.tolist(),**quality(qb)},
            "joint_projection_min_deg":lo.tolist(),"joint_projection_max_deg":hi.tolist(),
            "corner_count":len(corners),
            "infeasible_corner_count":sum(not c["feasible_in_example_task"] for c in corners),
            "worst_corner":worst,
            "conclusion":"Coordinate-wise projected joint intervals lose coupled task constraints."}

def warm_seed(model, target, measured, limits, radii):
    q=np.array(measured,float);current=model._forward(q);goal=pose_matrix(target)
    error=np.r_[(goal[:3,3]-current[:3,3])/1000/LENGTH_M,
                rotation_log(goal[:3,:3]@current[:3,:3].T)]
    a=task_jacobians(model,q)[1]
    u,s,vt=np.linalg.svd(a)
    lam=max(1e-4,.1*math.exp(-.5*(s[-1]/.05)**2))
    step=np.degrees(vt.T@((s/(s*s+lam*lam))*(u.T@error)))
    peak=np.max(np.abs(step))
    if peak>2: step*=2/peak  # numerical trust step, NOT a physical speed limit
    low=np.maximum(np.array(limits)[:,0],q-np.array(radii))
    high=np.minimum(np.array(limits)[:,1],q+np.array(radii))
    alpha=1.
    for i in range(6):
        if step[i]>0: alpha=min(alpha,(high[i]-q[i])/step[i])
        if step[i]<0: alpha=min(alpha,(low[i]-q[i])/step[i])
    alpha=max(0,min(1,alpha))
    seed=q+alpha*step
    return seed.tolist(),{"lambda":lam,"sigma_min":float(s[-1]),"trust_step_peak_deg":float(np.max(abs(seed-q)))}

def warmstart_comparison(model, ik_rows, output):
    from ranged_ik_adapter import RangedIKAdapter
    from ranged_ik_profile import RangedIKProfile
    profile=RangedIKProfile.load(PROFILE_PATH)
    indices=np.unique(np.linspace(0,len(ik_rows)-1,min(20,len(ik_rows)),dtype=int))
    adapters={name:RangedIKAdapter(model,profile,ik_rows[0]["data"]["measured_seed_deg"],output)
              for name in ("measured_seed","adls_seed")}
    results=[]
    try:
        for n,index in enumerate(indices):
            row=ik_rows[int(index)];event=row["data"]
            q=event["measured_seed_deg"];target=event["requested_pose_mm_deg"]
            trials={}
            order=("measured_seed","adls_seed") if n%2==0 else ("adls_seed","measured_seed")
            for name in order:
                adapter=adapters[name];adapter.reset(q)
                started=time.perf_counter()
                if name=="measured_seed":
                    result=adapter.propose(target,q);seed=q;seed_info=None
                else:
                    seed,seed_info=warm_seed(model,target,q,profile.joint_limits_deg,profile.joint_search_radius_deg)
                    bounds=[(max(lo,x-r),min(hi,x+r)) for x,r,(lo,hi) in
                            zip(q,profile.joint_search_radius_deg,profile.joint_limits_deg)]
                    from e6_feedback import euler_xyz_deg_to_xyzw
                    candidate=adapter.backend.propose(target[:3],euler_xyz_deg_to_xyzw(target[3:]),
                              profile.position_tolerance_mm+profile.rotation_tolerance_deg,seed,bounds)
                    result=adapter.evaluate(target,candidate,q)
                elapsed=(time.perf_counter()-started)*1000
                pe,oe=pose_error(target,model.fk(result["q_deg"]))
                trials[name]={"status":result["status"],"total_ms":elapsed,
                              "position_center_error_mm":pe,"orientation_center_error_deg":oe,
                              "peak_delta_from_measured_deg":float(max(abs(a-b) for a,b in zip(result["q_deg"],q))),
                              "normalized_error_cost":result["task_error"]["normalized_error_cost"],
                              "q_deg":result["q_deg"],"initial_seed_deg":seed,"adls_seed_details":seed_info,
                              "native":dict(adapter.backend.last_stats)}
            results.append({"pc_time":row["pc_time"],"measured_q_deg":q,"requested_pose_mm_deg":target,"trials":trials})
    finally:
        for adapter in adapters.values(): adapter.close()
    summary={}
    for name in adapters:
        rows=[r["trials"][name] for r in results]
        summary[name]={"counts":{s:sum(r["status"]==s for r in rows) for s in ("WITHIN_RANGE","APPROACHING","HOLD_NO_PROGRESS")},
                       "median_ms":float(np.median([r["total_ms"] for r in rows])),
                       "p95_ms":float(np.percentile([r["total_ms"] for r in rows],95)),
                       "median_position_center_error_mm":float(np.median([r["position_center_error_mm"] for r in rows])),
                       "max_joint_delta_deg":max(r["peak_delta_from_measured_deg"] for r in rows)}
    return {"scope":"20 independent single-step historical requests, current profile, histories reset to measured q for BOTH methods",
            "not_proven":["closed-loop stability","physical tracking","general performance superiority"],
            "warm_parameters":{"L_m":LENGTH_M,"lambda_max":.1,"sigma_width":.05,"lambda_floor":1e-4,"trust_step_deg":2},
            "profile":profile.snapshot(),"summary":summary,"cases":results}

def plot_report(report, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    fp=font_manager.FontProperties(fname=r"C:\Windows\Fonts\msyh.ttc")
    plt.rcParams["font.family"]=fp.get_name()
    plt.rcParams["axes.unicode_minus"]=False
    fig,axs=plt.subplots(2,2,figsize=(14,9),layout="constrained")
    fig.suptitle("E6 离线研究：用户提供的历史记录，不是当前状态",fontsize=18)
    seq=report["historical_series"]
    elapsed=[(x["perf_ns"]-seq[0]["perf_ns"])/1e9 for x in seq]
    axs[0,0].semilogy(elapsed,[x["sigma_min"] for x in seq],color="#176d99")
    axs[0,0].set(title="历史片段的最小奇异值",xlabel="片段内时间 / s",ylabel="归一化 σ_min（L=100 mm）")
    w=report["historical_worst"]
    vec=w["weakest_full_pose_mode"]["scaled_twist_v_over_L_omega"]
    axs[0,1].bar(["vX/L","vY/L","vZ/L","ωX","ωY","ωZ"],vec,color=["#318fc3"]*3+["#e6a84b"]*3)
    axs[0,1].set(title="最弱运动模式：平移与旋转的组合",ylabel="左奇异向量分量")
    axs[0,1].axhline(0,color="gray",lw=.8)
    probes=w["pure_axis_velocity_probes"][:3]
    axs[1,0].bar(["纯 X 方向","纯 Y 方向","纯 Z 方向"],[100*p["normalized_unachievable_fraction"] for p in probes],color="#c56755")
    axs[1,0].set_ylim(0,110)
    axs[1,0].set(title="要求其他五项速度为零时的局部不可实现部分",ylabel="归一化最小二乘残差 / %（非实测误差）")
    box=report["joint_box_counterexample"]
    vals=[box["endpoint_A"]["position_norm_mm"],box["endpoint_B"]["position_norm_mm"],box["worst_corner"]["position_norm_mm"]]
    axs[1,1].bar(["可行端点 A","可行端点 B","关节区间混合角点"],vals,color=["#559872","#559872","#c56755"])
    axs[1,1].axhline(math.sqrt(3),ls="--",color="gray",label="每轴 1mm 容差的欧氏上界")
    axs[1,1].set(title="逐关节范围不能保证组合后末端可行",ylabel="末端位置误差 / mm")
    axs[1,1].legend(fontsize=8)
    for ax in axs.flat: ax.grid(axis="y",alpha=.2)
    fig.text(.5,-.018,"仅运动学分析；无碰撞/驱动动态验证；历史受限方向使用机器人基座坐标。",ha="center",fontsize=11)
    fig.savefig(output/"Historical_Diagnostics.png",dpi=150,bbox_inches="tight")
    plt.close(fig)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--model",type=Path,required=True)
    parser.add_argument("--prepare",type=Path,required=True)
    parser.add_argument("--episode",type=Path,required=True)
    args=parser.parse_args()
    model_path, prepare_path, episode = args.model.resolve(), args.prepare.resolve(), args.episode.resolve()
    output=args.output.resolve()
    output.mkdir(parents=True,exist_ok=False)
    files=[model_path,prepare_path,episode/"events.jsonl",PROFILE_PATH,PROJECT/"controller_kinematics.py"]
    hashes={str(p):digest(p) for p in files}
    manifest=json.loads(model_path.read_text(encoding="utf-8"))
    model=PoseModel.from_dict(manifest["model"])
    prep=json.loads(prepare_path.read_text(encoding="utf-8"))["result"]
    records=[json.loads(x) for x in (episode/"events.jsonl").read_text(encoding="utf-8").splitlines() if x]
    fb=[r for r in records if r["kind"]=="robot_feedback" and not r.get("pre_roll") and r["data"].get("valid")]
    ik=[r for r in records if r["kind"]=="controller_event" and not r.get("pre_roll") and r["data"].get("kind")=="IK_SOLUTION"]
    accepted=[];rejected=[]
    for row in fb:
        d=row["data"]
        pe,oe=pose_error(model.fk(d["joint_position_deg"]),d["pose_mm_deg"])
        if pe>.05 or oe>.05:
            rejected.append({"pc_time":row["pc_time"],"position_mm":pe,"angle_deg":oe});continue
        a=task_jacobians(model,d["joint_position_deg"])[1]
        accepted.append({"row":row,"sigma":float(np.linalg.svd(a,compute_uv=False)[-1]),"fk_mm":pe,"fk_deg":oe})
    if not accepted: raise RuntimeError("No consistent historical feedback")
    worst=min(accepted,key=lambda r:r["sigma"])
    provenance=lambda row: {"kind":"HISTORICAL_MEASURED_NOT_CURRENT","pc_time":row["pc_time"],
                             "robot_id":row["data"]["robot_id"],"robot_mode":row["data"]["robot_mode"],
                             "source_file":str(episode/"events.jsonl")}
    first=accepted[0]["row"];wr=worst["row"]
    checks=[numerical_jacobian_check(model,r["row"]["data"]["joint_position_deg"]) for r in (accepted[0],worst,accepted[-1])]
    # A basis-independent identity validates the 5D tool-axis Jacobian.
    q=wr["data"]["joint_position_deg"];j,_,ja=task_jacobians(model,q)
    axis=model._forward(q)[:3,2];b=tangent_basis(axis)
    test=np.array([.1,-.2,.3,-.1,.15,-.05])
    expected=b.T@np.cross(j[3:]@test,axis)
    if not np.allclose(ja[3:]@test,expected,atol=1e-12): raise AssertionError("5D axis derivative")
    report={"created_at":datetime.now(timezone(timedelta(hours=8))).isoformat(),
            "scope":"OFFLINE_ONLY_NO_NETWORK_NO_MOTION","current_measurement_available":False,
            "sources_sha256":hashes,"model_binding":{"arm_id":manifest["arm_id"],"model_sha256":manifest["model_sha256"],
                "historical_commission_evidence":prep["kinematics"],"current_robot_binding":"NOT_CHECKED"},
            "checks":{"finite_difference_max_errors_mm_or_rad_per_deg":checks,"tool_axis_jacobian_identity":True,
                      "historical_consistent_feedback_count":len(accepted),"historical_rejected_feedback":rejected},
            "historical_first":diagnostic(model,first["data"]["joint_position_deg"],provenance(first)),
            "historical_worst":diagnostic(model,q,provenance(wr)),
            "historical_series":[{"pc_time":r["row"]["pc_time"],"perf_ns":r["row"]["pc_perf_ns"],"sigma_min":r["sigma"],
                                   "q_deg":r["row"]["data"]["joint_position_deg"]} for r in accepted]}
    probe=first["data"]["joint_position_deg"]
    report["first_sample_model_probe"]=diagnostic(model,probe,{"kind":"SYNTHETIC_MODEL_QUERY_NOT_MEASURED",
                      "source":"FK evaluated at the supplied recording first sample"})
    report["elbow_counterfactuals"]=[]
    for delta in (-5,-1,1,5):
        modified=list(q);modified[2]+=delta
        report["elbow_counterfactuals"].append({"j3_change_deg":delta,
            "full_pose_6d":spectrum(task_jacobians(model,modified)[1]),
            "pose_change_mm_deg":pose_error(model.fk(q),model.fk(modified)),
            "scope":"synthetic FK query, not a proposed robot movement"})
    report["joint_box_counterexample"]=box_counterexample(model,first["data"]["joint_position_deg"])
    if not report["joint_box_counterexample"]["found"]: raise AssertionError("No interval counterexample found")
    report["warmstart_comparison"]=warmstart_comparison(model,ik,output/"native_model_bundles")
    for p in files:
        if digest(p)!=hashes[str(p)]:raise RuntimeError("Source changed during study: "+str(p))
    report["checks"]["source_hashes_unchanged"]=True
    report["checks"]["network_attempts"]=NETWORK_ATTEMPTS
    (output/"results.json").write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding="utf-8")
    plot_report(report,output)
    summary={"results":str(output/"results.json"),"historical_time":wr["pc_time"],
             "historical_q_deg":q,"full_pose":report["historical_worst"]["full_pose_6d"],
             "axis_task":report["historical_worst"]["position_plus_tool_z_direction_5d"],
             "weak_mode":report["historical_worst"]["weakest_full_pose_mode"],
             "pure_xyz_probes":[{"axis":p["direction"],"peak_joint_deg_s":p["peak_abs_joint_rate_deg_s"],
                                  "residual":p["normalized_residual"]} for p in report["historical_worst"]["pure_axis_velocity_probes"][:3]],
             "box":report["joint_box_counterexample"],
             "warmstart":report["warmstart_comparison"]["summary"],"checks":report["checks"]}
    print(json.dumps(summary,ensure_ascii=False,indent=2))

if __name__=="__main__":main()
