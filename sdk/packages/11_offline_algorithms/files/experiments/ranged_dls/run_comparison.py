"""Run reproducible, network-disabled reference-generation comparisons."""
import offline_runtime as env
import argparse,json,math,time
from dataclasses import asdict
from datetime import datetime,timezone,timedelta
from pathlib import Path
import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation
from controller_kinematics import PoseModel,pose_matrix,rotation_log
from relative_pose import _hand_pose,_near_euler
from ranged_ik_profile import RangedIKProfile
from data_replay import load_episode,time_of
from controllers import Settings,ReferencePlanner,ConstrainedDLS,ControllerInfeasible,tilt_deg,certify_down_segment

def summary_numbers(values):
    a=np.array(values,dtype=float)
    a=a[np.isfinite(a)]
    if not len(a):return {"count":0}
    return {"count":len(a),"mean":float(np.mean(a)),"rms":float(np.sqrt(np.mean(a*a))),
            "median":float(np.median(a)),"p95":float(np.percentile(a,95)),"max":float(np.max(a))}
def row_pose_error(actual,goal):
    return float(np.linalg.norm(np.array(actual[:3])-np.array(goal[:3])))

class Inputs:
    def __init__(self,episode,scenario,model,q0,settings):
        self.ep=episode;self.scenario=scenario;self.model=model;self.cfg=settings
        self.initial=model._forward(np.degrees(q0))
        self.anchor_position,self.anchor_rotation=_hand_pose(episode.grip["data"]["hand_pose"])
        self.anchor_position=np.array(self.anchor_position)
        self.anchor_rotation=np.array(self.anchor_rotation)
        self.yaw=0.
        self.initial_euler=model.fk(np.degrees(q0))[3:]
        self.last_goal=model.fk(np.degrees(q0))
    def sample(self,now):
        row=self.ep.hand.at(now)
        if row is None:return self.last_goal,False,None,None,"NO_INPUT"
        frame=row["data"];hand=frame.get("hands",{}).get("left",{})
        age=now-time_of(row)
        active=(self.ep.start<=now<self.ep.stop and 0<=age<=self.cfg.input_timeout_s
                and hand.get("tracking_valid",False) and hand.get("grip_held",False)
                and hand.get("pose") is not None and not hand["pose"].get("emulated_position",False)
                and frame.get("visibility")=="visible")
        if self.scenario=="as_recorded":
            target=self.ep.targets.at(now)
            if target:self.last_goal=list(target["data"]["requested_pose_mm_deg"])
        elif active:
            p,r=_hand_pose(hand["pose"]);r=np.array(r)
            delta=self.ep.mapping@r@self.anchor_rotation.T@self.ep.mapping.T
            quaternion=Rotation.from_matrix(delta).as_quat()
            norm=math.hypot(quaternion[2],quaternion[3])
            if norm<1e-6:return self.last_goal,False,frame["seq"],age,"YAW_TWIST_UNDEFINED"
            angle=2*math.atan2(quaternion[2],quaternion[3])
            self.yaw=angle+2*math.pi*round((self.yaw-angle)/(2*math.pi))
            c,s=math.cos(self.yaw),math.sin(self.yaw)
            rz=np.array([[c,-s,0],[s,c,0],[0,0,1.]])
            target_r=rz@self.initial[:3,:3]
            target_p=self.initial[:3,3]+1000*self.ep.mapping@(np.array(p)-self.anchor_position)
            self.last_goal=target_p.tolist()+list(_near_euler(target_r,self.initial_euler))
        return self.last_goal,active,frame.get("seq"),age,"ACTIVE" if active else "RELEASED_INVALID_OR_STALE"

def reference_metrics(trace,settings):
    t=trace["t"];mask=trace["active"].astype(bool)
    error=np.linalg.norm(trace["pose"][:,:3]-trace["goal"][:,:3],axis=1)
    emit=trace["emitted"].astype(bool)
    ids=np.r_[0,np.where(emit)[0]]
    ids=np.unique(ids)
    dt=np.diff(t[ids])
    good=dt>1e-10
    velocity=np.diff(trace["q_deg"][ids],axis=0)[good]/dt[good,None]
    vt=t[ids][1:][good]
    dv=np.diff(vt)
    acceleration=np.diff(velocity,axis=0)/dv[:,None] if len(velocity)>1 else np.empty((0,6))
    counts={}
    for s in trace["status"]:
        counts[str(s)]=counts.get(str(s),0)+1
    return {"active_samples":int(mask.sum()),"position_reference_error_mm":summary_numbers(error[mask]),
        "tool_axis_tilt_deg":summary_numbers(trace["tilt_deg"][mask]),
        "tilt_exceeds_5_count":int(np.sum(trace["tilt_deg"][mask]>5+1e-6)),
        "max_command_step_deg":float(np.max(np.abs(np.diff(trace["q_deg"][ids],axis=0)))) if len(ids)>1 else 0,
        "joint_reference_speed_deg_s":summary_numbers(np.max(abs(velocity),axis=1)) if len(velocity) else {},
        "joint_reference_acceleration_deg_s2":summary_numbers(np.max(abs(acceleration),axis=1)) if len(acceleration) else {},
        "cycle_compute_ms":summary_numbers(trace["cycle_ms"][1:]),
        "planner_compute_ms":summary_numbers(trace["planner_ms"][trace["planner_ms"]>0]),
        "executor_compute_ms":summary_numbers(trace["executor_ms"][trace["executor_ms"]>0]),
        "cycle_over_20ms_count":int(np.sum(trace["cycle_ms"]>1000/settings.execution_hz)),
        "reference_rejections":int(np.sum(trace["reference_rejected"])),
        "solver_warning_count":int(np.sum(trace["warning_count"])),
        "status_counts":counts,
        "metric_scope":"FK of nominal joint references, not measured new robot motion; direct speed uses actual reference publication intervals"}

def replay(ep,scenario,variant,model,output,settings,down_seed_deg):
    down=scenario=="downward_rebased"
    hybrid=variant=="hybrid"
    matched=variant=="direct_matched"
    q0=np.radians(down_seed_deg) if down else np.radians(ep.q0)
    planner=ReferencePlanner(model,q0,output/"native",down_aware=down and (hybrid or matched))
    executor=ConstrainedDLS(model,q0,planner.profile.joint_limits_deg,settings,enforce_down=down) if hybrid else None
    inputs=Inputs(ep,scenario,model,q0,settings)
    q=q0.copy();reference=None;dt=1/settings.execution_hz
    stride=round(settings.execution_hz/settings.planner_hz)
    n=math.ceil((ep.stop-ep.start+1.0)/dt)
    initial_pose=model.fk(np.degrees(q))
    rows=[{"t":0.,"q_deg":np.degrees(q).tolist(),"pose":initial_pose,"goal":initial_pose,
           "active":False,"tilt_deg":tilt_deg(model,q),"cycle_ms":0.,"planner_ms":0.,"executor_ms":0.,
           "emitted":False,"status":"INITIAL_CONDITION","source_seq":-1,"input_age_ms":0.,
           "reference_rejected":False,"warning_count":0,"segment_tilt_upper_deg":tilt_deg(model,q)}]
    planner_events=[];failure=None;rejections=0
    try:
        for tick in range(1,n+1):
            started=time.perf_counter();now=ep.start+tick*dt
            goal,active,seq,age,input_status=inputs.sample(now)
            plan_ms=exec_ms=0.;rejected=False;emitted=False;warning=0
            status="HOLD";segment_upper=tilt_deg(model,q)
            if tick%stride==0 and active:
                ref,result,info=planner.solve(goal,q,now)
                plan_ms=info["elapsed_ms"]
                info.update(t=now-ep.start,target_pose=goal,
                            q_ref_deg=np.degrees(ref.q).tolist() if ref else None,
                            reference_history_contract="PLANNING_ONLY")
                if ref is None:
                    rejected=True;status="REFERENCE_DOWN_REJECTED"
                elif not hybrid and np.max(abs(np.degrees(ref.q-q)))>settings.max_command_delta_deg+1e-7:
                    rejected=True;status="DIRECT_50_DEG_GUARD"
                elif matched:
                    ok,segment_upper,subdivisions=certify_down_segment(model,q,ref.q,settings.down_limit_deg)
                    info["segment_bound_deg"]=segment_upper
                    if not ok:
                        rejected=True;status="DIRECT_DOWN_PATH_REJECTED"
                if not rejected and ref is not None:
                    planner.commit_reference(ref,result)
                    reference=ref
                    if not hybrid:
                        q=ref.q.copy();emitted=True;status=result["status"]
                    else:status="REFERENCE_UPDATED"
                else:
                    rejections+=1
                info["accepted"]=not rejected
                planner_events.append(info)
            if hybrid:
                usable=(active and reference is not None and now-reference.generated_at<=settings.reference_timeout_s)
                q,info=executor.step(reference,dt,active=usable)
                emitted=True;exec_ms=info["elapsed_ms"];warning=info["numerical_warning_count"]
                segment_upper=info["nominal_segment_tilt_upper_deg"];status=info["status"]
            pose=model.fk(np.degrees(q))
            cycle_ms=(time.perf_counter()-started)*1000
            rows.append({"t":now-ep.start,"q_deg":np.degrees(q).tolist(),"pose":pose,"goal":list(goal),
                "active":active,"tilt_deg":tilt_deg(model,q),"cycle_ms":cycle_ms,
                "planner_ms":plan_ms,"executor_ms":exec_ms,"emitted":emitted,"status":status,
                "source_seq":seq if seq is not None else -1,"input_age_ms":age*1000 if age is not None else -1.,
                "reference_rejected":rejected,"warning_count":warning,"segment_tilt_upper_deg":segment_upper})
    except ControllerInfeasible as error:
        failure={"reason":str(error),"t":(len(rows))*dt,
                 "q_deg":np.degrees(q).tolist(),"no_fabricated_continuation":True}
    finally:
        planner.close()
    trace={k:np.array([row[k] for row in rows]) for k in rows[0]}
    np.savez_compressed(output/"trace.npz",**trace)
    metrics=reference_metrics(trace,settings)
    metrics.update(completed=failure is None,failure=failure,scenario=scenario,variant=variant,
                   duration_s=float(trace["t"][-1]),initial_q_deg=np.degrees(q0).tolist(),
                   enforce_down=down and (hybrid or matched),
                   profile=planner.profile.snapshot(),executor_settings=asdict(settings),
                   mathematical_replay="Causal joint-reference generation with its own nominal state; no plant/servo dynamics model",
                   initial_condition_scope="synthetic down candidate, not a measured/validated start" if down else "historical measured initial q",
                   orientation_policy="tool Z -> base -Z; mapped hand twist controls yaw; no roll/pitch following" if down else "recorded original orientation goals; down constraint disabled")
    (output/"metrics.json").write_text(json.dumps(metrics,ensure_ascii=False,indent=2,allow_nan=False),encoding="utf-8")
    (output/"planner_events.json").write_text(json.dumps(planner_events,ensure_ascii=False,indent=2,allow_nan=False),encoding="utf-8")
    return metrics,trace

def historical_metrics(ep,model,output):
    rows=[]
    for row in ep.feedback.rows:
        now=time_of(row)
        if not ep.start<=now<=ep.stop:continue
        target=ep.targets.at(now);ack=ep.acks.at(now)
        if target is None:continue
        data=row["data"];goal=target["data"]["requested_pose_mm_deg"]
        actual=data["pose_mm_deg"]
        command=ack["data"]["pose_mm_deg"] if ack else ep.grip["data"]["robot_pose_mm_deg"]
        rows.append({"t":now-ep.start,"pose":actual,"goal":goal,"command":command,
                     "q_deg":data["joint_position_deg"],"tilt_deg":tilt_deg(model,np.radians(data["joint_position_deg"]))})
    trace={k:np.array([r[k] for r in rows]) for k in rows[0]}
    np.savez_compressed(output/"historical_actual.npz",**trace)
    metrics={"kind":"HISTORICAL_REAL_MEASUREMENTS_NOT_NEW_ALGORITHM",
             "target_to_actual_error_mm":summary_numbers(np.linalg.norm(trace["pose"][:,:3]-trace["goal"][:,:3],axis=1)),
             "ack_command_to_actual_error_mm":summary_numbers(np.linalg.norm(trace["pose"][:,:3]-trace["command"][:,:3],axis=1)),
             "tool_axis_tilt_deg":summary_numbers(trace["tilt_deg"]),
             "input_receive_to_ack_ms":summary_numbers(ep.audit["input_receive_to_ack_ms"]),
             "servoj_request_to_ack_ms":summary_numbers(ep.audit["servoj_request_to_ack_ms"]),
             "note":"Tracking error includes original mapping/target choice and physical servo lag; ACK is not physical arrival"}
    return metrics,trace

def make_plots(ep,all_runs,historical,out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    plt.rcParams["font.family"]=font_manager.FontProperties(fname=r"C:\Windows\Fonts\msyh.ttc").get_name()
    plt.rcParams["axes.unicode_minus"]=False
    colors={"direct_current":"#d07b24","direct_matched":"#7658a6","hybrid":"#16846b"}
    names={"direct_current":"当前 RangedIK 直接参考","direct_matched":"朝下约束匹配的直接参考","hybrid":"RangedIK + 受约束 DLS"}
    fig,axs=plt.subplots(2,2,figsize=(14,8),layout="constrained")
    fig.suptitle(ep.name+"：同一历史任务，真实数据与离线参考",fontsize=17)
    for i,ax in enumerate(axs.flat[:3]):
        ax.plot(historical["t"],historical["goal"][:,i],color="black",lw=1,label="历史末端请求")
        ax.plot(historical["t"],historical["pose"][:,i],color="#2769b1",lw=1.5,label="历史实测 TCP")
        ax.plot(historical["t"],historical["command"][:,i],color="#999999",ls=":",label="历史 ACK 目标")
        for v in ("direct_current","hybrid"):
            trace=all_runs[("as_recorded",v)][1]
            ax.plot(trace["t"],trace["pose"][:,i],color=colors[v],ls="--",lw=1,label=names[v]+"（模型）")
        ax.set(title="基座 "+"XYZ"[i],ylabel="位置 / mm",xlabel="Grip 后时间 / s")
    ax=axs[1,1]
    ax.plot(historical["t"],np.linalg.norm(historical["pose"][:,:3]-historical["goal"][:,:3],axis=1),
            color="#2769b1",label="历史实测误差")
    for v in ("direct_current","hybrid"):
        trace=all_runs[("as_recorded",v)][1]
        ax.plot(trace["t"],np.linalg.norm(trace["pose"][:,:3]-trace["goal"][:,:3],axis=1),
                color=colors[v],label=names[v]+"（模型参考误差）")
    ax.set(title="相对原始请求的位置误差",ylabel="欧氏误差 / mm",xlabel="时间 / s")
    for ax in axs.flat:
        ax.axvspan(ep.stop-ep.start,ep.stop-ep.start+1,color="gray",alpha=.1)
        ax.grid(alpha=.2)
    handles,labels=axs[0,0].get_legend_handles_labels()
    fig.legend(handles,labels,loc="outside lower center",ncol=3,fontsize=9)
    fig.savefig(out/"Original_Data_Comparison.png",dpi=150,bbox_inches="tight");plt.close(fig)
    fig,axs=plt.subplots(2,2,figsize=(14,8),layout="constrained")
    fig.suptitle(ep.name+"：同一手柄输入的朝下任务离线对照\n共同模型初态，1:1 位移重置原点；不是新算法实机结果",fontsize=16)
    for variant in ("direct_current","direct_matched","hybrid"):
        metrics,trace=all_runs[("downward_rebased",variant)]
        err=np.linalg.norm(trace["pose"][:,:3]-trace["goal"][:,:3],axis=1)
        axs[0,0].plot(trace["t"],err,label=names[variant],color=colors[variant])
        axs[0,1].plot(trace["t"],trace["tilt_deg"],color=colors[variant])
        ids=np.unique(np.r_[0,np.where(trace["emitted"])[0]])
        dt=np.diff(trace["t"][ids]);valid=dt>1e-10
        rates=np.diff(trace["q_deg"][ids],axis=0)[valid]/dt[valid,None]
        axs[1,0].plot(trace["t"][ids][1:][valid],np.max(abs(rates),axis=1),color=colors[variant])
        axs[1,1].plot(trace["t"],trace["cycle_ms"],color=colors[variant],alpha=.65)
    axs[0,0].set(title="相对手柄目标的位置参考误差",ylabel="欧氏误差 / mm")
    axs[0,1].set(title="工具 +Z 相对基座 -Z 的倾角",ylabel="角度 / deg")
    axs[0,1].axhline(5,color="red",ls="--",lw=1)
    axs[1,0].set(title="最大关节参考速度（按实际发出间隔差分）",ylabel="deg/s")
    axs[1,0].axhline(60,color="gray",ls="--",lw=1)
    axs[1,1].set(title="本机每周期计算耗时，不含网络/机器人",ylabel="ms")
    axs[1,1].axhline(20,color="red",ls="--",lw=1)
    for ax in axs.flat:
        ax.set_xlabel("Grip 后时间 / s")
        ax.axvspan(ep.stop-ep.start,ep.stop-ep.start+1,color="gray",alpha=.1)
        ax.grid(alpha=.2)
    handles,labels=axs[0,0].get_legend_handles_labels()
    fig.legend(handles,labels,loc="outside lower center",ncol=3,fontsize=10)
    fig.savefig(out/"Downward_Comparison.png",dpi=150,bbox_inches="tight");plt.close(fig)

def diagnose_reference_feasibility(runs,model):
    """Post-run static diagnostic only. Never supplies future data to a controller."""
    trace=runs[("downward_rebased","hybrid")][1]
    active=np.where(trace["active"])[0]
    if not len(active):return []
    errors=np.linalg.norm(trace["pose"][:,:3]-trace["goal"][:,:3],axis=1)
    ids=np.unique([active[len(active)//2],active[-1],active[np.argmax(errors[active])]])
    absolute=np.radians(np.array(RangedIKProfile.load(env.PROFILE).joint_limits_deg))
    results=[]
    for i in ids:
        qbase=np.radians(trace["q_deg"][i]);target=pose_matrix(trace["goal"][i])
        bounds=np.column_stack((np.maximum(absolute[:,0],qbase-math.pi/3),
                                np.minimum(absolute[:,1],qbase+math.pi/3)))
        def components(q):
            actual=model._forward(np.degrees(q))
            ep=target[:3,:3].T@(actual[:3,3]-target[:3,3])
            er=rotation_log(target[:3,:3].T@actual[:3,:3])
            g=np.r_[(-actual[2,2])-math.cos(math.radians(5)),
                    np.radians([3,3,30])-np.abs(er)]
            return ep,er,g
        def objective(q):
            ep,er,g=components(q)
            return float(ep@ep)/10000+1e-8*float((q-qbase)@(q-qbase))
        seeds=[qbase]
        for variant in ("direct_current","direct_matched"):
            other=runs[("downward_rebased",variant)][1]
            j=min(int(np.searchsorted(other["t"],trace["t"][i])),len(other["t"])-1)
            seeds.append(np.clip(np.radians(other["q_deg"][j]),bounds[:,0],bounds[:,1]))
        candidates=[]
        started=time.perf_counter()
        for seed in seeds:
            result=minimize(objective,seed,bounds=list(map(tuple,bounds)),
                constraints=[{"type":"ineq","fun":lambda q:components(q)[2]}],
                method="SLSQP",options={"maxiter":150,"ftol":1e-10,"disp":False})
            ep,er,g=components(result.x)
            candidates.append({"optimizer_success":bool(result.success),"message":str(result.message),
                "position_error_mm":float(np.linalg.norm(ep)),"max_position_component_mm":float(np.max(abs(ep))),
                "tilt_deg":tilt_deg(model,result.x),"rotation_components_deg":np.degrees(er).tolist(),
                "constraints_valid":bool(np.min(g)>=-1e-7),"q_deg":np.degrees(result.x).tolist()})
        valid=[x for x in candidates if x["constraints_valid"]]
        best=min(valid,key=lambda x:x["position_error_mm"]) if valid else None
        results.append({"t":float(trace["t"][i]),"target_pose_mm_deg":trace["goal"][i].tolist(),
            "best_admissible_local_result":best,"all_attempts":candidates,
            "exact_task_feasible_example_found":bool(best and best["max_position_component_mm"]<=1+1e-6),
            "diagnostic_ms":(time.perf_counter()-started)*1000,
            "scope":"Post-run three-start constrained IK diagnostic, same +/-60deg local search box. Failure is not a global infeasibility proof. No path/velocity certificate."})
    return results

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--model",type=Path,required=True,help="An identified model exported by your own setup")
    parser.add_argument("--recordings",type=Path,required=True,help="Parent directory of your own event recordings")
    parser.add_argument("--episodes",nargs="+",required=True,help="Names of recording directories")
    parser.add_argument("--down-seed-deg",type=float,nargs=6,required=True,help="Six research-only joint angles for the downward comparison")
    args=parser.parse_args()
    env.RECORDINGS=args.recordings.resolve()
    model_file=args.model.resolve()
    model=PoseModel.from_dict(json.loads(model_file.read_text(encoding="utf-8-sig"))["model"])
    if len(set(args.episodes)) != len(args.episodes):
        parser.error("Episode names must be unique")
    for name in args.episodes:
        if not (env.episode_directory(name)/"events.jsonl").is_file():
            parser.error("Missing events.jsonl in "+name)
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    settings=Settings()
    files=[model_file,env.PROFILE,*[env.PROJECT/x for x in
           ("controller_kinematics.py","ranged_ik_adapter.py","ranged_ik_backend.py","relative_pose.py","left_z_trial.py","server.py","dobot_safe_sdk.py")]]
    for name in args.episodes:files.append(env.episode_directory(name)/"events.jsonl")
    hashes={str(p):env.sha256(p) for p in files}
    report={"created_at":datetime.now(timezone(timedelta(hours=8))).isoformat(),
        "scope":"OFFLINE_PROTOTYPE_NO_ROBOT_CONNECTIONS","source_hashes":hashes,
        "settings":asdict(settings),"episodes":{},"measurement_boundaries":[
        "Historical TCP is measured; all new-controller traces are mathematical reference trajectories.",
        "No actuator, ServoJ queue, communication latency, collision or calibrated dynamic model is simulated.",
        "Downward replay is a counterfactual task with a declared common synthetic initial pose.",
        "Original replay retains historical orientation goals and disables the down constraint.",
        "RangedIK history is planning-reference history; executor state/history is maintained independently.",
        "Stopping tail is a one-second offline braking check, excluded from active tracking-error metrics."]}
    for name in args.episodes:
        ep=load_episode(name,model)
        episode_out=out/name;episode_out.mkdir()
        hm,ht=historical_metrics(ep,model,episode_out)
        runs={};serialized={}
        for scenario,variants in (("as_recorded",("direct_current","hybrid")),
                                  ("downward_rebased",("direct_current","direct_matched","hybrid"))):
            for variant in variants:
                dest=episode_out/(scenario+"__"+variant);dest.mkdir()
                metrics,trace=replay(ep,scenario,variant,model,dest,settings,args.down_seed_deg)
                runs[(scenario,variant)]=(metrics,trace)
                serialized[scenario+"__"+variant]=metrics
                print(json.dumps({"episode":name,"scenario":scenario,"variant":variant,
                    "completed":metrics["completed"],"error_rms_mm":metrics["position_reference_error_mm"].get("rms"),
                    "max_tilt_deg":metrics["tool_axis_tilt_deg"].get("max"),
                    "max_speed_deg_s":metrics["joint_reference_speed_deg_s"].get("max"),
                    "p95_compute_ms":metrics["cycle_compute_ms"].get("p95"),
                    "failure":metrics["failure"]},ensure_ascii=False),flush=True)
        diagnostics=diagnose_reference_feasibility(runs,model)
        make_plots(ep,runs,ht,episode_out)
        report["episodes"][name]={"data_audit":ep.audit,"historical_metrics":hm,"runs":serialized,
                                  "reference_feasibility_diagnostics":diagnostics}
        (out/"summary.json").write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding="utf-8")
    for p in files:
        if env.sha256(p)!=hashes[str(p)]:raise RuntimeError("A source/production file changed during replay: "+str(p))
    report["source_hashes_unchanged"]=True
    report["network_attempts"]=env.NETWORK_ATTEMPTS
    report["implementation_hashes"]={p.name:env.sha256(p) for p in env.ROOT.glob("*.py")}
    (out/"summary.json").write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding="utf-8")
    print("FINAL_REPORT "+str(out/"summary.json"),flush=True)
if __name__=="__main__":main()
