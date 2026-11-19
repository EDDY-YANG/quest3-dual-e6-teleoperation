"""Reference planner and constrained DLS. Pure kinematics; no robot driver."""
import offline_runtime as env
from dataclasses import dataclass,replace
import math,time,warnings
import numpy as np
from scipy.optimize import minimize
from controller_kinematics import PoseModel,pose_matrix,rotation_log
from ranged_ik_profile import RangedIKProfile
from ranged_ik_adapter import RangedIKAdapter

DOWN=np.array([0.,0.,-1.])
def skew(v):
    x,y,z=v
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
def tilt_deg(model,q):
    axis=model._forward(np.degrees(q))[:3,2]
    return math.degrees(math.acos(float(np.clip(DOWN@axis,-1,1))))
def jacobian_si(model,q):
    j=model.jacobian(np.degrees(q))*180/math.pi
    j[:3]/=1000
    return j
def cone_value_gradient(model,q):
    transform,j_deg=model._forward(np.degrees(q),with_jacobian=True)
    axis=transform[:3,2]
    j=j_deg[3:]*(180/math.pi)
    return float(DOWN@axis),DOWN@(-skew(axis))@j
def stopping_displacement_jacobian(velocity,dt,amax):
    index=int(np.argmax(np.abs(velocity)));peak=float(abs(velocity[index]))
    factor=dt+peak/(2*amax)
    derivative=factor*np.eye(6)
    derivative[:,index]+=velocity*np.sign(velocity[index])/(2*amax)
    return factor*velocity,derivative
def certify_down_segment(model,q0,q1,limit_deg=5.0):
    """Conservative bound for this mathematical joint-linear segment only.

    Tool-axis angular path length is <= sum(abs(delta joint angles)).
    Nearest-grid-point angular distance <= that sum/(2*N).
    Does not certify E6 firmware's actual interpolation/dynamics.
    """
    delta=q1-q0
    length=math.degrees(float(np.sum(np.abs(delta))))
    for n in (1,2,4,8,16,32,64):
        tilts=[tilt_deg(model,q0+float(s)*delta) for s in np.linspace(0,1,n+1)]
        peak=max(tilts)
        if peak>limit_deg+1e-7:return False,peak,n
        upper=peak+length/(2*n)
        if upper<=limit_deg+1e-7:return True,upper,n
    return False,upper,n

@dataclass(frozen=True)
class Settings:
    down_constraint_strategy:str="CERTIFIED_UNIFORM_BRAKING_ENVELOPE"
    execution_hz:float=50.
    planner_hz:float=10.
    max_joint_velocity_deg_s:float=60.
    max_joint_acceleration_deg_s2:float=120.
    max_command_delta_deg:float=50.
    down_limit_deg:float=5.
    solver_down_limit_deg:float=4.5
    position_gain:float=15.
    orientation_gain:float=15.
    posture_gain:float=.4
    characteristic_length_m:float=.1
    max_cartesian_velocity_m_s:float=.3
    max_angular_velocity_rad_s:float=1.5
    damping_floor:float=.002
    damping_max:float=.05
    damping_sigma_width:float=.05
    input_timeout_s:float=.2
    reference_timeout_s:float=.3

@dataclass
class Reference:
    q:np.ndarray
    transform:np.ndarray
    generated_at:float
    status:str
    generation:int

class ReferencePlanner:
    def __init__(self,model,q0,output,down_aware=False):
        self.model=model
        base=RangedIKProfile.load(env.PROFILE)
        self.profile=replace(base,rotation_tolerance_deg=(3.,3.,30.),
                    profile_id=base.profile_id+"__offline_down_3_3_30") if down_aware else base
        self.adapter=RangedIKAdapter(model,self.profile,np.degrees(q0).tolist(),output)
        self.plan_history=[]
        self.last=None
        self.down_aware=down_aware
    def solve(self,goal_pose,q,now):
        started=time.perf_counter()
        result=self.adapter.propose(goal_pose,np.degrees(q).tolist())
        candidate=np.radians(result["q_deg"])
        t=self.model._forward(result["q_deg"])
        invalid=self.down_aware and tilt_deg(self.model,candidate)>5.0+1e-7
        info={"elapsed_ms":(time.perf_counter()-started)*1000,
              "status":result["status"],"native":result["native_solver"],
              "reference_down_rejected":invalid}
        if invalid:return None,result,info
        ref=Reference(candidate,t,now,result["status"],result["generation"])
        return ref,result,info
    def commit_reference(self,ref,result):
        # This is planning history, NEVER an acknowledgement of physical execution.
        self.adapter.commit(result)
        self.last=ref
        self.plan_history.append({"generated_at":ref.generated_at,"q_ref_deg":np.degrees(ref.q).tolist(),
                                  "contract":"PLANNING_REFERENCE_ONLY_NOT_ROBOT_ACK"})
    def close(self):
        self.adapter.close()

class ControllerInfeasible(RuntimeError):
    pass

class ConstrainedDLS:
    def __init__(self,model,q0,joint_limits_deg,settings=Settings(),enforce_down=False):
        self.model=model;self.q=np.array(q0,float);self.velocity=np.zeros(6)
        self.limits=np.radians(np.array(joint_limits_deg,float))
        self.settings=settings;self.enforce_down=enforce_down
        self.events=[]
        if enforce_down and tilt_deg(model,self.q)>settings.down_limit_deg:
            raise ValueError("Downward replay requires a valid initial pose; no instantaneous alignment")
    def bounds(self,dt):
        cfg=self.settings
        vmax=math.radians(cfg.max_joint_velocity_deg_s)
        amax=math.radians(cfg.max_joint_acceleration_deg_s2)
        lower=np.maximum(-vmax,self.velocity-amax*dt)
        upper=np.minimum(vmax,self.velocity+amax*dt)
        lower=np.maximum(lower,(self.limits[:,0]-self.q)/dt)
        upper=np.minimum(upper,(self.limits[:,1]-self.q)/dt)
        # Conservative braking envelopes near absolute joint limits.
        distance_lo=np.maximum(self.q-self.limits[:,0],0)
        distance_hi=np.maximum(self.limits[:,1]-self.q,0)
        neg=np.maximum(0,np.sqrt(2*amax*distance_lo)-amax*dt)
        pos=np.maximum(0,np.sqrt(2*amax*distance_hi)-amax*dt)
        lower=np.maximum(lower,-neg);upper=np.minimum(upper,pos)
        if np.any(lower>upper+1e-10):raise ControllerInfeasible("Velocity/acceleration/position bounds conflict")
        return lower,upper
    def step(self,reference,dt,active=True):
        if not 0<dt<=.1:raise ValueError("Actual step duration must be in (0,0.1] s")
        started=time.perf_counter();cfg=self.settings
        q0=self.q.copy();v0=self.velocity.copy()
        low,high=self.bounds(dt)
        current=self.model._forward(np.degrees(q0))
        j=jacobian_si(self.model,q0)
        a=np.vstack((j[:3]/cfg.characteristic_length_m,j[3:]))
        # Identical metric for damping and task cost; rotations receive explicit weight.
        weights=np.array([1.,1.,1.,.6,.6,.6]);a=weights[:,None]*a
        singular=np.linalg.svd(a,compute_uv=False)
        lam=cfg.damping_floor+cfg.damping_max*math.exp(-.5*(singular[-1]/cfg.damping_sigma_width)**2)
        if active and reference is not None:
            linear=cfg.position_gain*(reference.transform[:3,3]-current[:3,3])/1000
            angular=cfg.orientation_gain*rotation_log(reference.transform[:3,:3]@current[:3,:3].T)
            for vector,limit in ((linear,cfg.max_cartesian_velocity_m_s),(angular,cfg.max_angular_velocity_rad_s)):
                norm=np.linalg.norm(vector)
                if norm>limit:vector*=limit/norm
            target=weights*np.r_[linear/cfg.characteristic_length_m,angular]
            posture=cfg.posture_gain*(reference.q-q0)
        else:
            target=np.zeros(6);posture=np.zeros(6)
        h=a.T@a+lam*lam*np.eye(6)
        g=a.T@target+lam*lam*posture
        def objective(v):return .5*float(v@h@v)-float(g@v)
        def gradient(v):return h@v-g
        constraints=[]
        if self.enforce_down:
            # Reserve a feasible uniform joint-rate braking trajectory. All joint
            # rates decay by the same scale, at <= the per-joint acceleration cap.
            # Its path is joint-linear, so the same whole-segment tilt certificate
            # can validate both the next command and a subsequent stop.
            coslimit=math.cos(math.radians(cfg.solver_down_limit_deg))
            amax=math.radians(cfg.max_joint_acceleration_deg_s2)
            cached={}
            def stopping_map(v):
                return stopping_displacement_jacobian(v,dt,amax)
            def braking_constraints(v):
                if "v" in cached and np.array_equal(cached["v"],v):
                    return cached["g"],cached["jac"]
                displacement,derivative=stopping_map(v)
                vals=[];grads=[]
                for fraction in (1/3,2/3,1.):
                    value,grad=cone_value_gradient(self.model,q0+fraction*displacement)
                    vals.append(value-coslimit);grads.append(grad@(fraction*derivative))
                stop=q0+displacement
                vals.extend(stop-self.limits[:,0]);grads.extend(derivative)
                vals.extend(self.limits[:,1]-stop);grads.extend(-derivative)
                cached.update(v=v.copy(),g=np.array(vals),jac=np.array(grads))
                return cached["g"],cached["jac"]
            constraints.append({"type":"ineq","fun":lambda v:braking_constraints(v)[0],
                                "jac":lambda v:braking_constraints(v)[1]})
        initial=np.clip(v0,low,high)
        with warnings.catch_warnings(record=True) as numerical_warnings:
            warnings.simplefilter("always")
            result=minimize(objective,initial,jac=gradient,bounds=list(zip(low,high)),
                            constraints=constraints,method="SLSQP",
                            options={"maxiter":60,"ftol":1e-10,"disp":False})
        velocity=result.x
        status="TRACKING" if active else "BRAKING_OR_HOLD"
        failure=None
        def validate(v):
            q1=q0+dt*v
            if np.any(v<low-1e-7) or np.any(v>high+1e-7) or not np.all(np.isfinite(v)):
                return False,float("nan"),0
            if np.max(np.abs(np.degrees(q1-q0)))>cfg.max_command_delta_deg+1e-7:
                return False,float("nan"),0
            if self.enforce_down:
                displacement,_=stopping_map(v)
                stop=q0+displacement
                if np.any(stop<self.limits[:,0]-1e-7) or np.any(stop>self.limits[:,1]+1e-7):
                    return False,float("nan"),0
                return certify_down_segment(self.model,q0,stop,cfg.down_limit_deg)
            return True,tilt_deg(self.model,q1),0
        valid,upper,n=validate(velocity)
        if not result.success or not valid:
            failure=str(result.message) if not result.success else "DOWN_SEGMENT_NOT_CERTIFIED"
            peak=float(np.max(np.abs(v0)))
            brake_scale=max(0.,1-math.radians(cfg.max_joint_acceleration_deg_s2)*dt/peak) if peak>0 else 0.
            velocity=v0*brake_scale
            valid,upper,n=validate(velocity)
            status="CONSTRAINED_BRAKING_FALLBACK"
            self.events.append({"reason":failure,"braking_valid":valid})
            if not valid:
                raise ControllerInfeasible("No certified braking step: "+failure)
        self.q=q0+dt*velocity
        self.velocity=velocity
        accel=(velocity-v0)/dt
        if np.max(abs(velocity))>math.radians(cfg.max_joint_velocity_deg_s)+1e-7:
            raise AssertionError("Velocity constraint violated")
        if np.max(abs(accel))>math.radians(cfg.max_joint_acceleration_deg_s2)+1e-6:
            raise AssertionError("Acceleration constraint violated")
        return self.q.copy(),{"status":status,"fallback_reason":failure,"solver_success":bool(result.success),
            "numerical_warning_count":len(numerical_warnings),
            "iterations":int(result.nit),"elapsed_ms":(time.perf_counter()-started)*1000,
            "lambda":lam,"sigma_min":float(singular[-1]),"velocity_deg_s":np.degrees(velocity).tolist(),
            "acceleration_deg_s2":np.degrees(accel).tolist(),"nominal_segment_tilt_upper_deg":upper,
            "segment_subdivisions":n}

