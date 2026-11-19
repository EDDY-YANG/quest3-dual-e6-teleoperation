//! Local audited ABI around upstream RangedIK/PANOC. No robot or network I/O.
//! Outputs are caller-owned; previews never advance temporal objective history.
use crate::relaxed_ik::RelaxedIK;
use nalgebra::{Quaternion, UnitQuaternion, Vector3, Vector6};
use optimization_engine::{constraints::Rectangle, panoc::{PANOCCache, PANOCOptimizer}, Optimizer, Problem, SolverError};
use std::{cell::RefCell, ffi::{CStr, CString}, os::raw::{c_char,c_int}, panic::{catch_unwind,AssertUnwindSafe}};

thread_local! { static ERROR: RefCell<CString> = RefCell::new(CString::new("").unwrap()); }
fn error(message: impl AsRef<str>) { ERROR.with(|e| *e.borrow_mut() = CString::new(message.as_ref().replace('\0', " ")).unwrap()); }
fn guard<F: FnOnce() -> Result<(), String>>(f: F) -> c_int {
    error("");
    match catch_unwind(AssertUnwindSafe(f)) {
        Ok(Ok(())) => 0,
        Ok(Err(e)) => { error(e); -1 },
        Err(_) => { error("Rust panic caught at RangedIK ABI boundary"); -2 }
    }
}
pub struct RikHandle {
    core: RelaxedIK,
    pending: Option<Vec<f64>>,
    stats: [f64;5],
    geodesic_orientation: bool,
    preview_batch: bool,
    previews: Vec<Preview>,
    next_preview_id: u64,
    solve_budget_ms: f64,
}
#[derive(Clone)]
struct Preview {
    id:u64, q:Vec<f64>, stats:[f64;5], geodesic:bool,
    goal:Vector3<f64>, rotation:UnitQuaternion<f64>, tolerance:Vector6<f64>,
}
fn clear_previews(h:&mut RikHandle) {
    h.pending=None;h.previews.clear();h.preview_batch=false;
}
// Keep a useful center attraction when broad task ranges would otherwise make
// small VR translations cheap to ignore. This documented adapter objective is
// additional to upstream's preferred-range losses; it is not a hard tolerance.
struct PreferredPositionCenter;
impl crate::groove::objective::ObjectiveTrait for PreferredPositionCenter {
    fn call(&self, _x:&[f64], v:&crate::groove::vars::RelaxedIKVars,
        frames:&Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>) -> f64 {
        let p=frames[0].0.last().unwrap();
        let e=v.goal_quats[0].inverse() * (*p-v.goal_positions[0]);
        (0..3).map(|i|(e[i]/v.tolerances[0][i].max(1e-6)).powi(2)).sum()
    }
    fn call_lite(&self,_x:&[f64],v:&crate::groove::vars::RelaxedIKVars,
        poses:&Vec<(Vector3<f64>,UnitQuaternion<f64>)>) -> f64 {
        let e=v.goal_quats[0].inverse()*(poses[0].0-v.goal_positions[0]);
        (0..3).map(|i|(e[i]/v.tolerances[0][i].max(1e-6)).powi(2)).sum()
    }
}
// Optional orientation counterpart. A normalized quadratic maintains attraction
// outside the narrow preferred-range well; it is still a soft objective.
struct PreferredOrientationCenter;
impl crate::groove::objective::ObjectiveTrait for PreferredOrientationCenter {
    fn call(&self, _x:&[f64], v:&crate::groove::vars::RelaxedIKVars,
        frames:&Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>) -> f64 {
        let r=frames[0].1.last().unwrap();
        let e=(v.goal_quats[0].inverse() * *r).scaled_axis();
        (0..3).map(|i|(e[i]/v.tolerances[0][i+3].max(1e-6)).powi(2)).sum()
    }
    fn call_lite(&self,_x:&[f64],v:&crate::groove::vars::RelaxedIKVars,
        poses:&Vec<(Vector3<f64>,UnitQuaternion<f64>)>) -> f64 {
        let e=(v.goal_quats[0].inverse()*poses[0].1).scaled_axis();
        (0..3).map(|i|(e[i]/v.tolerances[0][i+3].max(1e-6)).powi(2)).sum()
    }
}
// One SO(3) angular radius, independent of the axis used to express the error.
// Three equal-weight slots each contribute one third, preserving the legacy
// 11-weight ABI while the effective range weight remains 50 (not 150).
struct GeodesicOrientationRange;
impl GeodesicOrientationRange {
    fn loss(angle:f64, bound:f64)->f64 {
        crate::groove::objective::swamp_groove_loss(
            angle,0.0,-bound,bound,bound*2.0,1.0,0.01,100.0,20)/3.0
    }
}
impl crate::groove::objective::ObjectiveTrait for GeodesicOrientationRange {
    fn call(&self,_x:&[f64],v:&crate::groove::vars::RelaxedIKVars,
        frames:&Vec<(Vec<Vector3<f64>>,Vec<UnitQuaternion<f64>>)>) -> f64 {
        let angle=(v.goal_quats[0].inverse() * *frames[0].1.last().unwrap()).angle();
        Self::loss(angle,v.tolerances[0][3])
    }
    fn call_lite(&self,_x:&[f64],v:&crate::groove::vars::RelaxedIKVars,
        poses:&Vec<(Vector3<f64>,UnitQuaternion<f64>)>) -> f64 {
        Self::loss((v.goal_quats[0].inverse()*poses[0].1).angle(),v.tolerances[0][3])
    }
}
unsafe fn values(ptr: *const f64, n: usize) -> Result<Vec<f64>,String> {
    if ptr.is_null() { return Err("Null input buffer".into()); }
    let out = std::slice::from_raw_parts(ptr,n).to_vec();
    if out.iter().any(|v|!v.is_finite()) { return Err("Non-finite input".into()); }
    Ok(out)
}
unsafe fn handle<'a>(ptr: *mut RikHandle) -> Result<&'a mut RikHandle,String> {
    if ptr.is_null() { Err("Null RangedIK handle".into()) } else { Ok(&mut *ptr) }
}
fn valid_joints(h: &RikHandle, q: &[f64]) -> Result<(),String> {
    for i in 0..6 {
        if q[i] < h.core.vars.robot.lower_joint_limits[i] || q[i] > h.core.vars.robot.upper_joint_limits[i] {
            return Err(format!("Joint {} outside URDF limits",i+1));
        }
    }
    Ok(())
}

#[no_mangle]
pub extern "C" fn rik_last_error() -> *const c_char { ERROR.with(|e| e.borrow().as_ptr()) }

#[no_mangle]
pub unsafe extern "C" fn rik_new(settings: *const c_char) -> *mut RikHandle {
    error("");
    let result=catch_unwind(AssertUnwindSafe(|| -> Result<RikHandle,String> {
        if settings.is_null() { return Err("Absolute settings path required".into()); }
        let path=CStr::from_ptr(settings).to_str().map_err(|e|e.to_string())?;
        if !std::path::Path::new(path).is_absolute() { return Err("Settings path must be absolute".into()); }
        let mut core=RelaxedIK::load_settings(path);
        if core.vars.robot.num_dofs!=6 || core.vars.robot.num_chains!=1 {return Err("This ABI accepts exactly one six-DOF chain".into());}
        // First six pose terms + six joint limits + velocity/acceleration/jerk/manipulability.
        // Remove, rather than zero-weight, unvalidated virtual-link collision terms.
        core.om.objectives.truncate(16);
        core.om.weight_priors.truncate(16);
        core.om.objectives.push(Box::new(PreferredPositionCenter));
        core.om.weight_priors.push(5.0);
        core.om.objectives.push(Box::new(PreferredOrientationCenter));
        core.om.weight_priors.push(0.0); // Older profiles keep their exact objective.
        Ok(RikHandle{core,pending:None,stats:[0.;5],geodesic_orientation:false,
                     preview_batch:false,previews:Vec::new(),next_preview_id:0,solve_budget_ms:80.})
    }));
    match result { Ok(Ok(h))=>Box::into_raw(Box::new(h)), Ok(Err(e))=>{error(e);std::ptr::null_mut()}, Err(_)=>{error("Settings/URDF constructor panic");std::ptr::null_mut()} }
}

#[no_mangle]
pub unsafe extern "C" fn rik_free(ptr: *mut RikHandle) { if !ptr.is_null() {drop(Box::from_raw(ptr));} }

#[no_mangle]
pub unsafe extern "C" fn rik_reset(ptr:*mut RikHandle, q:*const f64)->c_int {
    guard(|| {let h=handle(ptr)?;let q=values(q,6)?;valid_joints(h,&q)?;h.core.reset(q);clear_previews(h);Ok(())})
}

#[no_mangle]
pub unsafe extern "C" fn rik_configure(ptr:*mut RikHandle, weights:*const f64)->c_int {
    guard(|| {
        let h=handle(ptr)?;let w=values(weights,11)?;
        if w.iter().any(|x|*x<0.) {return Err("Negative objective weight".into());}
        for i in 0..6 {h.core.om.weight_priors[i]=w[i];h.core.om.weight_priors[6+i]=w[6];}
        for i in 0..4 {h.core.om.weight_priors[12+i]=w[7+i];}
        h.core.om.weight_priors[16]=(w[0]+w[1]+w[2])/3.0*0.1;
        clear_previews(h);
        Ok(())
    })
}

#[no_mangle]
pub unsafe extern "C" fn rik_set_orientation_center_weight(ptr:*mut RikHandle,weight:f64)->c_int {
    guard(|| {
        if !weight.is_finite() || weight<0. {return Err("Invalid orientation center weight".into());}
        let h=handle(ptr)?;
        h.core.om.weight_priors[17]=weight;
        clear_previews(h);
        Ok(())
    })
}

#[no_mangle]
pub unsafe extern "C" fn rik_set_orientation_error_metric(ptr:*mut RikHandle,mode:c_int)->c_int {
    guard(|| {
        if mode!=0 && mode!=1 {return Err("Unknown orientation metric".into());}
        let h=handle(ptr)?;
        for axis in 0..3 {
            h.core.om.objectives[3+axis]=if mode==1 {
                Box::new(GeodesicOrientationRange)
            } else {
                Box::new(crate::groove::objective::MatchEERotaDoF::new(0,axis))
            };
        }
        h.geodesic_orientation=mode==1;
        clear_previews(h);
        Ok(())
    })
}

#[no_mangle]
pub unsafe extern "C" fn rik_begin_preview_batch(ptr:*mut RikHandle)->c_int {
    guard(|| {let h=handle(ptr)?;clear_previews(h);h.preview_batch=true;Ok(())})
}
#[no_mangle]
pub unsafe extern "C" fn rik_get_preview_id(ptr:*mut RikHandle,out:*mut u64)->c_int {
    guard(|| {
        if out.is_null(){return Err("Null preview ID output".into());}
        let h=handle(ptr)?;
        *out=h.previews.last().ok_or("No successful preview")?.id;
        Ok(())
    })
}
#[no_mangle]
pub unsafe extern "C" fn rik_select_preview(ptr:*mut RikHandle,id:u64)->c_int {
    guard(|| {
        let h=handle(ptr)?;
        if !h.preview_batch{return Err("No active preview batch".into());}
        let selected=h.previews.iter().find(|p|p.id==id && p.geodesic==h.geodesic_orientation)
            .cloned().ok_or("Unknown or stale preview ID")?;
        h.pending=Some(selected.q);h.stats=selected.stats;
        h.core.vars.goal_positions[0]=selected.goal;h.core.vars.goal_quats[0]=selected.rotation;
        h.core.vars.tolerances[0]=selected.tolerance;
        h.previews.clear();h.preview_batch=false;
        Ok(())
    })
}
#[no_mangle]
pub unsafe extern "C" fn rik_set_solve_budget_ms(ptr:*mut RikHandle,budget:f64)->c_int {
    guard(|| {
        if !budget.is_finite() || budget<=0. || budget>80. {return Err("Solve budget must be in (0,80] ms".into());}
        handle(ptr)?.solve_budget_ms=budget;Ok(())
    })
}

#[no_mangle]
pub unsafe extern "C" fn rik_fk(ptr:*mut RikHandle,q:*const f64,out:*mut f64)->c_int {
    guard(|| {
        if out.is_null(){return Err("Null output buffer".into());}
        let h=handle(ptr)?;let q=values(q,6)?;valid_joints(h,&q)?;
        let poses=h.core.vars.robot.get_ee_pos_and_quat_immutable(&q);
        let p=poses[0].0;let r=poses[0].1.quaternion();
        let value=[p.x,p.y,p.z,r.i,r.j,r.k,r.w];
        if value.iter().any(|v|!v.is_finite()){return Err("Non-finite FK".into());}
        std::ptr::copy_nonoverlapping(value.as_ptr(),out,7);Ok(())
    })
}

#[no_mangle]
pub unsafe extern "C" fn rik_stats(ptr:*mut RikHandle,out:*mut f64)->c_int {
    guard(|| {if out.is_null(){return Err("Null output buffer".into());}
        let h=handle(ptr)?;std::ptr::copy_nonoverlapping(h.stats.as_ptr(),out,5);Ok(())})
}

#[no_mangle]
pub unsafe extern "C" fn rik_solve_position(ptr:*mut RikHandle,pos:*const f64,quat:*const f64,tol:*const f64,seed:*const f64,lower:*const f64,upper:*const f64,out:*mut f64)->c_int {
    guard(|| {
        if out.is_null(){return Err("Null output buffer".into());}
        let h=handle(ptr)?;h.pending=None;
        if !h.preview_batch {h.previews.clear();}
        if h.previews.len()>=32{return Err("Preview batch limit exceeded".into());}
        let p=values(pos,3)?;let r=values(quat,4)?;let t=values(tol,6)?;
        let mut q=values(seed,6)?;let mut lo=values(lower,6)?;let mut hi=values(upper,6)?;
        if t.iter().any(|v|*v<0.){return Err("Negative tolerance".into());}
        if h.geodesic_orientation && (t[3]<=0. || t[3]>=std::f64::consts::PI ||
            (t[3]-t[4]).abs()>1e-12 || (t[3]-t[5]).abs()>1e-12) {
            return Err("Geodesic mode requires one positive angle radius replicated in ABI".into());
        }
        let norm2=r.iter().map(|x|x*x).sum::<f64>();
        if (norm2-1.).abs()>1e-3{return Err("Quaternion must be unit xyzw".into());}
        for i in 0..6 {
            lo[i]=lo[i].max(h.core.vars.robot.lower_joint_limits[i]);
            hi[i]=hi[i].min(h.core.vars.robot.upper_joint_limits[i]);
            if lo[i]>=hi[i] || q[i]<lo[i] || q[i]>hi[i] {return Err(format!("Infeasible bounds or seed at joint {}",i+1));}
        }
        h.core.vars.goal_positions[0]=Vector3::new(p[0],p[1],p[2]);
        h.core.vars.goal_quats[0]=UnitQuaternion::from_quaternion(Quaternion::new(r[3],r[0],r[1],r[2]));
        h.core.vars.tolerances[0]=Vector6::new(t[0],t[1],t[2],t[3],t[4],t[5]);
        let vars=&h.core.vars;let om=&h.core.om;
        let df=|u:&[f64],g:&mut[f64]|->Result<(),SolverError>{let(_,v)=om.gradient(u,vars);g.copy_from_slice(&v);Ok(())};
        let cost=|u:&[f64],c:&mut f64|->Result<(),SolverError>{*c=om.call(u,vars);Ok(())};
        let bounds=Rectangle::new(Some(lo.as_slice()),Some(hi.as_slice()));
        let problem=Problem::new(&bounds,df,cost);
        // A fresh cache makes rejected previews unable to alter the next preview.
        let mut cache=PANOCCache::new(6,1e-14,10);
        let mut solver=PANOCOptimizer::new(problem,&mut cache).with_max_iter(500)
            .with_max_duration(std::time::Duration::from_secs_f64(h.solve_budget_ms/1000.)).with_tolerance(0.000001);
        let status=solver.solve(&mut q).map_err(|e|format!("PANOC failure: {:?}",e))?;
        h.stats=[if status.has_converged(){1.}else{0.},status.iterations() as f64,
            status.solve_time().as_secs_f64()*1000.,status.norm_fpr(),status.cost_value()];
        if q.iter().enumerate().any(|(i,v)|!v.is_finite()||*v<lo[i]-1e-9||*v>hi[i]+1e-9){return Err("Invalid/out-of-bounds PANOC output".into());}
        std::ptr::copy_nonoverlapping(q.as_ptr(),out,6);
        h.next_preview_id=h.next_preview_id.checked_add(1).ok_or("Preview ID exhausted")?;
        h.previews.push(Preview{id:h.next_preview_id,q:q.clone(),stats:h.stats,geodesic:h.geodesic_orientation,
            goal:h.core.vars.goal_positions[0],rotation:h.core.vars.goal_quats[0],tolerance:h.core.vars.tolerances[0]});
        h.pending=Some(q);Ok(())
    })
}

#[no_mangle]
pub unsafe extern "C" fn rik_commit(ptr:*mut RikHandle,q:*const f64)->c_int {
    guard(|| {
        let h=handle(ptr)?;let q=values(q,6)?;valid_joints(h,&q)?;
        if h.preview_batch{return Err("Select a preview before committing a batch".into());}
        let candidate=h.pending.as_ref().ok_or_else(||"No preview candidate to commit".to_string())?;
        if q.iter().zip(candidate).any(|(a,b)|(a-b).abs()>1e-9){return Err("Commit differs from preview candidate".into());}
        h.core.vars.update(q);clear_previews(h);Ok(())
    })
}
