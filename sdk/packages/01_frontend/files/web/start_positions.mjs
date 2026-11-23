// Presentation only. The server independently owns every movement guard.
const STATES={NO_START:"未保存开始位",PRESET:"关节预设已保存",SAVED:"实测开始位已保存",CAPTURING:"正在记录实测姿态",
 COLLISION_PROTECTION:"碰撞保护 · 可恢复回位",RECOVERING:"正在清除本臂碰撞保护",
 PREPARING_RETURN:"准备回位",ENABLING:"正在使能本臂",RETURNING:"正在回位",AT_START:"已实测到位",STOPPING:"正在停止",
 STOPPED:"已停止",ERROR:"回位/保存未完成",FAULT:"停止未确认",ALARMS_CLEARED:"报警已清除，请确认后回位"};
export function startPositionView(remote={},fresh=false){
 const arms={};
 for(const side of ["left","right"]){
  const a=remote.arms?.[side] || {},saved=a.saved,target=a.target || saved;
  const configured=a.target_source==="CONFIGURED_JOINT_TARGET";
  const summary=target?`${configured?`预设目标 ${target.label || ""}`:"实测保存 "+(saved?.saved_at || "")} · J1–J6 ${target.q_deg.map(x=>x.toFixed(1)).join(" / ")}°${configured?" · 回位前由控制器核对末端位姿":""}`:"连接反馈后保存实测开始位，或加载已配置的关节预设。";
  arms[side]={...a,label:fresh?(STATES[a.state] || "未保存开始位"):"状态未更新",
   canCapture:fresh && a.can_capture===true,canReturn:fresh && a.can_return===true,
   summary};
 }
 return {arms,busy:fresh && remote.busy===true,activeSide:remote.active_side,
  fault:fresh && remote.fault_latched===true,operationId:remote.operation_id,
  speed:remote.settings?.speed_percent ?? null};
}
