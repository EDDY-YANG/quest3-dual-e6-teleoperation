import {E6SideModels} from "./e6_urdf_view.mjs";
import {ControllerModels} from "./controller_model_view.mjs";
import {armState,handState,renderView,formatNumber as f} from "./pc_monitor_state.mjs";
import {startPositionView} from "./start_positions.mjs";
const $=s=>document.querySelector(s), sides=["left","right"], names={left:"左",right:"右"};
const states={};let data=null,lastReceived=-Infinity,busy=false,stopBusy=false,lastPaint=-Infinity,polling=false;
const changes=new Map(),events=[];
const homeClient=crypto.randomUUID();let ownHomeOperation=null,heartbeatPending=false;
function element(tag,text,className){const n=document.createElement(tag);if(text!==undefined)n.textContent=text;if(className)n.className=className;return n;}
function metric(grid,label){const c=element("div",undefined,"metric");c.append(element("label",label));const v=element("strong","—");c.append(v);grid.append(c);return v;}
for(const side of sides){
 const arm=element("article",undefined,"card"),head=element("div",undefined,"card-head");
 head.append(element("h2",`${names[side]}臂 E6`));const badge=element("span","等待反馈","badge");head.append(badge);arm.append(head);
 const content=element("div",undefined,"arm-content"),canvas=element("canvas",undefined,"arm-model");canvas.setAttribute("aria-label",`${names[side]}臂官方E6实时模型`);content.append(canvas);
 const detail=element("div",undefined,"arm-data"),table=element("table"),thead=element("thead"),tr=element("tr");
 for(const label of ["关节","实测 / °","已接受指令 / °","误差 / °"])tr.append(element("th",label));thead.append(tr);table.append(thead);const tbody=element("tbody"),joint=[];
 for(let i=0;i<6;i++){const row=element("tr");row.append(element("td",`J${i+1}`));const values=Array.from({length:3},(_,j)=>element("td","—",j===0?"joint-actual":""));row.append(...values);tbody.append(row);joint.push(values);}table.append(tbody);detail.append(table);
 const grid=element("div",undefined,"pose-grid"),pose=["X / mm","Y / mm","Z / mm","Rx / °","Ry / °","Rz / °"].map(x=>metric(grid,x));detail.append(grid);const frame=element("p","User / Tool —","detail-line");detail.append(frame);content.append(detail);arm.append(content);
 const status=element("div",undefined,"robot-status"),mode=element("span","模式 —"),rate=element("span","反馈 — Hz"),age=element("span","数据年龄 — ms"),owner=element("span","未连接");status.append(mode,rate,age,owner);arm.append(status);const address=element("p","设备/源网口等待核对","detail-line");arm.append(address);$("#arm-grid").append(arm);
 const hand=element("article",undefined,"card"),hh=element("div",undefined,"card-head"),hb=element("span","等待追踪","badge");hh.append(element("h2",`${names[side]}手柄实时状态`),hb);hand.append(hh);
 const hc=element("div",undefined,"hand-content"),hcanvas=element("canvas",undefined,"controller-model");hcanvas.setAttribute("aria-label",`${names[side]}Touch Plus设备模型`);hc.append(hcanvas);const hm=element("div",undefined,"hand-metrics"),ht=element("table",undefined,"hand-pose"),hv=[];
 for(const label of ["世界 X / m","世界 Y / m","世界 Z / m","四元数 x","四元数 y","四元数 z","四元数 w"]){const row=element("tr"),value=element("td","—");row.append(element("td",label),value);ht.append(row);hv.push(value);}hm.append(ht);
 const gauges=element("div",undefined,"gauges"),gv=[];
 for(const label of ["Grip","Trigger","摇杆 X","摇杆 Y"]){const row=element("div",undefined,"gauge"),track=element("div",undefined,"track"),fill=element("div",undefined,"fill"),value=element("span","—");track.append(fill);row.append(element("span",label),track,value);gauges.append(row);gv.push({fill,value});}
 const gb=element("div",undefined,"button-badges");gauges.append(gb);hm.append(gauges);hc.append(hm);hand.append(hc);const handInfo=element("p","频率 — · 数据年龄 —","detail-line"),profile=element("p","Profile —","detail-line");hand.append(handInfo,profile);$("#hand-grid").append(hand);
 states[side]={arm,badge,canvas,joint,pose,frame,mode,rate,age,owner,address,hand,hb,hcanvas,hv,gv,gb,handInfo,profile};
}
const models=new E6SideModels({leftCanvas:states.left.canvas,rightCanvas:states.right.canvas});
const controllers=new ControllerModels(sides.map(s=>states[s].hcanvas));
Promise.all([models.loading,controllers.loading]).then(()=>{$("#asset-status").textContent=`E6：${models.ready?"沿用官方 URDF/STL，已加载":models.error} · 手柄：${controllers.ready?"WebXR Touch Plus 左右设备模型，已加载":controllers.error}。模型不代替实测数据。`;});
function badge(node,text,kind=""){node.textContent=text;node.className=`badge ${kind}`;}
function changed(key,value,label){if(changes.get(key)===value)return;changes.set(key,value);events.unshift(`${new Date().toLocaleTimeString()} · ${label}：${value}`);events.splice(12);const list=$("#events");list.replaceChildren(...events.map(x=>element("li",x)));}
function paint(now){
 requestAnimationFrame(paint);if(now-lastPaint<50)return;lastPaint=now;
 const fresh=Boolean(data && now-lastReceived<1500),view=renderView(data,fresh);
 models.paint(view,now);
 const quest=data?.quest || {}, handViews=sides.map(side=>handState(quest,side,fresh));controllers.paint(handViews,now);
 badge($("#pc-link"),fresh?"PC 服务在线":"PC 状态未更新",fresh?"good":"bad");
 badge($("#quest-link"),fresh && quest.connected?`Quest · ${quest.status}`:"Quest 未连接",fresh && quest.status==="LIVE"?"good":"warn");
 const rec=data?.recording;badge($("#recording-link"),fresh?`记录 ${rec?.state || "—"}`:"记录状态 —",rec?.writer_error?"bad":rec?.active?"good":"");
 sides.forEach((side,i)=>{
  const dom=states[side],robot=data?.robots?.[side] || {},control=data?.controls?.[side] || {},a=armState(robot,fresh),h=handViews[i];
  dom.arm.classList.toggle("offline",!a.poseFresh);badge(dom.badge,a.label,a.collision?"bad":a.poseFresh?"good":"warn");
  const command=fresh && control.armed && control.state==="FOLLOWING"?control.target?.joint_position_deg:null;
  dom.joint.forEach((cells,j)=>{cells[0].textContent=f(a.q?.[j]);cells[1].textContent=f(command?.[j]);cells[2].textContent=a.q && command?f(Math.abs(command[j]-a.q[j])):"—";});
  dom.pose.forEach((v,j)=>v.textContent=f(a.pose?.[j],j<3?2:1));
  dom.frame.textContent=a.packetFresh?`User ${robot.user ?? "—"} · Tool ${robot.tool ?? "—"} · 反馈校验：${a.controlValid?"通过":"未通过"}`:"User / Tool —";
  dom.mode.textContent=`模式 ${a.mode}`;dom.rate.textContent=`反馈 ${fresh?f(robot.receive_hz,1):"—"} Hz`;dom.age.textContent=`数据年龄 ${fresh?f(robot.feedback_age_ms,0):"—"} ms`;
  dom.owner.textContent=robot.monitor_owner==="control"?"复用控制链反馈":robot.monitor_owner==="passive_monitor"?"独立只读反馈":"未请求连接";
  dom.address.textContent=fresh?`${robot.ip || "—"} ← ${robot.local_ip || "—"} · ${robot.reason || "等待反馈"}`:"设备状态未知";
  badge(dom.hb,h.label,h.tracked?"good":"warn");dom.hand.classList.toggle("offline",!h.tracked);
  [...(h.position || Array(3).fill(null)),...(h.quaternion || Array(4).fill(null))].forEach((v,j)=>dom.hv[j].textContent=f(v,4));
  const values=[h.buttons[1]?.value,h.buttons[0]?.value,h.axes[2],h.axes[3]];
  dom.gv.forEach((g,j)=>{const v=values[j];g.value.textContent=f(v,2);g.fill.style.width=Number.isFinite(v)?`${Math.max(0,Math.min(1,j<2?v:(v+1)/2))*100}%`:"0%";});
  dom.gb.replaceChildren(...h.buttons.map((b,j)=>{const labels=["Trigger","Grip","触控板","摇杆按下",side==="left"?"X":"A",side==="left"?"Y":"B"];const n=element("span",`${labels[j] || `按键${j}`} ${f(b.value,2)}${b.touched?" · 触摸":""}`,b.pressed?"pressed":"");return n;}));
  dom.handInfo.textContent=`输入 ${fresh?f(quest.receive_hz,1):"—"} Hz · 数据年龄 ${fresh?f(quest.age_ms,0):"—"} ms · Grip ${h.gripHeld===null?"—":h.gripHeld?"握持":"释放"}`;
  dom.profile.textContent=`${h.profiles.join(" / ") || "Profile —"} · ${h.referenceSpace || "—"} / epoch ${h.referenceEpoch ?? "—"}`;
  if(fresh)changed(side,a.label,`${names[side]}臂`);
 });
 const selectedSide=data?.teleop_side==="right"?"right":"left",control=data?.controls?.[selectedSide] || {},limits=control.limits || {};
 const home=startPositionView(data?.start_positions,fresh);
 for(const side of sides){
  const h=home.arms[side];badge($(`#home-${side}-state`),h.label,h.state==="FAULT"?"bad":h.state==="AT_START"?"good":"");
  $(`#home-${side}-detail`).textContent=h.summary;
  $(`#home-${side}-reason`).textContent=fresh?(h.blocked_reason || h.reason || ""):"状态未更新，回位不可用";
  $(`#home-${side}-capture`).disabled=busy || !h.canCapture || control.armed===true;
  const homeButton=$(`#home-${side}-return`);
  homeButton.disabled=false;
  homeButton.textContent=h.clear_only ? `${names[side]}臂回开始位 · 仅清警` : `${names[side]}臂回开始位`;
 }
 if(ownHomeOperation && fresh && (!home.busy || home.operationId!==ownHomeOperation))ownHomeOperation=null;
 $("#control-status").textContent=fresh?(data.group?.participants?.length===2?
  `双臂遥操作 · 统一状态 ${data.group.state} · `+sides.map(s=>`${names[s]}臂 ${data.controls?.[s]?.state || "未准备"}`).join("；"):
  `单臂遥操作：${names[selectedSide]}臂/${names[selectedSide]}手 · ${control.state || "未准备"} · ${control.reason || "无动作请求"}；另一侧可离线，回位无需方向标定。`):"控制状态未更新";
 $("#runtime-limits").textContent=fresh && limits.control_mode?`运行模式 ${limits.control_mode} · 新指令角差 ${f(limits.reference_max_tracking_error_deg,0)}° · 有效搜索 ${limits.effective_ik_search_radius_deg?.join(" / ") || "—"}°` :"运行参数等待控制链准备；不把设计值当作已生效参数。";
 const cone=control.tool_orientation_cone;
 if(fresh && cone?.enabled)$("#runtime-limits").textContent+=` · 向下倾角 ${f(cone.measured_tilt_deg,1)}° / ${f(cone.limit_deg,0)}° · 首次开始≤${f(cone.initial_limit_deg,0)}°（${cone.initial_ready?"就绪":"未满足"}）`;
 else if(fresh && data.configured_tool_orientation_cone){
  const planned=data.configured_tool_orientation_cone;
  $("#runtime-limits").textContent+=` 已加载RangedIK倾角配置：朝下≤${f(planned.limit_deg,0)}°，初始≤${f(planned.initial_limit_deg,0)}°；实测初始位姿尚未检查。`;
 }
 const rotation=fresh?(control.ranged_ik || data.configured_ranged_ik):null;
 if(rotation?.orientation_target_policy==="GRIP_ANCHORED_WORLD"){
  const tolerance=rotation.orientation_error_metric==="GEODESIC_ANGLE"?
   `整体朝向误差≤${f(rotation.orientation_tolerance_deg,0)}°`:`旋转分量容差 ${rotation.rotation_vector_tolerance_deg?.join(" / ")}°`;
  $("#runtime-limits").textContent+=` · 姿态目标保持1:1 · ${tolerance} · 旋转权重 ${rotation.objective_weights?.rotation?.join(" / ")} · 中心权重 ${f(rotation.objective_weights?.orientation_center,1)}`;
  if(rotation.seed_strategy==="VALIDATED_MULTI_START")$("#runtime-limits").textContent+=" · 多初值精修已配置";
  if(control.orientation_tracking)$("#runtime-limits").textContent+=` · 当前姿态误差 ${f(control.orientation_tracking.error_angle_deg,2)}°`;
  if(control.orientation_tracking?.within_orientation_tolerance!==undefined && control.orientation_tracking?.within_orientation_tolerance!==null)
   $("#runtime-limits").textContent+=control.orientation_tracking.within_orientation_tolerance?"（已进入姿态容差）":"（尚未进入姿态容差）";
 }
 const warning=fresh?control.tracking_warning:null;$("#tracking-warning").textContent=warning?.active?warning.message:"";$("#tracking-warning").classList.toggle("active",Boolean(warning?.active));
 const maps=data?.direction_calibrations || {},operator=data?.operator_frame;
 $("#calibration-status").textContent=fresh?`只采X，Y/Z自动补齐 · 操作者前方${operator?.current?"已记录":"未记录"}；`+sides.map(side=>{
  const map=maps[side] || {};return `${names[side]}臂 ${map.control_ready?"映射就绪":map.state || "待标定"}，前方角 ${f(map.operator_forward_yaw_deg,1)}°${map.last_error?"；失败："+map.last_error:""}`;
 }).join("；"):"方向标定状态未更新";
 $("#lan-info").textContent=fresh && data.lan?.enabled?`Quest 页面 ${data.lan.url}/ · 配对码 ${data.lan.pair_code} · 剩余 ${data.lan.code_remaining_seconds}s`:"Quest LAN 页面未启用；使用原连接台设置。";
 $("#stop-left").disabled=stopBusy || (sides.every(s=>!data?.controls?.[s]?.state || data.controls[s].state==="NOT_PREPARED") && !data?.start_positions?.busy && !data?.start_positions?.fault_latched);
}
async function homeAction(side,actionName){
 if(actionName==="return"){
  const fresh=performance.now()-lastReceived<1000;
  const h=startPositionView(data?.start_positions,fresh).arms[side];
  if(!fresh || busy || (!h.canReturn && !h.can_clear_alarms)){
   $("#action-message").textContent=!fresh?"PC 状态已过期，请等待实时反馈":busy?"上一项操作正在进行":h.blocked_reason || "当前无法清警或回位，请检查控制器状态";
   return;
  }
  if(h.clear_only)actionName="clear_alarms";
 }
 if(busy)return;busy=true;$("#action-message").textContent=actionName==="capture"?"正在核对并保存静止实测姿态…":"回位请求已发送，请保持本页可见…";
 try{
  const response=await fetch('/api/start-positions/action',{method:'POST',headers:{'Content-Type':'application/json'},
   body:JSON.stringify({side,action:actionName,client_id:homeClient}),signal:AbortSignal.timeout(12000)});
  const body=await response.json();if(!response.ok)throw Error(body.error || `HTTP ${response.status}`);
  if(actionName!=='capture')ownHomeOperation=body.operation_id;
  $("#action-message").textContent=actionName==='capture'?"该臂开始位已保存为实测姿态。":actionName==="clear_alarms"?"仅清警请求已受理，请等待复核后再点回位":"回位已受理，请查看进度与实测到位状态；STOP可随时停止。";
 }catch(error){$("#action-message").textContent=`操作未完成：${error.message}`;}finally{busy=false;poll();}
}
for(const side of sides){$(`#home-${side}-capture`).onclick=()=>homeAction(side,'capture');$(`#home-${side}-return`).onclick=()=>homeAction(side,'return');}
setInterval(async()=>{
 if(!ownHomeOperation || document.hidden || heartbeatPending)return;heartbeatPending=true;
 try{await fetch('/api/start-positions/heartbeat',{method:'POST',headers:{'Content-Type':'application/json'},
  body:JSON.stringify({operation_id:ownHomeOperation,client_id:homeClient}),signal:AbortSignal.timeout(800)});}catch{}finally{heartbeatPending=false;}
},400);
async function poll(){
 if(polling)return;polling=true;
 try{const response=await fetch("/api/monitor/status",{cache:"no-store",signal:AbortSignal.timeout(1200)});if(!response.ok)throw Error(`HTTP ${response.status}`);data=await response.json();lastReceived=performance.now();$("#updated").textContent=`更新 ${new Date().toLocaleTimeString()}`;}
 catch(e){lastReceived=-Infinity;$("#updated").textContent=`读取失败：${e.message}`;}
 finally{polling=false;}
}
async function action(path){const isStop=path.includes("/stop");if((busy&&!isStop)||(isStop&&stopBusy))return;if(isStop)stopBusy=true;else busy=true;$("#action-message").textContent="正在处理…";
 try{const r=await fetch(path,{method:"POST",signal:AbortSignal.timeout(15000)}),body=await r.json();if(!r.ok)throw Error(body.error || `HTTP ${r.status}`);
  $("#action-message").textContent=path.includes("/stop")?`停止请求已处理：回位${body.start_positions?.fault_latched?"停止未确认，请检查机器人":"任务已停止/无活动任务"}；左臂控制 ${body.state}`:path.includes("disconnect")?"独立监控已关闭；已有控制链反馈保持不变。":"已请求只读反馈连接；请以两台各自的实时状态为准。";
 }catch(e){$("#action-message").textContent=`操作未完成：${e.message}`;}finally{if(isStop)stopBusy=false;else busy=false;poll();}}
$("#connect-monitors").onclick=()=>action("/api/monitor/connect?side=both");
$("#disconnect-monitors").onclick=()=>action("/api/monitor/disconnect?side=both");
$("#stop-left").onclick=()=>action("/api/left-trial/stop");
document.addEventListener("visibilitychange",()=>{if(document.hidden)lastReceived=-Infinity;else poll();});
setInterval(()=>{if(!document.hidden)poll();},50);poll();requestAnimationFrame(paint);
