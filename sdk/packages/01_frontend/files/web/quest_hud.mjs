// In-scene WebXR UI, also rendered by the read-only PC mirror. No robot SDK.
import {calibrationMessage, calibrationView} from "./direction_calibration.mjs";
import {E6SideModels, SIDE_MODEL_WIDTH, SIDE_MODEL_HEIGHT, preloadE6Assets} from "./e6_urdf_view.mjs";
export const preloadRobotVisual = preloadE6Assets;
import {TrackingAnnouncer, trackingWarningView} from "./tracking_warning.mjs";
import {startPositionView} from "./start_positions.mjs";
export const HUD_WIDTH = 1280, HUD_HEIGHT = 800;
const BUTTONS = [
  {id: "prepare", label: "连接 / 重检", x: 36, y: 658, w: 244, h: 76},
  {id: "arm", label: "开始遥操作", x: 296, y: 658, w: 320, h: 76},
  {id: "stop", label: "STOP 双臂", x: 632, y: 658, w: 300, h: 76},
  {id: "calibration_panel", label: "选择/标定", x: 948, y: 658, w: 140, h: 76},
  {id: "recenter", label: "面板居中", x: 1104, y: 658, w: 140, h: 76},
  {id: "exit", label: "退出 AR", x: 1064, y: 30, w: 180, h: 64},
];
const HOME_BUTTONS = [
  {id:"return_left_start",label:"左臂回开始位",x:36,y:570,w:592,h:72},
  {id:"return_right_start",label:"右臂回开始位",x:652,y:570,w:592,h:72},
];
const CALIBRATION_BUTTONS = [
  {id:"calibration_select_left",label:"左臂/左手",x:780,y:132,w:200,h:56},
  {id:"calibration_select_right",label:"右臂/右手",x:998,y:132,w:200,h:56},
  {id:"calibration_record_forward",label:"一键记录操作者前方",x:54,y:282,w:690,h:64},
  {id: "calibration_begin", label: "1 开始记录X", x: 54, y: 480, w: 555, h: 66},
  {id: "calibration_capture_x", label: "2 完成X标定 · 自动应用", x: 633, y: 480, w: 555, h: 66},
  {id: "calibration_cancel", label: "取消采集", x: 54, y: 564, w: 220, h: 54},
  {id: "calibration_clear", label: "清除标定", x: 294, y: 564, w: 220, h: 54},
  {id: "voice_toggle", label: "语音提示", x: 784, y: 564, w: 404, h: 54},
];
const REASONS = {
  LEFT_GRIP_ANCHORED_WORLD_POSE_1_TO_1: "位移与转角按本次握持起点1:1保持目标；倾角≤45°，松Grip暂停",
  INITIAL_TOOL_TILT_EXCEEDS_5_DEG: "初始工具轴偏离朝下超过5°；请先回开始位，重检后开始",
  MEASURED_TOOL_TILT_EXCEEDS_45_DEG: "实测工具轴超出朝下45°锥范围，已请求停止跟随",
  IK_TOOL_TILT_EXCEEDS_45_DEG: "IK候选超出朝下45°锥范围，本点未下发；请向锥内调整",
  COMMAND_TOOL_TILT_EXCEEDS_45_DEG: "最终指令超出朝下45°锥范围，本点未下发",
  TOOL_CONE_PATH_OUTSIDE: "模型预测关节插值途中越过45°锥面，本点未下发",
  TOOL_CONE_PATH_NOT_CERTIFIED: "本次路径尚无法确认在45°锥内，本点未下发",
  TOOL_CONE_PATH_START_CHANGED: "实测关节在路径检查时变化，本点作废并重新计算",
  LEFT_GRIP_ANCHORED_WORLD_POSITION_1_TO_1: "位置跟随本次握持起点的累计位移；手停仍追目标，松 Grip 暂停",
  LEFT_WORLD_INCREMENTAL: "按已标定的基座方向跟随本窗增量；松开 Grip 暂停",
  LEFT_WORLD_INCREMENTAL_MEASURED_BASE_1_TO_1: "按已标定的基座方向跟随本窗增量；松开 Grip 暂停",
  RANGEDIK_WITHIN_RANGE: "IK候选已在任务容差内；实测 TCP 与已发送目标见侧面视图",
  RANGEDIK_APPROACHING: "正在接近任务范围，尚未到位；允许位姿误差内求连续解",
  DIRECT_TRACKING_ERROR_BEFORE_SEND: "RangedIK指令与最新实测关节偏差超过下发上限，目标已拒绝并停止",
  DIRECT_COMMAND_FK_MODEL_MISMATCH: "RangedIK直接指令的正解回验不一致，已停止",
  RANGEDIK_BACKEND_NOT_INITIALIZED: "RangedIK尚未准备，请连接 / 重检",
  CURRENT_POSE_CONTROLLER_FK_MISMATCH: "当前实测关节与末端正解不一致，预检未通过",
  RANGEDIK_HOLD_NO_PROGRESS: "本窗未找到有效接近解，保持当前实测姿态；不补发旧增量",
  RANGEDIK_NO_ADMISSIBLE_ORIENTATION_STEP: "未找到满足原边界的姿态接近解，本点未下发；握持目标保留",
  TRACKING_WINDOW_30_DEG_FEEDBACK_TIMEOUT: "前一指令跟踪误差或队列进度未满足30°放行条件，等待超时，已停止",
  REFERENCE_TRACKING_BOX_INFEASIBLE: "实际关节与平滑参考偏差过大，已停止；请重检",
  REFERENCE_DT_OUT_OF_RANGE: "指令更新间隔异常，平滑参考已停止；请重检",
  QP_TRACKING_ERROR_BEFORE_SEND: "平滑指令与最新实测关节偏差超过设定上限，已停止",
  QP_COMMAND_FK_MODEL_MISMATCH: "平滑指令的正解回验不一致，已停止",
  IK_NO_CONTINUOUS_SOLUTION_WITHIN_10_DEG: "当前TCP增量未找到10°邻域内的连续解；请减小本次动作或调整起始姿态后重检",
  IK_NO_CONTINUOUS_SOLUTION_WITHIN_30_DEG: "当前TCP增量未找到30°邻域内的连续解；请减小本次动作或调整起始姿态后重检",
  IK_JOINT_STEP_EXCEEDS_30_DEG: "IK候选相对实测关节变化超过30°，目标已拒绝",
  CONTROLLER_IK_BRANCH_DISAGREES_WITH_LOCAL: "控制器IK与连续解分支不一致，目标已拒绝，请重检",
  IK_CONTROLLER_FK_TARGET_MISMATCH: "IK正解回算与TCP目标不一致，目标已拒绝",
  LOCAL_FK_MEASURED_MISMATCH: "运动学模型与实测末端不一致，请连接 / 重检",
  QUEST_DISCONNECTED_OR_STALE: "PC未收到新输入，已撤销跟随；恢复后重新开始",
  QUEST_NOT_VISIBLE: "AR失去焦点：关闭系统菜单，回到透视画面",
  LEFT_CONTROLLER_TRACKING_INVALID: "左手柄追踪无效：放到头显摄像头能看到的位置",
  RELEASE_GRIP_BEFORE_FIRST_PRESS: "先完全松开中指Grip，再握持进行小幅位移与转角操作",
  PRESS_GRIP_TO_FOLLOW: "待命：按住左Grip，进行位移与转角操作；松开即停止跟随",
  LEFT_Z_ONLY: "正在跟随左手上下位移；XY及工具朝向保持固定",
  LEFT_SE3_TOOL_RELATIVE_1_TO_1: "六自由度1:1跟随；手柄局部轴对应Grip时工具局部轴",
  LEFT_INCREMENTAL_MEASURED_BASE_1_TO_1: "增量跟随：本窗手柄变化 + 当前实测末端；不补历史欠账",
  GRIP_RELEASED: "Grip已松开，等待静止确认",
  EXPLICIT_ARM_REQUIRED: "预检通过后点击开始；开始后先松开一次Grip",
  OPERATOR_DISARMED: "已停止遥操作；重检后可重新开始",
};
const finite = value => typeof value === "number" && Number.isFinite(value);
const valueText = (value, digits = 1) => finite(value) ? value.toFixed(digits) : "—";
const stateForInitial = state => ["PREPARED", "DISARMED"].includes(state);
const direction = q => [
  -2 * (q[0] * q[2] + q[3] * q[1]),
  -2 * (q[1] * q[2] - q[3] * q[0]),
  -(1 - 2 * (q[0] ** 2 + q[1] ** 2)),
];
const dot = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0);

export function rayToPanel(origin, ray, panel, pixelWidth = HUD_WIDTH, pixelHeight = HUD_HEIGHT) {
  if (!panel || ![...origin, ...ray].every(finite)) return null;
  const denominator = dot(ray, panel.normal);
  if (denominator >= -1e-6) return null;
  const distance = dot(panel.center.map((v, i) => v - origin[i]), panel.normal) / denominator;
  if (!(distance > 0 && distance < 5)) return null;
  const point = origin.map((v, i) => v + distance * ray[i]);
  const relative = point.map((v, i) => v - panel.center[i]);
  const u = dot(relative, panel.right) / panel.width + 0.5;
  const v = 0.5 - dot(relative, panel.up) / panel.height;
  if (u < 0 || u > 1 || v < 0 || v > 1) return null;
  return {x: u * pixelWidth, y: v * pixelHeight, point};
}

export function buttonAt(x, y, calibrationOpen = false) {
  return [...BUTTONS, ...(calibrationOpen ? CALIBRATION_BUTTONS : HOME_BUTTONS)].find(b =>
    x >= b.x && x <= b.x + b.w && y >= b.y && y <= b.y + b.h)?.id || null;
}

export function hudView(remote, receivedAt, client, now) {
  const policy = remote?.timing_policy || {};
  const statusAge = policy.hud_status_max_age_ms ?? 1500;
  const admissionAge = policy.quest_admission_max_age_ms ?? 400;
  const motionAge = policy.quest_motion_max_age_ms ?? 200;
  const fresh = client.linkState === "CONNECTED" && remote?.type === "teleop_status" &&
    finite(receivedAt) && now >= receivedAt && now - receivedAt < statusAge;
  const robot = fresh ? remote.robot || {} : {};
  const control = fresh ? remote.control || {} : {};
  const group = fresh ? remote.group : null;
  const participants = group?.participants?.length ? group.participants : [control.side || "left"];
  const dual = participants.length === 2;
  const groupBusy = Boolean(group?.armed || ["ARMING","STOPPING"].includes(group?.state));
  const quest = fresh ? remote.quest || {} : {};
  const capabilities = fresh ? remote.capabilities || {} : {};
  const directRanged = control.limits?.motion_reference === "RANGEDIK_DIRECT" &&
    control.ranged_ik?.backend === "official_rangedik_panoc" &&
    control.ranged_ik?.output_mode === "direct";
  const fixedWorld = control.limits?.coordinate_mapping === "XR_WORLD_FIXED_CALIBRATED_BASE";
  const gripAnchoredPosition = control.limits?.control_mode === "GRIP_ANCHORED_WORLD_POSITION";
  const persistentOrientation = control.limits?.orientation_target_policy === "GRIP_ANCHORED_WORLD" &&
    control.ranged_ik?.orientation_target_policy === "GRIP_ANCHORED_WORLD" &&
    control.limits?.orientation_target_reference === "GRIP_ENGAGEMENT_TCP_ORIENTATION";
  const geodesic = control.ranged_ik?.orientation_error_metric === "GEODESIC_ANGLE";
  const metricContractReady = !geodesic || (
    control.ranged_ik.native_orientation_error_metric === "GEODESIC_ANGLE" &&
    control.limits?.orientation_error_metric === "GEODESIC_ANGLE" &&
    control.ranged_ik.orientation_tolerance_deg === control.limits.orientation_tolerance_deg &&
    finite(control.limits.orientation_tolerance_deg) && control.limits.orientation_tolerance_deg > 0);
  const multiStart = control.ranged_ik?.seed_strategy === "VALIDATED_MULTI_START";
  const seedContractReady = !multiStart || (
    control.ranged_ik.native_preview_selection === true &&
    control.ranged_ik.seed_search?.strategy === "VALIDATED_MULTI_START");
  const orientationContractReady = persistentOrientation || (
    control.ranged_ik?.orientation_target_policy !== "GRIP_ANCHORED_WORLD" &&
    control.limits?.orientation_target_policy !== "GRIP_ANCHORED_WORLD" &&
    control.limits?.orientation_target_reference === "CURRENT_MEASURED_TCP_PLUS_WINDOW_ROTATION");
  const positionContractReady = control.limits?.control_mode === "INCREMENTAL_MEASURED_BASE" ||
    (gripAnchoredPosition && directRanged && fixedWorld &&
      control.limits.position_target_policy === "GRIP_ANCHORED_WORLD" &&
      control.limits.target_reference === "GRIP_ENGAGEMENT_TCP_POSITION" &&
      control.limits.hand_delta_reference === "GRIP_ENGAGEMENT_POSITION" &&
      orientationContractReady);
  const legacyQP = control.limits?.motion_reference === "FILTERED_VELOCITY_ACCELERATION_BOX_QP" &&
    control.limits?.coordinate_mapping === "GRIP_DIRECTION_FIXED_INCREMENTAL_MEASURED_TCP_NOT_WORLD_CALIBRATED";
  const contractReady = (fixedWorld || legacyQP) && control.limits?.motion_dof === 6 && control.limits?.translation_scale === 1 &&
    control.limits?.tracking_window_deg === 30 && control.limits?.max_unstarted_targets === 1 &&
    positionContractReady &&
    (directRanged || control.limits?.motion_reference === "FILTERED_VELOCITY_ACCELERATION_BOX_QP");
  const connected = client.linkState === "CONNECTED" && fresh;
  const teleopSide = (remote?.teleop_side || control.side) === "right" ? "right" : "left";
  const handName = teleopSide === "right" ? "右" : "左";
  const calibrationSide = client.calibrationSide || teleopSide;
  const selectionMatches = Boolean(group) || calibrationSide === teleopSide;
  const activeGripHeld = teleopSide === "right" ? client.rightGripHeld : client.gripHeld;
  const controlCalibration = calibrationView(control, capabilities, connected);
  const sideCaps = remote?.calibration_capabilities?.[calibrationSide];
  const calibration = calibrationView({...control,direction_calibration:
    remote?.direction_calibrations?.[calibrationSide] || (calibrationSide==="left"?control.direction_calibration:undefined)},
    sideCaps?{...capabilities,...Object.fromEntries(Object.entries(sideCaps).map(([k,v])=>['calibration_'+k,v]))}:capabilities,connected);
  const trackingWarning = trackingWarningView({fresh, control});
  const cone = control.tool_orientation_cone || {};
  const coneInitialBlocked = cone.enabled === true && cone.initial_ready !== true &&
    !control.armed && stateForInitial(control.state);
  const homes = startPositionView(remote?.start_positions,fresh);
  const localTracked = (teleopSide === "right" ? client.rightTracking : client.leftTracking) === true && client.visibility === "visible";
  const inputFresh = finite(quest.age_ms) && quest.age_ms <
    (control.state === "FOLLOWING" ? motionAge : admissionAge);
  const rawGrip = quest.active_grip_value ?? quest[teleopSide+"_grip_value"];
  // The server owns start/motion freshness gates. Read-only preparation must
  // remain requestable without left-hand tracking; avoid a second UI-only gate.
  const allowed = action => connected && capabilities[action] === true && !client.busy &&
    (action !== "arm" || (selectionMatches && (group ?
      participants.every(s=>(s==="right"?client.rightGripHeld:client.gripHeld)!==true) : activeGripHeld!==true) &&
      contractReady && (!fixedWorld || controlCalibration.ready) &&
      control.preflight_current === true && !coneInitialBlocked && metricContractReady && seedContractReady));
  const state = control.state || "等待PC状态";
  const collisionProtection = robot.connected === true && robot.robot_mode === 11 &&
    finite(robot.feedback_age_ms) && robot.feedback_age_ms >= 0 && robot.feedback_age_ms < 200;
  const clutchPhase = control.clutch?.phase;
  const paused = control.armed === true && clutchPhase === "PAUSED";
  const pausing = control.armed === true && clutchPhase === "PAUSING";
  const preflightExpired = state === "PREPARED" && control.preflight_current === false;
  const names = {NOT_PREPARED: "未连接机器人", PREPARED: "已准备 · 尚未开始", ARMING: "正在允许控制…",
    WAIT_RELEASE: "先松开Grip", READY: "已待命 · 按住Grip跟随", FOLLOWING: gripAnchoredPosition ?
      "正在跟随 · 握持起点位置" : "正在跟随 · 六自由度增量",
    STOPPING: "正在停止并确认…", DISARMED: "已停止遥操作", FAULT: "控制故障 · 已撤销跟随"};
  let reason;
  if (!fresh) reason = "PC状态未更新；旧输出已清除，不会自动重新开始";
  else if (group?.state === "FAULT") reason = "双臂停止尚未全部确认；查看各侧状态，处理后显式STOP";
  else if (group?.state === "ARMING") reason = "正在准备所有已连接臂；两侧均通过后才允许各自Grip跟随";
  else if (group?.state === "STOPPING") reason = "正在分别确认所有已连接臂停止";
  else if (homes.busy) reason = homes.arms[homes.activeSide]?.reason || "回位处理中；请保持AR可见，STOP可中断";
  else if (homes.fault) reason = "回位停止未确认；检查机械臂后使用STOP重试确认";
  else if (collisionProtection) reason = "E6 当前处于碰撞保护（模式11）；暂不能继续控制，请先在控制器检查并恢复。";
  else if (state === "FAULT" && String(control.reason).startsWith("STOP_NOT_CONFIRMED"))
    reason = "停止尚未通过反馈确认；请检查机器人状态，确认恢复后再重检。";
  else if (state === "FAULT" && control.reason === "DIRECT_TRACKING_ERROR_BEFORE_SEND") {
    const limit = control.limits?.reference_max_tracking_error_deg;
    reason = finite(limit) ? `RangedIK指令与最新实测关节偏差超过${valueText(limit, 0)}°，目标已拒绝并停止` : REASONS[control.reason];
  }
  else if (state === "FAULT") reason = REASONS[control.reason] || control.reason || "故障停止；等待后端确认恢复条件";
  else if (pausing) reason = "正在确认暂停；保持会话，确认后可再次握持继续";
  else if (paused) reason = "Grip 已松开，跟随暂停；连接保留，再次握持即可继续";
  else if (client.visibility !== "visible" || quest.status === "XR_NOT_VISIBLE")
    reason = "网络已连接；AR失去输入焦点，请关闭系统菜单并回到透视画面";
  else if (!inputFresh)
    reason = `网络已连接；当前位姿输入已 ${valueText(quest.age_ms, 0)} ms 未更新`;
  else if (!selectionMatches) reason = `已选${calibrationSide==="right"?"右":"左"}臂；点击连接/重检切换控制侧，仅检查该侧手柄。`;
  else if (!localTracked) reason = `网络已连接；${handName}手柄追踪无效，请把手柄放回头显视野`;
  else if (preflightExpired)
    reason = "机器人预检已过期：反馈已重连或坐标上下文改变，请点击连接 / 重检";
  else if (dual && group.state === "PREPARED" && !capabilities.arm) {
    const pending = participants.filter(s=>!group.arm_ready?.[s]).map(s=>s==="left"?"左":"右").join("、");
    reason = `${pending}侧尚未就绪：检查该侧手柄、方向标定、预检与初始朝下姿态；回位无需标定。`;
  }
  else if (fixedWorld && !controlCalibration.ready && !control.armed)
    reason = "先完成基座水平航向标定和操作者前方记录；工具Z不决定平移方向";
  else if (coneInitialBlocked)
    reason = finite(cone.measured_tilt_deg) ?
      `当前向下倾角 ${valueText(cone.measured_tilt_deg)}°；开始需 ≤${valueText(cone.initial_limit_deg,0)}°，请先回开始位` :
      "初始工具倾角尚无有效反馈，暂不能开始";
  else if (finite(rawGrip) && rawGrip > 0.2 &&
           ["NOT_PREPARED", "PREPARED", "WAIT_RELEASE", "DISARMED"].includes(state))
    reason = `${handName}Grip尚未完全松开：当前 ${rawGrip.toFixed(2)}，请松至 ≤0.20`;
  else if (state === "DISARMED" && /^(QUEST_|LEFT_CONTROLLER_|RIGHT_CONTROLLER_)/.test(control.reason || ""))
    reason = "当前输入已恢复；上次因输入失效停止。请重检后重新开始";
  else if (state === "DISARMED" && String(control.reason).includes("InverseKin"))
    reason = "当前连接正常；上次IK拒绝目标。请核对新起始姿态并重检";
  else reason = client.notice || REASONS[control.reason] || control.reason || "连接后先重检机器人";
  return {
    grippers: fresh ? remote?.grippers : null,
    fresh, connected, robot, robots: fresh ? remote.robots || {} : {}, controls: fresh ? remote.controls || {} : {},
    control, quest, state, group, participants, dual, teleopSide, handName, localTracked, selectionMatches, directRanged, fixedWorld, gripAnchoredPosition, persistentOrientation, geodesic, multiStart, calibration, calibrationSide, paused, pausing, trackingWarning, homes, cone,
    title: !connected ? "PC状态未更新 · 跟随不可用" : homes.busy ? `${homes.activeSide === "left" ? "左" : "右"}臂 · ${homes.arms[homes.activeSide]?.label}` : homes.fault ? "回位停止未确认" : collisionProtection ? "E6 碰撞保护 · 控制未恢复" : !contractReady ?
      "机械臂控制链尚未就绪 · 请先连接/重检" : state === "FAULT" ? names.FAULT : preflightExpired ?
      "机器人预检已过期 · 请重检" : paused ? "已暂停 · 握持继续" : pausing ? "正在确认暂停…" :
      (fixedWorld && !controlCalibration.ready && !control.armed) ? "先完成水平航向与操作者前方标定" : (names[state] || state),
    reason,
    color: !connected || ["FAULT", "STOPPING"].includes(state) ? "#ff8587" : control.armed ? "#66efbd" : "#ffd27a",
    pcLabel: connected ? "PC 已连接" : "PC 未更新 / 重连中",
    robotLabel: collisionProtection ? "E6 碰撞保护（模式11）" : robot.valid && robot.connected ?
      `E6 模式${robot.robot_mode} · ${robot.error_status === 0 ? "无错误标志" : "故障"}` : "E6 未准备 / 反馈失效",
    handLabel: dual ? `双手追踪 L:${client.leftTracking?"有效":"缺失"} R:${client.rightTracking?"有效":"缺失"}` : localTracked ? (finite(rawGrip) ?
      `${handName}手 · Grip ${rawGrip.toFixed(2)} ${activeGripHeld ? "握持" : rawGrip > 0.2 ? "请全松" : "释放"}` :
      `${handName}手有效 · Grip ${activeGripHeld ? "握持" : "释放"}`) : `${handName}手追踪 / AR焦点无效`,
    allowed: {prepare: allowed("prepare"), arm: allowed("arm"),
      return_left_start: allowed("return_left_start") && client.visibility === "visible" && client.gripHeld !== true,
      return_right_start: allowed("return_right_start") && client.visibility === "visible" && client.rightGripHeld !== true,
      ...Object.fromEntries(Object.entries(calibration.allowed).map(([key, permitted]) => [key,
        permitted && !client.busy && (["calibration_cancel", "calibration_clear"].includes(key) ||
          (client.visibility === "visible" && inputFresh &&
           (calibrationSide==="left" ? client.leftTracking===true && client.gripHeld!==true : client.rightTracking===true && client.rightGripHeld!==true)))])),
      calibration_select_left: !client.busy && !groupBusy && !control.armed && !["ARMING","STOPPING"].includes(state) && !homes.busy,
      calibration_select_right: !client.busy && !groupBusy && !control.armed && !["ARMING","STOPPING"].includes(state) && !homes.busy,
      calibration_panel: true,
      // Stop is still requestable when telemetry is stale, if the WSS link exists.
      stop: client.linkState === "CONNECTED", recenter: true, exit: true},
    actual: robot.valid ? robot.position_mm : null,
    targetZ: control.target?.pose_mm_deg?.[2],
    actualPose: robot.valid ? robot.pose_mm_deg : null,
    targetPose: control.target?.pose_mm_deg,
    joints: robot.valid ? robot.joint_position_deg : null,
  };
}

export class HeadsetHUD {
  constructor({onAction = () => {}, canvas = null, leftModelCanvas = null, rightModelCanvas = null, readOnly = false} = {}) {
    this.onAction = onAction; this.readOnly = readOnly;
    this.canvas = canvas || document.createElement("canvas");
    this.canvas.width = HUD_WIDTH; this.canvas.height = HUD_HEIGHT;
    this.ctx = this.canvas.getContext("2d");
    this.robotModels = new E6SideModels({leftCanvas: leftModelCanvas, rightCanvas: rightModelCanvas});
    this._modelDirty = true; this.calibrationOpen = false;
    this.announcer = new TrackingAnnouncer();
    this.remote = null; this.receivedAt = -Infinity;
    this.client = {linkState: "DISCONNECTED", visibility: "hidden", calibrationSide:null};
    this.panel = null; this.trigger = {}; this.stick = {};
    this.hover = null; this.history = []; this.lastPaint = -Infinity;
  }

  acceptState(state, now = performance.now()) {
    if (state?.type !== "teleop_status" || state.schema !== 1) return;
    this.remote = state; this.receivedAt = now;
    const z = state.robot?.valid ? state.robot.position_mm?.[2] : null;
    if (finite(z)) this.history.push({z, target: state.control?.target?.pose_mm_deg?.[2]});
    if (this.history.length > 150) this.history.shift();
  }

  setClient(client) { this.client = {...this.client, ...client}; }
  resetInteraction() { this.trigger = {}; this.stick = {}; }

  actionMessage(action, id) {
    const side=this.client.calibrationSide || this.remote?.teleop_side || "left";
    if (action.startsWith("calibration_")) return calibrationMessage(action, id, side);
    return ["prepare", "arm", "stop", "return_left_start", "return_right_start"].includes(action) ?
      {type: "control_action", id, action,...(["prepare","arm"].includes(action)?{side:this.remote?.group?"auto":side}:{})} : null;
  }

  get modelPanels() {
    if (!this.panel) return null;
    const p = this.panel;
    const width = p.height * SIDE_MODEL_WIDTH / SIDE_MODEL_HEIGHT;
    const offset = p.width / 2 + width / 2 + .025;
    return [-1, 1].map(side => ({...p, width,
      center: p.center.map((v, i) => v + side * offset * p.right[i])}));
  }

  recenter(transform) {
    const p = transform.position, q = transform.orientation;
    const f = direction([q.x, q.y, q.z, q.w]);
    const norm = Math.hypot(f[0], f[2]);
    if (norm < 0.1) return;
    const forward = [f[0] / norm, 0, f[2] / norm];
    const right = [-forward[2], 0, forward[0]];
    this.panel = {right, up: [0, 1, 0], normal: forward.map(v => -v), width: 0.96, height: 0.60,
      center: [p.x, p.y - 0.10, p.z].map((v, i) => v + 1.45 * forward[i])};
  }

  update(frame, reference, viewer, hands, now) {
    if (!this.panel) this.recenter(viewer.transform);
    const rays = [];
    let hover = null;
    const view = hudView(this.remote, this.receivedAt, this.client, now);
    view.allowed.voice_toggle = this.announcer.supported && this.announcer.status !== "UNAVAILABLE";
    for (const side of ["left", "right"]) {
      if (!hands[side]?.pose || hands[side].pose.emulated_position || this.client.visibility !== "visible") {
        delete this.trigger[side]; delete this.stick[side];
      }
    }
    for (const source of frame.session.inputSources) {
      const side = source.handedness, hand = hands[side], gp = source.gamepad;
      if (!hand?.is_controller || !hand.pose || hand.pose.emulated_position ||
          this.client.visibility !== "visible" || !source.targetRaySpace) continue;
      const pose = frame.getPose(source.targetRaySpace, reference);
      if (!pose) continue;
      const p = pose.transform.position, q = pose.transform.orientation;
      const origin = [p.x, p.y, p.z], ray = direction([q.x, q.y, q.z, q.w]);
      const hit = rayToPanel(origin, ray, this.panel);
      const pressed = Boolean(gp?.buttons?.[0]?.pressed || gp?.buttons?.[0]?.value >= 0.7);
      const stick = Boolean(gp?.buttons?.[3]?.pressed);
      if (!pressed) this.trigger[side] = false;
      if (stick && this.stick[side] === false) this.recenter(viewer.transform);
      this.stick[side] = stick;
      const id = hit ? buttonAt(hit.x, hit.y, this.calibrationOpen) : null;
      if (id) hover = id;
      if (pressed && this.trigger[side] === false && id && view.allowed[id] && !this.readOnly) {
        if (id === "voice_toggle") this.announcer.toggle();
        else if (id === "calibration_panel") this.calibrationOpen = !this.calibrationOpen;
        else if (id === "calibration_select_left") this.client.calibrationSide="left";
        else if (id === "calibration_select_right") this.client.calibrationSide="right";
        else if (id === "recenter") this.recenter(viewer.transform);
        else this.onAction(id);
      }
      if (pressed) this.trigger[side] = true;
      rays.push({side, origin, end: hit?.point || origin.map((v, i) => v + 0.7 * ray[i]), hit: Boolean(hit)});
    }
    this.hover = hover;
    return rays;
  }

  paint(now = performance.now(), force = false) {
    const v = hudView(this.remote, this.receivedAt, this.client, now);
    if (!this.readOnly) this.announcer.update(v.control.armed === true && v.state === "FOLLOWING" ?
      v.trackingWarning : {active: false}, now);
    v.voice = this.announcer.snapshot();
    v.allowed.voice_toggle = v.voice.supported && v.voice.status !== "UNAVAILABLE";
    this._modelDirty = this.robotModels.paint(v, now, force) || this._modelDirty;
    if (!force && now - this.lastPaint < 180) return false;
    this.lastPaint = now;
    const c = this.ctx;
    c.clearRect(0, 0, HUD_WIDTH, HUD_HEIGHT);
    const rounded = (x, y, w, h, fill, radius = 18) => {
      c.fillStyle = fill; c.beginPath(); c.roundRect(x, y, w, h, radius); c.fill();
    };
    const text = (str, x, y, size = 25, color = "#d6e2f4") => {
      c.font = `${size}px system-ui, sans-serif`; c.fillStyle = color; c.fillText(str, x, y);
    };
    rounded(0, 0, 1280, 800, "rgba(9,17,32,0.94)", 28);
    text(v.dual?"双臂同步遥操":`${v.handName}臂单臂遥操`, 36, 70, 36, "#ffffff");
    const range = v.control.ranged_ik;
    text(range ? `RangedIK ${v.directRanged ? "Direct" : "+ QP"} · ${v.persistentOrientation ? "位姿目标保持" : v.gripAnchoredPosition ? "握持起点位置" : "固定方向"}${v.multiStart?" · 初值优化":""}` :
      "姿态与控制参数 · 待机械臂预检", 310, 68, 21, "#99aecb");
    const rec = this.remote?.recording;
    const recText = !rec?.enabled ? "REC 未启用" : rec.writer_error ? "REC 写盘错误" :
      rec.dropped_records ? `REC 缺样 ${rec.dropped_records}` : rec.active ? "REC 记录中" :
      rec.state === "FINALIZING" ? "REC 保存中" : rec.state === "SAVED" ? "REC 已保存" : "REC 自动待命";
    text(recText, 790, 68, 23, rec?.writer_error ? "#ff8587" : rec?.active ? "#66efbd" : "#ffd27a");
    const badges = [[36, v.pcLabel, v.connected], [448, v.robotLabel, v.robot.valid],
      [860, v.handLabel, v.localTracked]];
    for (const [x, label, good] of badges) {
      rounded(x, 126, 384, 72, good ? "#163a35" : "#442b35");
      text(label, x + 16, 170, 26, good ? "#8ff1ce" : "#ffb8b9");
    }
    text(`输入 ${valueText(v.quest.receive_hz)} Hz  ·  PC数据年龄 ${valueText(v.quest.age_ms, 0)} ms`, 38, 233, 23);
    text(`网页RTT ${valueText(this.client.rtt, 0)} ms  ·  机器人反馈年龄 ${valueText(v.robot.feedback_age_ms, 0)} ms`, 645, 233, 23);
    rounded(36, 254, 1208, 102, "#17253d");
    text(v.title, 58, 299, 36, v.color);
    text(String(v.reason).slice(0, 65), 58, 337, 24, "#e2e9f6");
    text(`姿态°  实际 ${[3,4,5].map(i=>valueText(v.actualPose?.[i],1)).join(" / ")}    目标 ${[3,4,5].map(i=>valueText(v.targetPose?.[i],1)).join(" / ")}`,38,375,20,"#a8bad5");
    if (v.cone.enabled) text(`向下倾角 ${valueText(v.cone.measured_tilt_deg)}° / ${valueText(v.cone.limit_deg,0)}° · 初始≤${valueText(v.cone.initial_limit_deg,0)}°`,
      780,375,20,v.cone.inside_cone?"#8ff1ce":"#ff8587");
    const metrics = [["实际 X / mm", v.actual?.[0]], ["实际 Y / mm", v.actual?.[1]],
      ["实际 Z / mm", v.actual?.[2]], ["目标指令 XYZ / mm", v.targetZ]];
    metrics.forEach(([label, value], i) => {
      const x = 36 + i * 308;
      rounded(x, 380, 284, 109, "#132137");
      text(label, x + 16, 414, 23, "#a8bad5");
      if (i===3) [0,1,2].forEach(axis=>text(`${"XYZ"[axis]} ${valueText(v.targetPose?.[axis],3)}`,x+16,440+axis*21,22,"#ffd27a"));
      else text(valueText(value, 3), x + 16, 467, 43, "#8ff1ce");
    });
    const joints = Array.from({length: 6}, (_, i) => `J${i + 1} ${valueText(v.joints?.[i], 2)}°`).join("   ");
    text(joints, 38, 528, 24);
    if(v.persistentOrientation)text(`总角误差 ${valueText(v.control.orientation_tracking?.error_angle_deg)}°${v.geodesic?" / "+valueText(v.control.ranged_ik.orientation_tolerance_deg,0)+"°":""}`,955,528,20,
      v.control.orientation_tracking?.within_orientation_tolerance===false?"#ffd27a":"#a8bad5");
    if (!this.calibrationOpen) {
      text(v.dual ? `左：${v.controls.left?.state || "未准备"} · ${v.homes.arms.left.label} | 右：${v.controls.right?.state || "未准备"} · ${v.homes.arms.right.label}` :
        '左：'+v.homes.arms.left.label+'   |   右：'+v.homes.arms.right.label,38,553,20,'#ffd27a');
    }
    if (this.calibrationOpen) {
      rounded(24, 114, 1232, 526, "#161e2b");
      const hand=v.calibrationSide==="left"?"左":"右";
      text(`方向标定 · ${hand}臂/手柄`, 54, 169, 32, "#ffffff");
      text(v.calibration.label,54,213,25,v.calibration.ready?"#66efbd":"#ffd27a");
      text(v.calibration.instruction, 54, 256, 23);
      const op=v.calibration.data.operator_frame || {};
      text(`记录时的${hand}手柄指向作为前方；同一手柄点击即可，另一手可离线。`,54,374,23);
      const distance=v.calibration.data.live_horizontal_distance_mm ?? v.calibration.data.axis_lengths_mm?.[0];
      text(`水平X位移 ${valueText(distance,0)} mm / 至少 ${v.calibration.data.min_distance_mm ?? 80} mm · Y/Z自动补齐 · 前方${op.current?"已记录":"未记录"}`,54,412,24);
      const error=v.calibration.data.last_error;
      text(error?`失败：${String(error).slice(0,50)}`:`操作者+X在该臂基座内 ${valueText(v.calibration.data.operator_forward_yaw_deg,1)}°；Z向上，不随手腕转动。`,54,448,22,error?"#ff8587":"#aabbd1");
    }
    for (const b of [...BUTTONS, ...(this.calibrationOpen ? CALIBRATION_BUTTONS : HOME_BUTTONS)]) {
      const enabled = v.allowed[b.id] && !this.readOnly;
      const fill = b.id === "stop" ? "#9a3542" : b.id === "arm" ? "#226553" : "#273b58";
      rounded(b.x, b.y, b.w, b.h, enabled ? fill : "#1c293d");
      if (this.hover === b.id && enabled) {
        c.strokeStyle = "#ffffff"; c.lineWidth = 3; c.strokeRect(b.x + 3, b.y + 3, b.w - 6, b.h - 6);
      }
      const label = b.id === "voice_toggle" ? (v.voice.status === "UNAVAILABLE" ? "语音不可用 · 仅文字" :
        !v.voice.supported ? "不支持语音 · 仅文字提示" : `语音提示：${v.voice.enabled ? "开" : "关"}（最多每10秒）`) :
        b.id === "calibration_record_forward" ? `${b.label}（${v.calibrationSide==="left"?"左":"右"}手）` :
        b.id === "calibration_select_"+v.calibrationSide ? "✓ "+b.label :
        b.id.startsWith("return_") ? `${b.label} · ${v.homes.speed ?? "—"}%` :
        b.id === "arm" && v.paused ? "握持 Grip 继续" : b.id === "arm" && v.pausing ? "正在确认暂停" :
        b.id === "prepare" ? (v.group?"重检已连接臂":`连接${v.calibrationSide==="right"?"右":"左"}臂/重检`) :
        b.id === "arm" ? (v.group?"开始遥操作":`开始${v.handName}臂遥操`) :
        b.id === "calibration_panel" && this.calibrationOpen ? "返回状态" : b.label;
      text(label, b.x + (b.w < 200 ? 14 : 22), b.y + b.h / 2 + 10,
        b.w < 200 || b.id.startsWith("calibration_") || b.id === "voice_toggle" ? 25 : b.id === "exit" ? 27 : 32,
        enabled ? "#ffffff" : "#74849b");
    }
    text(this.readOnly ? "PC只读镜像 · 按钮仅在头显内操作" :
      v.dual ? "回位前确认工作区安全；按需清除对应臂碰撞保护再回位 · 日志保留" :
      v.gripAnchoredPosition ? "回位前确认工作区安全；按需清除本臂碰撞保护再回位 · 无需标定" :
      v.fixedWorld ? "基座方向标定后固定 · Grip松开暂停/再握持继续 · STOP结束控制" :
      "旧版工具相对方向 · 以后台控制状态为准", 38, 775, 23, "#aabbd1");
    return true;
  }

  initGL(gl) {
    this.gl = gl;
    const shader = (type, source) => {
      const s = gl.createShader(type); gl.shaderSource(s, source); gl.compileShader(s);
      if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw Error(gl.getShaderInfoLog(s));
      return s;
    };
    this.program = gl.createProgram();
    gl.attachShader(this.program, shader(gl.VERTEX_SHADER,
      "attribute vec3 aPosition;attribute vec2 aUv;uniform mat4 uProjection,uView;varying vec2 vUv;void main(){gl_Position=uProjection*uView*vec4(aPosition,1.0);vUv=aUv;}"));
    gl.attachShader(this.program, shader(gl.FRAGMENT_SHADER,
      "precision mediump float;uniform sampler2D uTexture;varying vec2 vUv;void main(){gl_FragColor=texture2D(uTexture,vUv);}"));
    gl.linkProgram(this.program);
    if (!gl.getProgramParameter(this.program, gl.LINK_STATUS)) throw Error(gl.getProgramInfoLog(this.program));
    this.buffer = gl.createBuffer(); this.texture = gl.createTexture();
    this.modelTextures = [gl.createTexture(), gl.createTexture()];
    for (const texture of [this.texture, ...this.modelTextures]) {
      gl.bindTexture(gl.TEXTURE_2D, texture);
      for (const key of [gl.TEXTURE_MIN_FILTER, gl.TEXTURE_MAG_FILTER]) gl.texParameteri(gl.TEXTURE_2D, key, gl.LINEAR);
      for (const key of [gl.TEXTURE_WRAP_S, gl.TEXTURE_WRAP_T]) gl.texParameteri(gl.TEXTURE_2D, key, gl.CLAMP_TO_EDGE);
    }
  }

  upload(now) {
    const gl = this.gl;
    const mainDirty = this.paint(now);
    for (const [dirty, texture, canvas] of [[mainDirty, this.texture, this.canvas],
      [this._modelDirty, this.modelTextures[0], this.robotModels.leftCanvas],
      [this._modelDirty, this.modelTextures[1], this.robotModels.rightCanvas]]) {
      if (!dirty) continue;
      gl.bindTexture(gl.TEXTURE_2D, texture); gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL, true);
      gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, canvas);
    }
    this._modelDirty = false;
  }

  draw(view) {
    if (!this.panel) return;
    for (const [panel, texture] of [[this.panel, this.texture],
      [this.modelPanels[0], this.modelTextures[0]], [this.modelPanels[1], this.modelTextures[1]]])
      this.drawPanel(view, panel, texture);
  }

  drawPanel(view, p, texture) {
    const gl = this.gl, vertices = [];
    for (const [u, v] of [[0, 0], [1, 0], [0, 1], [0, 1], [1, 0], [1, 1]]) {
      vertices.push(...p.center.map((x, i) => x + (u - 0.5) * p.width * p.right[i] + (v - 0.5) * p.height * p.up[i]), u, v);
    }
    gl.useProgram(this.program); gl.bindBuffer(gl.ARRAY_BUFFER, this.buffer);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(vertices), gl.DYNAMIC_DRAW);
    for (const [name, size, offset] of [["aPosition", 3, 0], ["aUv", 2, 12]]) {
      const index = gl.getAttribLocation(this.program, name); gl.enableVertexAttribArray(index);
      gl.vertexAttribPointer(index, size, gl.FLOAT, false, 20, offset);
    }
    gl.uniformMatrix4fv(gl.getUniformLocation(this.program, "uProjection"), false, view.projectionMatrix);
    gl.uniformMatrix4fv(gl.getUniformLocation(this.program, "uView"), false, view.transform.inverse.matrix);
    gl.activeTexture(gl.TEXTURE0); gl.bindTexture(gl.TEXTURE_2D, texture);
    gl.uniform1i(gl.getUniformLocation(this.program, "uTexture"), 0);
    gl.enable(gl.BLEND); gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
    gl.drawArrays(gl.TRIANGLES, 0, 6); gl.disable(gl.BLEND);
  }
}
