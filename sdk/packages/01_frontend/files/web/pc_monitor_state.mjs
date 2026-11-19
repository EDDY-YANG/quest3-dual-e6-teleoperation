// Pure read-only presentation rules. Never grants motion permission.
export const finiteVector = (v, n) => Array.isArray(v) && v.length === n && v.every(Number.isFinite);
export const formatNumber = (v, digits=2) => Number.isFinite(v) ? v.toFixed(digits) : "—";
export const MODE_NAMES = {4:"未使能",5:"已使能 / 空闲",6:"拖动",7:"运动中",8:"单步",9:"错误",10:"暂停",11:"碰撞保护"};

export function armState(robot={}, fresh=false) {
  const packetFresh = fresh && robot.connected === true && Number.isFinite(robot.feedback_age_ms) &&
    robot.feedback_age_ms >= 0 && robot.feedback_age_ms < 200;
  const poseFresh = packetFresh && robot.packet_valid === true && robot.robot_type === 150 &&
    robot.user === 0 && robot.tool === 0 && finiteVector(robot.joint_position_deg,6) && finiteVector(robot.pose_mm_deg,6);
  const collision = packetFresh && robot.robot_mode === 11;
  return {packetFresh, poseFresh, collision, controlValid: packetFresh && robot.valid === true,
    label: !fresh ? "PC 状态未更新" : !robot.connected ? "未连接 / 等待反馈" : collision ? "碰撞保护 · 不可控制" :
      !packetFresh ? "反馈过期" : poseFresh ? "实测反馈有效" : "数据不完整 / 坐标未支持",
    mode: packetFresh ? `${robot.robot_mode} · ${MODE_NAMES[robot.robot_mode] || "未知模式"}` : "—",
    q: poseFresh ? robot.joint_position_deg : null, pose: poseFresh ? robot.pose_mm_deg : null};
}

export function renderView(data, fresh) {
  const viewFor = side => {
    const raw=data?.robots?.[side] || {}, state=armState(raw,fresh);
    // Visualization-only validity: even a collision-protected E6 can report
    // real encoder positions. Do not mutate backend `valid` or reuse this for control.
    return {...raw, valid:state.poseFresh};
  };
  const side=data?.teleop_side === "right" ? "right" : "left";
  return {fresh,grippers:fresh ? data?.grippers : null,teleopSide:side,robot:viewFor(side),robots:{left:viewFor("left"),right:viewFor("right")},
    control:data?.controls?.[side] || {},controls:data?.controls || {}};
}

export function handState(quest={}, side, fresh=false) {
  const frame=quest.frame || {}, hand=frame.hands?.[side], gamepad=hand?.gamepad;
  const frameFresh=fresh && quest.connected === true && Number.isFinite(quest.age_ms) && quest.age_ms>=0 && quest.age_ms<500;
  const tracked=Boolean(frameFresh && frame.visibility==="visible" && hand?.tracking_valid===true &&
    hand?.pose?.emulated_position===false && finiteVector(hand.pose.position_m,3) && finiteVector(hand.pose.quaternion_xyzw,4));
  const buttonsValid=Boolean(frameFresh && gamepad?.connected && gamepad.mapping==="xr-standard");
  const matchingModel=Boolean(hand?.profiles?.includes("meta-quest-touch-plus"));
  return {tracked,buttonsValid,matchingModel,
    label: tracked ? "位置与姿态有效" : frameFresh && hand?.pose?.emulated_position ? "推算位置 · 不用于控制" :
      buttonsValid ? "按键在线 · 位置无效" : "等待手柄输入",
    position:tracked ? hand.pose.position_m : null, quaternion:tracked ? hand.pose.quaternion_xyzw : null,
    buttons:buttonsValid ? gamepad.buttons : [], axes:buttonsValid ? gamepad.axes : [],
    gripHeld:buttonsValid ? hand.grip_held : null,
    profiles:hand?.profiles || [], session:quest.session, referenceSpace:frame.reference_space,
    referenceEpoch:frame.reference_epoch};
}
