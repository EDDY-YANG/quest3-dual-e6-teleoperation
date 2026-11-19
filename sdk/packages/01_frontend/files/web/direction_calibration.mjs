// UI-only direction calibration contract. Samples and the authoritative matrix
// are owned by the backend; this module never invents or transmits poses.
export const CALIBRATION_ACTIONS = Object.freeze([
  "begin", "capture_x", "cancel", "clear", "record_forward",
]);

export function isRotationMatrix(matrix) {
  if (!Array.isArray(matrix) || matrix.length !== 3 || matrix.some(row =>
    !Array.isArray(row) || row.length !== 3 || !row.every(Number.isFinite))) return false;
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) {
    const dot = matrix[i].reduce((sum, v, k) => sum + v * matrix[j][k], 0);
    if (Math.abs(dot - Number(i === j)) > 1e-4) return false;
  }
  const m = matrix;
  const det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
    m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
    m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
  return Math.abs(det - 1) <= 1e-4;
}

export function isYawMatrix(matrix) {
  return isRotationMatrix(matrix) && matrix[2].every((v,i)=>Math.abs(v-Number(i===1))<1e-6);
}

export function calibrationMessage(action, id, side) {
  const key = action?.startsWith("calibration_") ? action.slice(12) : null;
  if (!CALIBRATION_ACTIONS.includes(key) || typeof id !== "string" ||
      !/^[A-Za-z0-9_-]{1,64}$/.test(id) || (side!==undefined && !["left","right"].includes(side))) return null;
  return {type: "calibration_action", id, action: key,...(side===undefined?{}:{side})};
}

export function calibrationView(control = {}, capabilities = {}, connected = false) {
  const data = control.direction_calibration || {};
  const supported = data.schema === 3 && data.mode === "X_ONLY_YAW_FIXED_UP";
  const state = supported ? data.state : "UNAVAILABLE";
  const ready = supported && state === "CALIBRATED" && data.current === true &&
    data.control_ready === true && data.operator_frame?.current === true && isYawMatrix(data.R_base_from_xr);
  const stopped = control.armed === false && !["ARMING", "STOPPING"].includes(control.state);
  const labels = {
    UNAVAILABLE: "等待标定接口", UNCALIBRATED: "未标定", WAIT_X: "记录基座 +X",
    CALIBRATED: ready?"水平映射已就绪":"基座已标定 · 待记录前方", INVALID: "标定已失效",
  };
  const minimum = Number.isFinite(data.min_distance_mm) ? data.min_distance_mm : 80;
  const hand = data.side === "right" ? "右" : "左";
  const instructions = {
    UNAVAILABLE: "等待后端提供方向标定状态；不使用手柄朝向代替标定。",
    UNCALIBRATED: `任选舒适位置，松开${hand} Grip，点击开始记录X；无需记住或返回起点。`,
    WAIT_X: `沿该臂基座 +X 平移${hand}手柄至少 ${minimum} mm，再点完成X标定。`,
    CALIBRATED: ready ? "X已自动应用，Y/Z已补齐；平移方向不随手腕转动。" :
      "基座X已自动应用；请记录一次共享的操作者前方。",
    INVALID: "XR 会话或参考空间已变化，请重新标定。",
  };
  const allowed = Object.fromEntries(CALIBRATION_ACTIONS.map(action => [
    `calibration_${action}`, connected && supported && stopped &&
      capabilities[`calibration_${action}`] === true,
  ]));
  const allowedStates = {begin: ["UNCALIBRATED", "INVALID", "CALIBRATED"],
    capture_x: ["WAIT_X"],
    cancel: ["WAIT_X"], clear: ["CALIBRATED", "INVALID"],
    record_forward:["UNCALIBRATED","WAIT_X","CALIBRATED","INVALID"]};
  for (const action of CALIBRATION_ACTIONS) {
    allowed[`calibration_${action}`] &&= allowedStates[action].includes(state);
  }
  return {supported, state, ready, data, allowed,
    label: labels[state] || "标定状态未知",
    instruction: control.armed === true ? "方向标定须先点击 STOP，确认停止后再进行。" :
      (instructions[state] || "请等待后端确认标定状态。")};
}
