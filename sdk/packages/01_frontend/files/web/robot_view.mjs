// Virtual kinematic reference viewport. This is not a camera image, collision
// mesh, or registration to the physical robot seen through passthrough.
export const ROBOT_VIEW_WIDTH = 960, ROBOT_VIEW_HEIGHT = 800;
export const DISPLAY_TOOL_Z_MM = 120;
const vec = (x, n) => Array.isArray(x) && x.length === n &&
  x.every(v => Number.isFinite(v) && Math.abs(v) < 100000);
const add = (a, b) => a.map((v, i) => v + b[i]);

export function poseAxes(pose, lengths = [65, 65, DISPLAY_TOOL_Z_MM]) {
  if (!vec(pose, 6) || !vec(lengths, 3) || lengths.some(x => x <= 0)) return null;
  const [x, y, z] = pose.slice(3).map(v => v * Math.PI / 180);
  const sx = Math.sin(x), cx = Math.cos(x), sy = Math.sin(y), cy = Math.cos(y),
    sz = Math.sin(z), cz = Math.cos(z);
  // Fixed-axis XYZ convention: Rz * Ry * Rx. Columns are tool axes in base.
  const directions = [[cz * cy, sz * cy, -sy],
    [cz * sy * sx - sz * cx, sz * sy * sx + cz * cx, cy * sx],
    [cz * sy * cx + sz * sx, sz * sy * cx - cz * sx, cy * cx]];
  return directions.map((direction, i) => ({axis: "XYZ"[i], origin: pose.slice(0, 3),
    direction, end: add(pose.slice(0, 3), direction.map(v => v * lengths[i]))}));
}

export function robotViewState(view) {
  const r = view.robot || {}, control = view.control || {};
  const valid = view.fresh === true && r.connected === true && r.valid === true &&
    Number.isFinite(r.feedback_age_ms) && r.feedback_age_ms >= 0 && r.feedback_age_ms < 200 &&
    r.user === 0 && vec(r.pose_mm_deg, 6) && vec(r.joint_position_deg, 6);
  if (!valid) return {valid: false, reason: view.fresh ?
    "等待有效的实测 TCP 与运动学链点" : "实时状态已过期，模型输出已清除"};
  const active = control.armed === true && control.state === "FOLLOWING";
  return {valid: true, pose: [...r.pose_mm_deg],
    joints: vec(r.joint_position_deg, 6) ? [...r.joint_position_deg] : null,
    user: r.user, tool: r.tool, toolValue: vec(r.tool_value_mm_deg, 6) ? r.tool_value_mm_deg : null,
    command: active && vec(control.target?.pose_mm_deg, 6) ? [...control.target.pose_mm_deg] : null,
    requested: active && vec(control.requested_target?.pose_mm_deg, 6) ? [...control.requested_target.pose_mm_deg] : null};
}

