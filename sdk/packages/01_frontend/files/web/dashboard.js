const $ = (selector) => document.querySelector(selector);
const names = {
  WAITING_FOR_QUEST: "等待 Quest 连接",
  LIVE: "正在接收 WebXR 数据",
  STALE_OR_DISCONNECTED: "失联 / 数据已过期",
  XR_NOT_VISIBLE: "XR 页面失去焦点",
};
let polling = false;
let lastGoodPoll = 0;
function label(index, side, gp, profiles) {
  if (gp?.mapping !== "xr-standard") return `Button ${index}`;
  const standard = ["Trigger", "Grip", "Touchpad", "Stick click"];
  if (standard[index]) return `${standard[index]} [${index}]`;
  if (profiles.some((p) => /oculus|meta-quest/i.test(p))) {
    if (index === 4) return `${side === "left" ? "X" : "A"} [4]`;
    if (index === 5) return `${side === "left" ? "Y" : "B"} [5]`;
  }
  return `Button ${index}`;
}
function renderHand(side, quest) {
  const root = $(`#${side}`),
    hand = quest.frame?.hands?.[side];
  const valid = Boolean(quest.valid?.[side]);
  const idleTracked = quest.status === "LIVE" &&
    quest.frame?.input_policy === "grip-gated-v1" &&
    hand?.tracking_valid === true && hand?.grip_held === false;
  const status = root.querySelector(".tracking");
  status.textContent = valid
    ? "TRACKED · 有效"
    : idleTracked
      ? "TRACKED · 松开待机 / 不发送位姿"
    : quest.status === "LIVE" && hand?.tracking_valid === false
      ? "头显跟踪无效 · 非网络断线"
    : hand?.pose?.emulated_position
      ? "推算位置 · 无效"
      : "未跟踪 / 已过期";
  status.classList.toggle("good", valid || idleTracked);
  const pose = valid ? hand.pose : null;
  const fields = ["X / m", "Y / m", "Z / m", "qX", "qY", "qZ", "qW"];
  const values = pose
    ? [...pose.position_m, ...pose.quaternion_xyzw]
    : Array(7).fill(null);
  const grid = root.querySelector(".pose");
  grid.replaceChildren();
  fields.forEach((name, i) => {
    const m = document.createElement("div");
    m.className = "metric";
    const l = document.createElement("label");
    l.textContent = name;
    const v = document.createElement("strong");
    v.textContent = values[i] === null ? "—" : values[i].toFixed(4);
    m.append(l, v);
    grid.append(m);
  });
  const buttons = root.querySelector(".buttons");
  buttons.replaceChildren();
  const gp = (valid || idleTracked) && hand.gamepad?.connected ? hand.gamepad : null;
  if (gp)
    gp.buttons.forEach((button, i) => {
      const el = document.createElement("span");
      el.className = "button-state" + (button.pressed ? " active" : "");
      el.textContent = `${label(i, side, gp, hand.profiles || [])} ${button.value.toFixed(2)}${button.touched ? " · touch" : ""}`;
      buttons.append(el);
    });
  else {
    const el = document.createElement("span");
    el.className = "muted";
    el.textContent = "等待真实手柄按键";
    buttons.append(el);
  }
  root.querySelector(".axes").textContent =
    `Axes: ${gp ? gp.axes.map((v, i) => `[${i}] ${v.toFixed(3)}`).join("  ") : "—"}`;
  root.querySelector(".profiles").textContent =
    `Profile: ${(hand?.profiles || []).join(", ") || "—"} · ${quest.frame?.reference_space || "—"} / epoch ${quest.frame?.reference_epoch ?? "—"}`;
  const stats = quest.stats?.[side];
  root.querySelector(".evidence").textContent = stats
    ? `本坐标原点下有效帧 ${stats.tracked_frames}；已观察按下→松开按钮索引：${stats.released_after_press.join(", ") || "无"}`
    : "尚无采样";
}
function renderRobots(e6) {
  if (!e6.robots.length) return;
  const container = $("#robots");
  container.replaceChildren();
  e6.robots.forEach((r) => {
    const card = document.createElement("div");
    card.className = "robot";
    const h = document.createElement("h3");
    const modeBlocked = r.replies.some(
      (x) => x.protocol_error === "NOT_TCP_MODE",
    );
    h.textContent = `${r.side === "left" ? "Left / 左臂" : "Right / 右臂"} · ${r.ip} · ${r.protocol_queries_ok ? "查询成功" : modeBlocked ? "已连通 · 当前非 TCP 控制模式" : "查询未通过"}`;
    const p = document.createElement("p");
    p.className = "muted";
    const localIp = r.dashboard_local_ip || r.feedback_local_ip || r.local_ip;
    p.textContent = `${localIp ? `本机网口 ${localIp} · ` : ""}TCP 29999: ${r.dashboard_tcp ? "可连接" : "未连通"} · TCP 30004: ${r.feedback_tcp ? "可连接" : "未连通"}`;
    const feedback = document.createElement("p");
    feedback.className = "muted";
    feedback.textContent = r.feedback_header
      ? `实际反馈 ${r.feedback_header.packet_bytes} bytes · 模式 ${r.feedback_header.robot_mode} (${r.feedback_header.robot_mode_name}) · 最近检查 ${r.checked_at}`
      : `反馈包未验证 · 最近检查 ${r.checked_at}`;
    const pre = document.createElement("pre");
    pre.textContent = [...r.replies.map((x) => x.raw_reply), ...r.errors].join(
      "\n",
    );
    card.append(h, p, feedback, pre);
    container.append(card);
  });
}
function invalidateDashboard() {
  $("#quest-status").textContent = "本地服务失联";
  $("#quest-status").classList.remove("safe");
  $("#save").disabled = true;
  ["left", "right"].forEach((side) =>
    renderHand(side, { valid: {}, stats: {} }),
  );
  $("#timing").textContent = "页面无法读取服务器状态，旧数值已清除";
}
async function poll() {
  if (polling) return;
  polling = true;
  try {
    const response = await fetch("/api/status", {
      signal: AbortSignal.timeout(1800),
    });
    if (!response.ok) throw Error(`HTTP ${response.status}`);
    const data = await response.json();
    lastGoodPoll = performance.now();
    const q = data.quest;
    $("#control-mode").textContent =
      data.mode === "READ_ONLY_NO_MOTION"
        ? "运动接口未启动"
        : data.teleop_side==="both"?"双臂同时遥操作":`单臂遥操作 · ${data.teleop_side==="right"?"右臂 / 右手":"左臂 / 左手"}`;
    $("#trial-status").textContent = JSON.stringify(
      data.teleop || data.left_trial || { state: "NOT_PREPARED" },
      null,
      2,
    );
    const lanInfo = $("#lan-info");
    lanInfo.hidden = !data.lan?.enabled;
    if (data.lan?.enabled) {
      const remaining = data.lan.code_remaining_seconds ?? 0;
      const expiry = data.lan.code_expired ? "已过期，请更新配对码" : `剩余 ${Math.floor(remaining / 60)}分${remaining % 60}秒`;
      lanInfo.textContent = `Quest 浏览器访问 ${data.lan.url}/ · 配对码 ${data.lan.pair_code}（${expiry}） · ${data.lan.paired_peer ? "已配对 " + data.lan.paired_peer : "等待头显配对"}。请从根地址重新打开配对页。`;
    }
    $("#lan-renew").hidden = !data.lan?.enabled;
    $("#lan-renew").disabled = Boolean(q.connected);
    $("#quest-status").textContent = names[q.status] || q.status;
    $("#quest-status").classList.toggle(
      "safe",
      q.status === "LIVE" && q.valid.left && q.valid.right,
    );
    $("#timing").textContent = !q.connected && q.age_ms !== null
      ? `数据流已断开 · 已 ${(q.age_ms / 1000).toFixed(1)} 秒未收到数据（不是动作延迟）`
      : `接收 ${q.receive_hz} Hz · 最新数据年龄 ${q.age_ms ?? "—"} ms（非端到端延迟）`;
    const network = q.frame?.diagnostics?.network;
    $("#network-diagnostics").textContent = network
      ? `Quest→PC网页请求往返 ${network.http_rtt_ms === null ? "超时/尚无结果" : Math.round(network.http_rtt_ms) + " ms"} · 输入重连 ${network.reconnects} 次 · 发送排队 ${network.buffered_bytes} B · 丢弃 ${network.skipped_frames} 帧 · 保持亮屏 ${network.screen_wake_lock || "未报告"} · ${network.last_close ? "上次关闭码 " + network.last_close.code : "尚无断线记录"}（往返时间不是机械臂端到端延迟）`
      : "暂无新链路诊断：请在Quest刷新采集页并重新进入AR。运动过期阈值未放宽。";
    ["left", "right"].forEach((s) => renderHand(s, q));
    renderRobots(data.e6);
    $("#save").disabled = !(
      q.status === "LIVE" &&
      (q.valid.left || q.valid.right)
    );
  } catch {
    invalidateDashboard();
  } finally {
    polling = false;
  }
}
async function action(button, path) {
  button.disabled = true;
  $("#message").textContent = "正在执行所选操作…";
  try {
    const r = await fetch(path, { method: "POST" });
    const text = await r.text();
    let data;
    try {
      data = JSON.parse(text);
    } catch {
      data = { error: text };
    }
    if (!r.ok) throw Error(data.error || text);
    $("#message").textContent =
      data.message ||
      (data.file
        ? `已保存 ${data.sample_count} 帧：${data.file}`
        : JSON.stringify(data, null, 2));
  } catch (e) {
    $("#message").textContent = `检查未完成：${e.message}`;
  } finally {
    button.disabled = false;
    poll();
  }
}
$("#usb").addEventListener("click", (e) =>
  action(e.currentTarget, "/api/quest/connect-usb"),
);
$("#lan-renew").addEventListener("click", (e) =>
  action(e.currentTarget, "/api/quest/lan/renew"),
);
$("#e6").addEventListener("click", (e) =>
  action(e.currentTarget, "/api/e6/check"),
);
$("#e6-left").addEventListener("click", (e) =>
  action(e.currentTarget, "/api/e6/check?side=left"),
);
$("#e6-right").addEventListener("click", (e) =>
  action(e.currentTarget, "/api/e6/check?side=right"),
);
$("#save").addEventListener("click", (e) =>
  action(e.currentTarget, "/api/quest/save"),
);
setInterval(poll, 250);
$("#trial-prepare").addEventListener("click", (e) =>
  action(e.currentTarget, "/api/teleop/prepare?side="+$("#teleop-side").value),
);
$("#trial-arm").addEventListener("click", (e) =>
  action(e.currentTarget, "/api/teleop/arm?side="+$("#teleop-side").value),
);
$("#trial-stop").addEventListener("click", (e) =>
  action(e.currentTarget, "/api/left-trial/stop"),
);
setInterval(() => {
  if (lastGoodPoll && performance.now() - lastGoodPoll > 2000)
    invalidateDashboard();
}, 500);
poll();
