import { GripGate } from "/static/grip_gate.mjs";
import { HeadsetHUD, hudView, preloadRobotVisual } from "/static/quest_hud.mjs";
const gripGate = new GripGate();
// Continuously streams measured poses; Grip is exclusively a motion permission gate.
const status = document.querySelector("#xr-status");
let session = null,
  reference = null,
  referenceType = null,
  referenceEpoch = 0,
  ws = null,
  seq = 0,
  sentAt = 0,
  gl = null,
  program = null,
  vertexBuffer = null;
let lastUiAt = 0,
  shuttingDown = false;
let requestedMode = null;
let reconnectTimer = null, rttTimer = null, linkGeneration = 0,
  reconnectAttempt = 0, reconnectCount = 0, skippedFrames = 0,
  lastInputError = "", linkState = "DISCONNECTED", lastClose = null,
  lastRttMs = null, lastRttAt = 0, rttInFlight = false;
let screenWakeLock = null, wakeLockState = "NOT_REQUESTED";
let hud = null, hudAction = null, hudActionSeq = 0, hudNotice = "", hudNoticeAt = 0;
const starts = [
  document.querySelector("#start-ar"),
  document.querySelector("#start-vr"),
];
const endButton = document.querySelector("#end");

function makeRenderer() {
  const canvas = document.createElement("canvas");
  document.body.append(canvas);
  gl = canvas.getContext("webgl", {
    xrCompatible: true,
    alpha: true,
    antialias: true,
  });
  if (!gl) throw Error("WebGL context unavailable");
  const shader = (type, source) => {
    const s = gl.createShader(type);
    gl.shaderSource(s, source);
    gl.compileShader(s);
    if (!gl.getShaderParameter(s, gl.COMPILE_STATUS))
      throw Error(gl.getShaderInfoLog(s));
    return s;
  };
  program = gl.createProgram();
  gl.attachShader(
    program,
    shader(
      gl.VERTEX_SHADER,
      "attribute vec3 aPosition; attribute vec3 aColor; uniform mat4 uProjection; uniform mat4 uView; varying vec3 color; void main(){gl_Position=uProjection*uView*vec4(aPosition,1.0);gl_PointSize=12.0;color=aColor;}",
    ),
  );
  gl.attachShader(
    program,
    shader(
      gl.FRAGMENT_SHADER,
      "precision mediump float; varying vec3 color; void main(){gl_FragColor=vec4(color,1.0);}",
    ),
  );
  gl.linkProgram(program);
  if (!gl.getProgramParameter(program, gl.LINK_STATUS))
    throw Error(gl.getProgramInfoLog(program));
  vertexBuffer = gl.createBuffer();
  hud = new HeadsetHUD({onAction: requestHudAction});
  hud.initGL(gl);
}

function requestHudAction(action) {
  if (action === "exit") { void stop("Operator exited AR"); return; }
  if (!hud) return;
  const view = hudView(hud.remote, hud.receivedAt, hud.client, performance.now());
  if (action.startsWith("return_") && (!view.allowed[action] || hudAction || ws?.readyState!==WebSocket.OPEN)) {
    const arm = view.homes.arms[action.split("_")[1]];
    const notice = !view.fresh || ws?.readyState!==WebSocket.OPEN ? "请恢复头显与 PC 的连接，再点回位" :
      hudAction || view.homes.busy ? "正在处理另一项操作，请等待结果" :
      (!arm.canReturn && !arm.can_clear_alarms && arm.blocked_reason) ||
      "请保持 AR 可见、保持手柄追踪并松开这一侧的中指 Grip，再点回位";
    hud.setClient({homeNotice:notice,homeNoticeUntil:performance.now()+6000});
    return;
  }
  if (ws?.readyState !== WebSocket.OPEN) return;
  if (!view.allowed[action] || (hudAction && action !== "stop")) return;
  if (ws.bufferedAmount > 4096 && action !== "stop") {
    hudNotice = "发送队列积压，暂不开始；请等待网络恢复";
    hudNoticeAt = performance.now(); return;
  }
  const id = `${Date.now().toString(36)}-${++hudActionSeq}`;
  const message = hud.actionMessage(action, id);
  if (!message) return;
  hudAction = {id, action, at: performance.now()};
  hudNotice = action === "arm" ? "开始请求已发送：请松开Grip并保持AR可见" :
    action === "gripper_power" ? "等待夹爪控制确认" :
    action === "stop" ? "正在请求停止并等待真实反馈" :
    action.startsWith("return_") ? "回位请求已发送；请保持AR可见并松开Grip，等待实测到位" :
    action.startsWith("calibration_") ? "标定请求已发送；等待后端采样与确认，不移动机械臂" :
    "正在查询机器人与逆解接口";
  hudNoticeAt = performance.now();
  ws.send(JSON.stringify(message));
}

function serialiseHand(source, frame) {
  const pose = source.gripSpace
    ? frame.getPose(source.gripSpace, reference)
    : null;
  const gp = source.gamepad;
  const targetRay = source.targetRaySpace ? frame.getPose(source.targetRaySpace, reference) : null;
  return {
    is_controller:
      !source.hand && source.targetRayMode === "tracked-pointer" && Boolean(gp),
    profiles: Array.from(source.profiles),
    target_ray: targetRay ? {
      quaternion_xyzw: [targetRay.transform.orientation.x,targetRay.transform.orientation.y,
        targetRay.transform.orientation.z,targetRay.transform.orientation.w],
      emulated_position: targetRay.emulatedPosition,
    } : null,
    pose: pose
      ? {
          position_m: [
            pose.transform.position.x,
            pose.transform.position.y,
            pose.transform.position.z,
          ],
          quaternion_xyzw: [
            pose.transform.orientation.x,
            pose.transform.orientation.y,
            pose.transform.orientation.z,
            pose.transform.orientation.w,
          ],
          emulated_position: pose.emulatedPosition,
        }
      : null,
    gamepad: gp
      ? {
          mapping: gp.mapping,
          connected: gp.connected,
          axes: Array.from(gp.axes),
          buttons: Array.from(gp.buttons, (b) => ({
            pressed: b.pressed,
            touched: b.touched,
            value: b.value,
          })),
        }
      : null,
  };
}

function packet(hands, frameTime) {
  return {
    schema: 1,
    input_policy: "continuous-pose-grip-v2",
    gripper_policy: "analog-trigger-ui-v1",
    type: "frame",
    seq: seq++,
    reference_space: referenceType,
    reference_epoch: referenceEpoch,
    visibility: session?.visibilityState || "hidden",
    xr_frame_time_ms: frameTime,
    client_send_time_ms: performance.now(),
    diagnostics: {
      requested_mode: requestedMode,
      dom_overlay: session?.domOverlayState?.type || null,
      document_visibility: document.visibilityState,
      document_has_focus: document.hasFocus(),
      network: {
        link_state: linkState,
        reconnects: reconnectCount,
        skipped_frames: skippedFrames,
        buffered_bytes: ws?.bufferedAmount ?? 0,
        http_rtt_ms: lastRttMs,
        http_rtt_age_ms: lastRttAt ? performance.now() - lastRttAt : null,
        last_close: lastClose,
        screen_wake_lock: wakeLockState,
      },
      input_sources: Array.from(session?.inputSources || [], (source) => ({
        handedness: source.handedness,
        profiles: Array.from(source.profiles),
        hand_tracking: Boolean(source.hand),
        has_gamepad: Boolean(source.gamepad),
      })),
    },
    hands,
  };
}
function send(data) {
  // Drop this frame if the transport backs up; never build a stale input queue.
  if (ws?.readyState === WebSocket.OPEN && ws.bufferedAmount < 16384)
    ws.send(JSON.stringify(data));
  else skippedFrames++;
}
function clearInput() {
  gripGate.reset();
  hud?.resetInteraction();
  if (referenceType)
    send(packet({ left: null, right: null }, performance.now()));
}

function sampleHud(frame, hands) {
  const viewer = frame.getViewerPose(reference);
  if (!viewer) return {viewer:null, rays:[]};
  const left = hands.left;
  if (hudAction && performance.now() - hudAction.at > 8000) {
    hudAction = null; hudNotice = "操作结果尚未确认，请查看状态；不会自动重试";
    hudNoticeAt = performance.now();
  }
  hud?.setClient({linkState, visibility: session.visibilityState,
    leftTracking: Boolean(left?.is_controller && left.pose && !left.pose.emulated_position),
    rightTracking: Boolean(hands.right?.is_controller && hands.right.pose && !hands.right.pose.emulated_position),
    rightGripHeld: gripGate.arms.right.held,
    gripHeld: gripGate.arms.left.held, rtt: lastRttMs,
    busy: Boolean(hudAction), notice: performance.now() - hudNoticeAt < 5000 ? hudNotice : ""});
  const rays = hud?.update(frame, reference, viewer, hands, performance.now()) || [];
  return {viewer, rays};
}

function draw(frame, hands, {viewer, rays}) {
  const layer = session.renderState.baseLayer;
  gl.bindFramebuffer(gl.FRAMEBUFFER, layer.framebuffer);
  const opaque = session.environmentBlendMode === "opaque";
  gl.clearColor(
    opaque ? 0.04 : 0,
    opaque ? 0.04 : 0,
    opaque ? 0.04 : 0,
    opaque ? 1 : 0,
  );
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
  if (!viewer) return;
  if (!session || shuttingDown) return;
  hud?.upload(performance.now());
  const vertices = [];
  for (const side of ["left", "right"]) {
    const h = hands[side];
    if (!h?.pose || !h.is_controller || h.pose.emulated_position) continue;
    const p = h.pose.position_m,
      q = h.pose.quaternion_xyzw,
      color = side === "left" ? [1, 1, 1] : [0.6, 0.6, 0.6];
    vertices.push(...p, ...color);
    // A short line along the measured controller's local -Z shows orientation.
    const direction = [
      -2 * (q[0] * q[2] + q[3] * q[1]),
      -2 * (q[1] * q[2] - q[3] * q[0]),
      -(1 - 2 * (q[0] * q[0] + q[1] * q[1])),
    ];
    vertices.push(...p.map((x, i) => x + 0.08 * direction[i]), ...color);
  }
  for (const ray of rays) {
    const color = ray.hit ? [0.35, 1.0, 0.8] : [0.45, 0.65, 0.8];
    vertices.push(...ray.origin, ...color, ...ray.end, ...color);
  }
  gl.bindBuffer(gl.ARRAY_BUFFER, vertexBuffer);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(vertices), gl.DYNAMIC_DRAW);
  for (const view of viewer.views) {
    const v = layer.getViewport(view);
    gl.viewport(v.x, v.y, v.width, v.height);
    hud?.draw(view);
    gl.useProgram(program);
    gl.bindBuffer(gl.ARRAY_BUFFER, vertexBuffer);
    for (const [name, offset] of [["aPosition", 0], ["aColor", 12]]) {
      const index = gl.getAttribLocation(program, name);
      gl.enableVertexAttribArray(index); gl.vertexAttribPointer(index, 3, gl.FLOAT, false, 24, offset);
    }
    gl.uniformMatrix4fv(
      gl.getUniformLocation(program, "uProjection"),
      false,
      view.projectionMatrix,
    );
    gl.uniformMatrix4fv(
      gl.getUniformLocation(program, "uView"),
      false,
      view.transform.inverse.matrix,
    );
    gl.drawArrays(gl.LINES, 0, vertices.length / 6);
    gl.drawArrays(gl.POINTS, 0, vertices.length / 6);
  }
  return rays;
}

function frameLoop(t, frame) {
  if (!session || shuttingDown) return;
  session.requestAnimationFrame(frameLoop);
  try {
    const hands = { left: null, right: null };
    if (session.visibilityState === "visible")
      for (const source of session.inputSources) {
        if (source.handedness === "right" || source.handedness === "left")
          hands[source.handedness] = serialiseHand(source, frame);
      }
    const gated = gripGate.update(hands, session.visibilityState === "visible");
    const sampled = sampleHud(frame, gated.hands);
    const rays = sampled.rays;
    if (!session || shuttingDown) return;
    let captureChanged = false;
    for (const side of ["left","right"]) {
      const hand = gated.hands[side];
      const trigger = sampled.viewer && hand?.tracking_valid ? hand.gamepad?.buttons?.[0]?.value : undefined;
      const claim = rays.find(r => r.side === side)?.claim ?? hud?.triggerOwnership.sample(side, trigger, null);
      if (hand) hand.gripper_ui_blocked = claim?.blocked ?? true;
      captureChanged ||= Boolean(claim?.sendNow);
    }
    if (captureChanged || gated.changed || t - sentAt >= 1000 / 30) {
      send(packet(gated.hands, t));
      sentAt = t;
    }
    draw(frame, hands, sampled);
    if (!session || shuttingDown) return;
    if (t - lastUiAt > 250) {
      status.textContent = `${linkState} · HTTP round trip: ${lastRttMs === null ? "unavailable" : Math.round(lastRttMs) + " ms"}\n${lastInputError}\nXR focus: ${session.visibilityState}${session.visibilityState !== "visible" ? " (input paused by runtime)" : ""}\nLeft: ${hands.left?.pose && !hands.left.pose.emulated_position ? "tracked" : "not tracked"} | Right: ${hands.right?.pose && !hands.right.pose.emulated_position ? "tracked" : "not tracked"}\n${referenceType}, origin epoch ${referenceEpoch}`;
      lastUiAt = t;
    }
  } catch (error) {
    stop(`Input error: ${error.message}`);
  }
}

async function stop(reason) {
  if (shuttingDown) return;
  const expectedSession = session;
  if (typeof reason === "string") lastInputError = reason;
  shuttingDown = true;
  clearInput();
  if (session) {
    try {
      await session.end();
    } catch {}
  }
  if (session === expectedSession || session === null) cleanup();
}
function cleanup() {
  linkGeneration++;
  clearTimeout(reconnectTimer);
  clearInterval(rttTimer);
  reconnectTimer = rttTimer = null;
  linkState = "DISCONNECTED";
  if (screenWakeLock) {
    const lock = screenWakeLock;
    screenWakeLock = null;
    void lock.release().catch(() => {});
  }
  wakeLockState = "RELEASED";
  gripGate.reset();
  hud?.resetInteraction();
  if (hud) hud.panel = null;
  hudAction = null;
  if (ws) {
    const old = ws;
    ws = null;
    old.close();
  }
  session = null;
  reference = null;
  endButton.disabled = true;
  shuttingDown = false;
  for (const button of starts)
    button.disabled = button.dataset.supported !== "true";
  status.textContent = lastInputError || "Input session ended. The PC will clear stale data.";
}

async function requestScreenWakeLock() {
  if (!session || shuttingDown || document.visibilityState !== "visible" || screenWakeLock) return;
  if (!navigator.wakeLock?.request) { wakeLockState = "UNSUPPORTED"; return; }
  const expectedSession = session;
  try {
    const lock = await navigator.wakeLock.request("screen");
    if (session !== expectedSession || shuttingDown) { await lock.release(); return; }
    screenWakeLock = lock;
    wakeLockState = "ACTIVE_WHILE_VISIBLE";
    lock.addEventListener("release", () => {
      if (screenWakeLock === lock) { screenWakeLock = null; wakeLockState = "RELEASED_BY_SYSTEM"; }
    });
  } catch (error) { wakeLockState = `UNAVAILABLE: ${error.name}`; }
}
document.addEventListener("visibilitychange", () => {
  if (document.visibilityState === "visible") void requestScreenWakeLock();
});

function scheduleReconnect(expectedSession) {
  if (reconnectTimer || session !== expectedSession || !session || shuttingDown) return;
  gripGate.reset();
  linkState = "RECONNECTING_INPUT_ONLY";
  const delay = Math.min(500 * 2 ** Math.min(reconnectAttempt++, 4), 5000);
  reconnectTimer = setTimeout(() => {
    reconnectTimer = null;
    void connectInput(expectedSession);
  }, delay);
}

async function measureRtt() {
  if (!session || ws?.readyState !== WebSocket.OPEN || rttInFlight) return;
  rttInFlight = true;
  const expectedSession = session, started = performance.now();
  try {
    const response = await fetch("/api/session", { cache: "no-store", signal: AbortSignal.timeout(1500) });
    if (!response.ok) throw Error(`HTTP ${response.status}`);
    if (session === expectedSession) lastRttMs = performance.now() - started;
  } catch {
    if (session === expectedSession) lastRttMs = null;
  } finally {
    lastRttAt = performance.now();
    rttInFlight = false;
  }
}

async function connectInput(expectedSession) {
  if (!expectedSession || session !== expectedSession || shuttingDown) return false;
  const generation = ++linkGeneration;
  let socket = null;
  gripGate.reset();
  linkState = "CONNECTING";
  try {
    const response = await fetch("/api/session", { cache: "no-store", signal: AbortSignal.timeout(1500) });
    if (!response.ok) {
      if (response.status === 401 || response.status === 403) {
        if (session === expectedSession) await stop("Pairing expired. Return to the address root and pair again.");
        return false;
      }
      throw Error(`Session request HTTP ${response.status}`);
    }
    const info = await response.json();
    if (session !== expectedSession || generation !== linkGeneration || shuttingDown) return false;
    socket = new WebSocket(`${location.protocol === "https:" ? "wss:" : "ws:"}//${location.host}/ws/quest?token=${encodeURIComponent(info.token)}`);
    ws = socket;
    await new Promise((resolve, reject) => {
      let opened = false;
      const openTimeout = setTimeout(() => reject(Error("WebSocket open timeout")), 3000);
      socket.onopen = () => { clearTimeout(openTimeout); opened = true; resolve(); };
      socket.onerror = () => { if (!opened) { clearTimeout(openTimeout); reject(Error("WebSocket connection failed")); } };
      socket.onmessage = event => {
        if (ws !== socket || session !== expectedSession) return;
        try {
          const data = JSON.parse(event.data);
          if (data.type === "teleop_status") hud?.acceptState(data);
          else if (data.type === "action_result") {
            if (data.id === hudAction?.id) {
              hudAction = null; hudNotice = data.message || (data.ok ? "操作完成" : "操作未通过");
              hudNoticeAt = performance.now();
            }
          } else if (data.error) void stop(`PC rejected input: ${data.error}`);
        } catch (error) { void stop(`Invalid PC response: ${error.message}`); }
      };
      socket.onclose = event => {
        clearTimeout(openTimeout);
        if (ws !== socket || session !== expectedSession || generation !== linkGeneration) return;
        ws = null;
        hudAction = null;
        lastClose = { code: event.code, reason: event.reason || "", clean: event.wasClean };
        gripGate.reset();
        if (event.code === 1007 || event.code === 1008) {
          void stop(`Input rejected (${event.code}): ${event.reason || "Check input format"}`);
          if (!opened) reject(Error(`Rejected before opening (${event.code})`));
        } else if (opened) {
          lastInputError = `Link interrupted (${event.code}). Release Grip; robot re-arming is required after reconnect.`;
          scheduleReconnect(expectedSession);
        } else reject(Error(`WebSocket closed before opening (${event.code})`));
      };
    });
    if (session !== expectedSession || generation !== linkGeneration || shuttingDown) {
      socket.close();
      return false;
    }
    if (reconnectAttempt) reconnectCount++;
    reconnectAttempt = 0;
    linkState = "CONNECTED";
    gripGate.reset();
    void measureRtt();
    return true;
  } catch (error) {
    if (session !== expectedSession || generation !== linkGeneration || shuttingDown) return false;
    if (ws === socket) ws = null;
    socket?.close();
    lastInputError = `Link unavailable: ${error.message}. Retrying input only.`;
    scheduleReconnect(expectedSession);
    return false;
  }
}

async function start(mode) {
  gripGate.reset();
  lastInputError = "";
  reconnectAttempt = reconnectCount = skippedFrames = 0;
  starts.forEach((b) => (b.disabled = true));
  shuttingDown = false;
  try {
    // Called immediately from the user's click, before any network await.
    requestedMode = mode;
    const options =
      mode === "immersive-vr"
        ? { optionalFeatures: ["local-floor"] }
        : {
            optionalFeatures: ["local-floor", "dom-overlay"],
            domOverlay: { root: document.querySelector("#overlay") },
          };
    session = await navigator.xr.requestSession(mode, options);
    const expectedSession = session;
    void requestScreenWakeLock();
    session.addEventListener("end", () => { if (session === expectedSession) cleanup(); }, { once: true });
    session.addEventListener("visibilitychange", () => {
      if (session === expectedSession) {
        clearInput();
        if (session.visibilityState === "visible") void requestScreenWakeLock();
      }
    });
    if (!gl) makeRenderer();
    await gl.makeXRCompatible();
    session.updateRenderState({
      baseLayer: new XRWebGLLayer(session, gl, { alpha: true }),
    });
    try {
      reference = await session.requestReferenceSpace("local-floor");
      referenceType = "local-floor";
    } catch {
      reference = await session.requestReferenceSpace("local");
      referenceType = "local";
    }
    referenceEpoch = 0;
    reference.addEventListener("reset", () => {
      referenceEpoch++;
      if (hud) hud.panel = null;
      clearInput();
    });
    await connectInput(expectedSession);
    if (session !== expectedSession) return;
    rttTimer = setInterval(measureRtt, 2000);
    seq = 0;
    sentAt = 0;
    endButton.disabled = false;
    session.requestAnimationFrame(frameLoop);
  } catch (error) {
    const message = error.message;
    await stop();
    status.textContent = `Unable to start: ${message}`;
  }
}

starts[0].addEventListener("click", () => start("immersive-ar"));
starts[1].addEventListener("click", () => start("immersive-vr"));
endButton.addEventListener("click", stop);
window.addEventListener("pagehide", stop);
async function initialise() {
  if (!window.isSecureContext || !navigator.xr) {
    status.textContent =
      "Open this page in Quest Browser via USB forwarding at http://localhost:8765/quest. WebXR is unavailable in this browser/context.";
    return;
  }
  const support = [];
  status.textContent = "正在加载官方 E6 外观模型，完成后可进入 AR…";
  await preloadRobotVisual(); // Parse visual assets before a live XR input loop starts.
  for (const [i, mode] of ["immersive-ar", "immersive-vr"].entries()) {
    let supported = false;
    try {
      supported = await navigator.xr.isSessionSupported(mode);
    } catch (error) {
      console.warn(`${mode}: ${error.message}`);
    }
    support.push(supported);
    starts[i].dataset.supported = String(supported);
    starts[i].disabled = !supported;
  }
  status.textContent = support.some(Boolean)
    ? `Ready. AR supported: ${support[0]}. VR supported: ${support[1]}. 两手射线都可用食指点击。单右臂：右食指控制夹爪，右中指 Grip 控制跟随；也可按右手 A 点击按钮。`
    : "No immersive XR device is available in this browser. Open this address in the Quest headset's own Browser app. A Windows browser or a mirrored PC desktop cannot read the Quest controllers here.";
}
initialise().catch((error) => {
  status.textContent = error.message;
});
