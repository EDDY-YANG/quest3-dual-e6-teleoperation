// Manufacturer URDF visuals only. Never supplies targets to the control system.
import * as THREE from "./robot-assets/three/three.module.js";
import {loadArmAssembly} from "./asset_store.mjs";
import {mountGripper} from "./gripper_model.mjs";
import {robotViewState} from "./robot_view.mjs";
export const SIDE_MODEL_WIDTH = 640, SIDE_MODEL_HEIGHT = 900;
const AXIS_COLORS = [0xed615e, 0x47bd83, 0x428aff];
let assetsPromise;

export function layoutToolLabels(tcp, projectedAxes, measure = () => 90) {
  const entries = [{label: "TCP", anchor: tcp, color: "#ffffff", offset: [0, 34]},
    ...["Z", "X", "Y"].map(axis => ({label: `工具 ${axis}`, anchor: projectedAxes[axis],
      color: `#${AXIS_COLORS["XYZ".indexOf(axis)].toString(16).padStart(6, "0")}`,
      offset: axis === "Z" ? [0, -20] : axis === "X" ? [-50, 5] : [50, 24]}))];
  if (entries.some(e => !Array.isArray(e.anchor) || e.anchor.length !== 2 ||
      !e.anchor.every(Number.isFinite))) return [];
  const placed = [];
  for (const entry of entries) {
    const width = Math.max(70, measure(entry.label) + 8), candidates = [entry.offset];
    for (const radius of [32, 58, 86, 120, 158]) candidates.push(
      [0, -radius], [0, radius], [-radius, 0], [radius, 0],
      [-radius, -radius], [radius, -radius], [-radius, radius], [radius, radius]);
    const options = candidates.map(([dx, dy]) => {
      const x = Math.max(width / 2 + 12, Math.min(SIDE_MODEL_WIDTH - width / 2 - 12, entry.anchor[0] + dx));
      const y = Math.max(142, Math.min(785, entry.anchor[1] + dy));
      const box = [x - width / 2 - 4, y - 29, x + width / 2 + 4, y + 9];
      const overlap = placed.reduce((sum, p) => sum +
        Math.max(0, Math.min(box[2], p.box[2]) - Math.max(box[0], p.box[0])) *
        Math.max(0, Math.min(box[3], p.box[3]) - Math.max(box[1], p.box[1])), 0);
      return {...entry, x, y, box, score: overlap * 1000 +
        Math.hypot(x - entry.anchor[0] - entry.offset[0], y - entry.anchor[1] - entry.offset[1])};
    });
    options.sort((a, b) => a.score - b.score); placed.push(options[0]);
  }
  return placed;
}

export function preloadE6Assets() {
  if (!assetsPromise) assetsPromise = loadArmAssembly("left").then(a => a.arm).catch(e => { assetsPromise=null; throw e; });
  return assetsPromise;
}

function origin(object, xyz, rpy) {
  object.position.fromArray(xyz);
  object.quaternion.setFromEuler(new THREE.Euler(...rpy, "ZYX"));
}

export function makeE6Graph(definition, geometry = new Map()) {
  const links = new Map(definition.links.map(link => [link.name, new THREE.Group()]));
  const materials = [], motions = new Map();
  for (const link of definition.links) {
    const group = links.get(link.name); group.name = link.name;
    for (const visual of link.visuals) {
      const g = geometry.get(visual.mesh);
      if (!g) continue;
      const material = new THREE.MeshStandardMaterial({color: 0xe2e5e8, roughness: .37, metalness: .18});
      materials.push(material);
      const mesh = new THREE.Mesh(g, material); origin(mesh, visual.xyz, visual.rpy);
      mesh.scale.fromArray(visual.scale); mesh.castShadow = mesh.receiveShadow = true;
      group.add(mesh);
    }
  }
  for (const joint of definition.joints) {
    const fixed = new THREE.Group(), motion = new THREE.Group();
    origin(fixed, joint.xyz, joint.rpy);
    links.get(joint.parent).add(fixed); fixed.add(motion); motion.add(links.get(joint.child));
    if (joint.type === "revolute") motions.set(joint.name, {motion, axis: new THREE.Vector3(...joint.axis).normalize()});
  }
  const root = links.get(definition.root_link);
  const setJoints = q => {
    if (!Array.isArray(q) || q.length !== 6 || !q.every(Number.isFinite)) return false;
    for (let i = 0; i < 6; i++) {
      const joint = motions.get(`joint${i + 1}`);
      joint.motion.quaternion.setFromAxisAngle(joint.axis,
        (q[i] + definition.controller_joint_offsets_deg[i]) * Math.PI / 180);
    }
    root.updateMatrixWorld(true); return true;
  };
  const nominalTCP = () => definition.link6_to_tool0_display_transform ? links.get("Link6").matrixWorld.clone().multiply(
    new THREE.Matrix4().set(...definition.link6_to_tool0_display_transform.flat())) : null;
  return {root, links, materials, setJoints, nominalTCP};
}

function arrow(length, color, radius = .003) {
  const group = new THREE.Group(), material = new THREE.MeshBasicMaterial({color, depthTest: false});
  const head = Math.min(.027, length * .25);
  const shaft = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, length - head, 12), material);
  shaft.position.y = (length - head) / 2;
  const cone = new THREE.Mesh(new THREE.ConeGeometry(radius * 3, head, 16), material);
  cone.position.y = length - head / 2; group.add(shaft, cone); group.renderOrder = 10;
  return group;
}

function axes(lengths) {
  const group = new THREE.Group();
  for (const [index, color] of AXIS_COLORS.entries()) {
    const a = arrow(lengths[index], color, index === 2 ? .005 : .003);
    a.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0),
      new THREE.Vector3(...[0, 1, 2].map(j => Number(j === index)))); group.add(a);
  }
  return group;
}

function setPose(object, pose) {
  object.position.set(pose[0] / 1000, pose[1] / 1000, pose[2] / 1000);
  object.quaternion.setFromEuler(new THREE.Euler(...pose.slice(3).map(x => x * Math.PI / 180), "ZYX"));
}

function sceneFor(assembly) {
  const assets = assembly.arm;
  const scene = new THREE.Scene(), graph = makeE6Graph(assets.definition, assets.geometry);
  const gripper = mountGripper(graph, assembly);
  scene.add(graph.root);
  scene.add(new THREE.HemisphereLight(0xffffff, 0x505966, 2.2));
  const key = new THREE.DirectionalLight(0xffffff, 3.3); key.position.set(1, -1.2, 1.8);
  key.castShadow = true; key.shadow.mapSize.set(1024, 1024);
  Object.assign(key.shadow.camera, {left: -.7, right: .7, top: .9, bottom: -.7, near: .1, far: 5});
  key.shadow.bias = -.001; scene.add(key);
  const rim = new THREE.DirectionalLight(0xdce7ff, 2); rim.position.set(-1, .6, 1.2); scene.add(rim);
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(1.8, 1.8), new THREE.ShadowMaterial({opacity: .22}));
  floor.position.z = -.002; floor.receiveShadow = true; scene.add(floor);
  const baseAxes = axes([.14, .14, .14]); scene.add(baseAxes);
  const tcp = axes([.085, .085, .17]);
  tcp.add(new THREE.Mesh(new THREE.SphereGeometry(.009, 18, 12), new THREE.MeshBasicMaterial({color: 0xffffff, depthTest: false})));
  scene.add(tcp);
  const command = new THREE.Mesh(new THREE.SphereGeometry(.014, 16, 12),
    new THREE.MeshBasicMaterial({color: 0xffc85b, wireframe: true, depthTest: false}));
  const requested = new THREE.Mesh(new THREE.OctahedronGeometry(.012),
    new THREE.MeshBasicMaterial({color: 0x9a9da4, wireframe: true, depthTest: false}));
  scene.add(command, requested);
  const camera = new THREE.OrthographicCamera(-.37, .37, .52, -.52, .01, 10);
  camera.up.set(0, 0, 1); camera.position.set(.65, -1.8, .57); camera.lookAt(0, 0, .31);
  return {scene, graph, gripper, referenceJoints:assets.descriptor.display_reference_joints_deg, camera, tcp, command, requested};
}

export function splitArmViews(view) {
  const side=view.teleopSide || view.teleop_side || view.control?.side || "left";
  return ["left","right"].map(arm=>({fresh:view.fresh,
    robot:view.robots?.[arm] || (side===arm?view.robot:{}),
    control:view.controls?.[arm] || (side===arm?view.control:{})}));
}

export class E6SideModels {
  constructor({leftCanvas = null, rightCanvas = null} = {}) {
    this.leftCanvas = leftCanvas || document.createElement("canvas");
    this.rightCanvas = rightCanvas || document.createElement("canvas");
    for (const c of [this.leftCanvas, this.rightCanvas]) { c.width = SIDE_MODEL_WIDTH; c.height = SIDE_MODEL_HEIGHT; }
    this.lastPaint = -Infinity; this.error = null; this.ready = false;
    this.loading = this.initialize();
  }
  async initialize() {
    try {
      const assemblies = await Promise.all(["left","right"].map(loadArmAssembly));
      const canvas = document.createElement("canvas");
      this.renderer = new THREE.WebGLRenderer({canvas, alpha: true, antialias: true, preserveDrawingBuffer: true});
      this.renderer.setSize(SIDE_MODEL_WIDTH * 2, SIDE_MODEL_HEIGHT, false);
      this.renderer.setClearColor(0xffffff, 0); this.renderer.shadowMap.enabled = true;
      this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
      this.renderer.toneMapping = THREE.ACESFilmicToneMapping; this.renderer.toneMappingExposure = 1.05;
      this.sides = assemblies.map(sceneFor); this.ready = true;
    } catch (error) { this.error = error.message; }
    this.lastPaint = -Infinity;
  }
  paint(view, now, force = false) {
    if (!force && now - this.lastPaint < 100) return false;
    this.lastPaint = now;
    const views = splitArmViews(view);
    const states = views.map(robotViewState);
    if (this.ready) {
      this.renderer.setScissorTest(true);
      for (let i = 0; i < 2; i++) {
        const s = this.sides[i], data = states[i];
        s.graph.setJoints(data.valid ? data.joints : s.referenceJoints);
        const toolState = view.grippers?.arms?.[i ? "right" : "left"];
        if (view.fresh && toolState?.model_pose) s.gripper.setPose(toolState.model_pose);
        for (const material of s.graph.materials) material.color.setHex(data.valid ? 0xe5e8ec : 0x949ba4);
        s.tcp.visible = data.valid;
        if (data.valid) setPose(s.tcp, data.pose);
        for (const [marker, pose] of [[s.command, data.command], [s.requested, data.requested]]) {
          marker.visible = Boolean(data.valid && pose); if (marker.visible) setPose(marker, pose);
        }
        this.renderer.setViewport(i * SIDE_MODEL_WIDTH, 0, SIDE_MODEL_WIDTH, SIDE_MODEL_HEIGHT);
        this.renderer.setScissor(i * SIDE_MODEL_WIDTH, 0, SIDE_MODEL_WIDTH, SIDE_MODEL_HEIGHT);
        this.renderer.clear(true, true, true); this.renderer.render(s.scene, s.camera);
      }
      this.renderer.setScissorTest(false);
    }
    for (const [i, canvas] of [this.leftCanvas, this.rightCanvas].entries()) {
      const c = canvas.getContext("2d"), state = states[i], side = i ? "右臂" : "左臂";
      c.clearRect(0, 0, SIDE_MODEL_WIDTH, SIDE_MODEL_HEIGHT);
      if (this.ready) c.drawImage(this.renderer.domElement, i * SIDE_MODEL_WIDTH, 0,
        SIDE_MODEL_WIDTH, SIDE_MODEL_HEIGHT, 0, 0, SIDE_MODEL_WIDTH, SIDE_MODEL_HEIGHT);
      const text = (str, y, size = 27, color = "#fff", x = 320) => {
        c.textAlign = "center"; c.font = `600 ${size}px system-ui,sans-serif`;
        c.lineWidth = 6; c.lineJoin = "round"; c.strokeStyle = "rgba(0,0,0,.8)";
        c.strokeText(str, x, y); c.fillStyle = color; c.fillText(str, x, y);
      };
      text(`${side} E6`, 45, 38);
      text(state.valid ? "实测关节姿态 · 固定侧视" : "无有效反馈 · 静态参考姿态", 85, 25, state.valid ? "#fff" : "#f1ce8c");
      const tool = view.fresh ? view.grippers?.arms?.[i ? "right" : "left"] : null;
      const gap = typeof tool?.model_gap_mm === "number" ? tool.model_gap_mm.toFixed(1) + " mm" : "—";
      const drive = tool?.hardware_rearm_required ? "Uno · 待重新授权" : tool?.drive?.mode === "blocked" ? "串口配置错误" :
        tool?.drive?.mode === "uno" ? "Uno · "+tool.drive.status : "虚拟";
      const targetGap = Number.isFinite(tool?.target_gap_mm) ? tool.target_gap_mm.toFixed(1)+" mm" : gap;
      const feedback = Number.isFinite(tool?.measured_gap_mm) ? "实测 "+tool.measured_gap_mm.toFixed(1)+" mm" : "无实测开度";
      text(`${drive} · 夹爪目标 ${targetGap} · ${feedback}`, 116, 21, "#9ce5df");
      if (tool?.independent_trigger) {
        const t = Number.isFinite(tool.trigger) ? Math.round(100*tool.trigger)+"%" : "—";
        text(`食指 ${t} · 目标 ${tool.target_pulse_us??"—"} / ACK ${tool.drive?.ack_us??"—"} µs`, 144, 20, "#9ce5df");
      }
      if (!this.ready) { text(this.error ? "模型未能加载" : "正在加载官方 E6 外观…", 430); if (this.error) text("连接与控制状态请看中央面板", 470, 20); }
      if (this.ready && state.valid) {
        const project = vector => { const p = vector.project(this.sides[i].camera); return [
          Math.max(65, Math.min(575, (p.x + 1) * SIDE_MODEL_WIDTH / 2)),
          Math.max(130, Math.min(795, (1 - p.y) * SIDE_MODEL_HEIGHT / 2))]; };
        const p = project(new THREE.Vector3(...state.pose.slice(0, 3)).multiplyScalar(.001));
        const projectedAxes = Object.fromEntries(["X", "Y", "Z"].map((axis, index) => {
          const endpoint = new THREE.Vector3(); endpoint.setComponent(index, index === 2 ? .17 : .085);
          return [axis, project(this.sides[i].tcp.localToWorld(endpoint))];
        }));
        c.font = "600 25px system-ui,sans-serif";
        for (const label of layoutToolLabels(p, projectedAxes, value => c.measureText(value).width)) {
          c.beginPath(); c.strokeStyle = label.color; c.lineWidth = 2;
          c.moveTo(...label.anchor); c.lineTo(label.x, label.y - 12); c.stroke();
          text(label.label, label.y, label.label === "TCP" ? 26 : 25, label.color, label.x);
        }
      }
      text(state.valid ? "工具 X 红 / Y 绿 / Z 蓝 · 随末端旋转" : "未显示实测 TCP，不代表机器人当前位置", 825, 23);
      text("基座 X 红 / Y 绿 / Z 蓝", 855, 21);
      text("名义 URDF 参考视图 · 未与实物配准", 883, 19, "#dce0e4");
    }
    return true;
  }
}
