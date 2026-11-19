// Synthetic unit inputs, never connected to the live observer or hardware.
import assert from "node:assert/strict";
import { GripGate } from "../web/grip_gate.mjs";
const gate = new GripGate();
const sample = value => ({ left: { is_controller: true,
  pose: { position_m: [0, 1, 0], quaternion_xyzw: [0, 0, 0, 1], emulated_position: false },
  gamepad: { connected: true, mapping: "xr-standard", buttons: [{value: 0}, {value}] }
}, right: null });
let blocked = gate.update(sample(1), true);
assert.ok(blocked.hands.left.pose, "startup retains real pose");
assert.equal(blocked.hands.left.grip_held, false, "startup held cannot permit motion");
assert.ok(gate.update(sample(0), true).hands.left.pose, "released still carries real pose");
let result = gate.update(sample(1), true);
assert.equal(result.active, true);
assert.ok(result.hands.left.pose);
assert.equal(gate.update(sample(0.4), true).hands.left.grip_held, true);
result = gate.update(sample(0), true);
assert.equal(result.changed, true, "release bypasses rate timer");
assert.ok(result.hands.left.pose, "release does not hide or fabricate measurements");
assert.equal(result.hands.left.grip_held, false);
gate.update(sample(1), true);
assert.equal(gate.update(sample(1), false).hands.left.grip_held, false);
assert.equal(gate.update(sample(1), true).hands.left.grip_held, false, "visibility recovery requires release");
gate.update(sample(0), true);
assert.ok(gate.update(sample(1), true).hands.left.pose);
gate.reset();
assert.equal(gate.update(sample(1), true).hands.left.grip_held, false, "origin reset requires release");
console.log("PASS: idle/press/release/hysteresis/focus loss/origin reset");
