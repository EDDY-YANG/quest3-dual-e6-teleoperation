import assert from "node:assert/strict";
import {readFile} from "node:fs/promises";
import * as THREE from "../web/robot-assets/three/three.module.js";
import {GLTFLoader} from "../web/robot-assets/three/GLTFLoader.js";
import {makeE6Graph} from "../web/e6_urdf_view.mjs";
import {mountGripper} from "../web/gripper_model.mjs";
import {TriggerOwnership} from "../web/gripper_ui_input.mjs";
const root=new URL("../web/",import.meta.url);
const file=p=>readFile(new URL(p,root));
const json=async p=>JSON.parse(await file(p));
const catalog=await json("asset_catalog.json");
const bin=await file("robot-assets/gripper-v1/gripper.glb");
const {scene}=await new GLTFLoader().parseAsync(bin.buffer.slice(bin.byteOffset,bin.byteOffset+bin.byteLength),"");
const mechanism=await json("robot-assets/gripper-v1/mechanism.json");
const definition=await json("robot-assets/e6/model.json"); const refs={};
for(const side of ["left","right"]){
  const arm=makeE6Graph(definition);const gripper=mountGripper(arm,{side,tool:{scene,mechanism},mount:catalog.instances[side].mount});
  arm.setJoints(catalog.assets.e6.display_reference_joints_deg);
  assert.equal(gripper.root.parent,arm.links.get("Link6"));
  assert.equal(arm.root.getObjectByName(side+"_gripper"),gripper.root);
  assert.equal(gripper.groups.fixed.children.length+gripper.groups.crank.children.length+gripper.groups.slide_negative.children.length+gripper.groups.slide_positive.children.length+gripper.groups.rod_negative.children.length+gripper.groups.rod_positive.children.length,28);
  const p0=new THREE.Vector3().setFromMatrixPosition(gripper.root.matrixWorld);const flange=gripper.root.parent.localToWorld(new THREE.Vector3(0,0,.019));assert.ok(flange.distanceTo(p0)<1e-9);
  arm.setJoints([5,-13,41,7,-21,72]);const p1=new THREE.Vector3().setFromMatrixPosition(gripper.root.matrixWorld);assert.ok(p0.distanceTo(p1)>.01);assert.ok(gripper.root.parent.localToWorld(new THREE.Vector3(0,0,.019)).distanceTo(p1)<1e-9);
  refs[side]=gripper;
}
assert.notEqual(refs.left.groups.slide_negative,refs.right.groups.slide_negative);
refs.left.setPose({crank_rad:1,slide_mm:19,rod_rad:.2,rod_offset_mm:[1,-6]});assert.equal(refs.left.groups.slide_negative.position.x,-.019);assert.equal(refs.right.groups.slide_negative.position.x,0);
const gate=new TriggerOwnership();assert.ok(gate.update("left",1,{hit:false}));assert.equal(gate.update("left",0,{hit:false}),false);assert.ok(gate.update("left",.01,{hit:true}));assert.ok(gate.update("left",.9,{hit:false}));assert.ok(gate.update("right",1,null));assert.equal(gate.update("left",0,{hit:false}),false);
console.log("PASS: real GLB, 28 occurrences, two independently articulated instances, moving Link6 and UI capture/release.");

