import assert from "node:assert/strict";
import {readFile} from "node:fs/promises";
import * as THREE from "../web/robot-assets/three/three.module.js";
import {GLTFLoader} from "../web/robot-assets/three/GLTFLoader.js";
import {makeE6Graph} from "../web/e6_urdf_view.mjs";
import {mountGripper} from "../web/gripper_model.mjs";
import {TriggerOwnership} from "../web/gripper_ui_input.mjs";
import {HeadsetHUD} from "../web/quest_hud.mjs";
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
const gate = new TriggerOwnership();
const outside = {hit:false};
const stopButton = {hit:true,button:"stop"};
const homeButton = {hit:true,button:"return_right_start"};
assert.equal(gate.update("right",1,outside),true);  // A held trigger at connection cannot start a stroke.
gate.sample("right",0,outside);
for (const [value,ray] of [[.12,outside],[.8,stopButton],[1,homeButton],[.8,null],[.2,homeButton],[.025,homeButton],[0,homeButton]]) {
  const sample=gate.sample("right",value,ray);
  assert.equal(sample.blocked,false);
  assert.equal(sample.click,null);
  if (value===0) assert.equal(sample.sendNow,true);
}
// A new press on a button belongs to that button; it must not operate the jaw.
assert.equal(gate.sample("right",.2,stopButton).blocked,true);
assert.equal(gate.sample("right",.8,stopButton).click,"stop");
assert.deepEqual(gate.sample("right",1,outside),{blocked:true,click:null,sendNow:false});
assert.equal(gate.sample("right",.8,homeButton).click,null);
gate.sample("right",0,outside);
gate.sample("right",.2,stopButton);
assert.equal(gate.sample("right",.8,homeButton).click,null);
// Independent left-hand menu access persists during a right-hand grasp.
gate.sample("right",0,outside); gate.sample("right",.6,outside);
gate.sample("left",0,homeButton);
assert.equal(gate.sample("left",1,homeButton).click,"return_right_start");
assert.equal(gate.sample("right",1,homeButton).blocked,false);
assert.equal(gate.sample("right",undefined,null).blocked,true);
assert.equal(gate.sample("right",.8,homeButton).click,null);
assert.equal(gate.sample("right",.8,outside).blocked,true);
console.log("PASS: gripper assembly, jaw release across HUD, UI click ownership, independent hands and tracking loss.");

// One right controller can perform every action. A second cursor, when present,
// uses the same buttons without changing which arm or jaw the right hand drives.
const events=[];
const hud=Object.assign(Object.create(HeadsetHUD.prototype),{
  triggerOwnership:new TriggerOwnership(), stick:{}, history:[], announcer:{supported:false},
  client:{linkState:"CONNECTED",visibility:"visible",rightTracking:true,calibrationSide:"left"},
  panel:{center:[0,0,-1],right:[1,0,0],up:[0,1,0],normal:[0,0,1],width:1.28,height:.8},
  onAction:id=>events.push(id)
});
hud.acceptState({type:"teleop_status",schema:1,teleop_side:"right",group:{participants:["right"],connected_sides:["right"]},
  capabilities:{prepare:true,stop:true},grippers:{arms:{right:{independent_trigger:true}}}},100);
assert.equal(hud.singleRightArm,true);
assert.equal(hud.client.calibrationSide,"right");
const buttons=Array.from({length:6},()=>({value:0,pressed:false}));
const source={handedness:"right",targetRaySpace:{},gamepad:{buttons}};
const transform={position:{x:.12,y:-.29,z:0},orientation:{x:0,y:0,z:0,w:1}};
const frame={session:{inputSources:[source]},getPose:()=>({transform})};
const hands={right:{is_controller:true,tracking_valid:true,pose:{emulated_position:false}}};
function rightInput(value, aDown) {
  buttons[0]={value,pressed:value>=.7};
  buttons[4]={value:Number(aDown),pressed:aDown};
  return hud.update(frame,{}, {transform},hands,100)[0].claim;
}
assert.equal(rightInput(1,true).blocked,true); // already down at connection
transform.position.x=0; transform.position.y=0; // Start over panel text.
for (const value of [0,.25,.5,1,.4,.025,0]) {
  if (value>=.5) { transform.position.x=.12; transform.position.y=-.29; }
  const claim=rightInput(value,false);
  assert.equal(claim.blocked,false);
  assert.equal(claim.click,null);
}
assert.deepEqual(events,[]);
assert.equal(rightInput(.25,false).blocked,false);
const triggerClick=rightInput(1,false);
assert.equal(triggerClick.click,"stop");
assert.equal(triggerClick.blocked,false); // The analog trigger still reaches the jaw.
rightInput(0,false);
assert.equal(rightInput(0,true).click,"stop");  // Right thumb can click with the jaw open.
assert.equal(rightInput(0,true).click,null);
rightInput(0,false);
assert.deepEqual(events,["stop","stop"]);
assert.equal(hud.actionMessage("calibration_begin","test").side,"right");
assert.equal(hud.triggerOwnership.sample("right",.5,null).blocked,false);
hud.triggerOwnership.sample("right",undefined,null);
assert.equal(hud.triggerOwnership.sample("right",1,null).blocked,true);
// Left trigger can click a right-arm menu item, including when the right hand is absent.
const leftButtons=Array.from({length:6},()=>({value:0,pressed:false}));
frame.session.inputSources=[{handedness:"left",targetRaySpace:{},gamepad:{buttons:leftButtons}}];
transform.position.x=.30; transform.position.y=-.20; // right-arm home button
const leftHands={left:{is_controller:true,tracking_valid:true,pose:{emulated_position:false}}};
hud.update(frame,{}, {transform},leftHands,100);
leftButtons[0]={value:1,pressed:true};
const leftRay=hud.update(frame,{}, {transform},leftHands,100)[0];
assert.equal(leftRay.claim.click,"return_right_start");
assert.deepEqual(events,["stop","stop","return_right_start"]);
console.log("PASS: single-right input, independent jaw over buttons, deliberate trigger/A clicks and left cursor on right-arm home.");

