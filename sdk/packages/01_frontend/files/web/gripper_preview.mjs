import * as THREE from "./robot-assets/three/three.module.js";
import {E6SideModels} from "./e6_urdf_view.mjs";
import {loadArmAssembly} from "./asset_store.mjs";
import {mountGripper} from "./gripper_model.mjs";

const sides=["left","right"], elements={};
for (const [i,side] of sides.entries()) {
  const card=document.createElement("article");
  card.innerHTML=`<div class="head"><h2>${i?"右":"左"}臂 E6</h2><output class="reading"></output></div><div class="views"><canvas class="arm"></canvas><canvas class="detail"></canvas></div><div class="control"><label>食指 Trigger <output class="value"></output><input aria-label="${i?"右":"左"}手 Trigger" type="range" min="0" max="1" step=".01" value="${i?1:0}"></label><span class="small">松开 = 张开　·　按到底 = 闭合</span></div>`;
  document.querySelector("#arms").append(card);
  elements[side]={input:card.querySelector("input"),value:card.querySelector(".value"),result:card.querySelector(".reading"),arm:card.querySelector(".arm"),detail:card.querySelector(".detail")};
}
const armViews=new E6SideModels({leftCanvas:elements.left.arm,rightCanvas:elements.right.arm});
const status=document.querySelector("#state");
function detailRenderer(assembly,canvas) {
  const scene=new THREE.Scene(); const anchor=new THREE.Group();scene.add(anchor);
  const gripper=mountGripper({links:new Map([["Link6",anchor]])},assembly);
  const renderer=new THREE.WebGLRenderer({canvas,antialias:true,alpha:true});
  renderer.setSize(410,550,false);renderer.setClearColor(0,0);
  scene.add(new THREE.HemisphereLight(0xffffff,0x84909c,2.3));
  const key=new THREE.DirectionalLight(0xffffff,2.4);key.position.set(-.3,-.45,-.18);scene.add(key);
  const camera=new THREE.OrthographicCamera(-.09,.09,.121,-.121,.001,3);
  camera.up.set(0,0,-1);camera.position.set(.13,-.46,.03);camera.lookAt(0,0,.106);
  return state=>{gripper.setPose(state.model_pose);renderer.render(scene,camera);};
}
async function main() {
  const details=await Promise.all(sides.map(async side=>detailRenderer(await loadArmAssembly(side),elements[side].detail)));
  await armViews.loading;
  if (!armViews.ready) throw Error(armViews.error||"E6 model failed");
  let running=false,dirty=false;
  async function update() {
    dirty=true;if(running)return;running=true;
    try {
      do {
        dirty=false;const params=new URLSearchParams(sides.map(s=>[s,elements[s].input.value]));
        const response=await fetch("/api/grippers/preview?"+params,{cache:"no-store"});
        if(!response.ok)throw Error("preview HTTP "+response.status);
        const grippers=await response.json();
        armViews.paint({fresh:true,grippers,robots:{},controls:{}},performance.now(),true);
        sides.forEach((side,i)=>{const state=grippers.arms[side];details[i](state);elements[side].value.textContent=state.trigger.toFixed(2);
          elements[side].result.textContent="模型开度 "+state.model_gap_mm.toFixed(1)+" mm";});
        status.textContent="离线装配 · 每侧一块 Uno · 预览滑块不发送串口或机械臂指令";
      } while(dirty);
    } catch(e){status.textContent="预览失败："+e.message;} finally{running=false;}
  }
  for(const s of sides)elements[s].input.addEventListener("input",update);
  await update();
}
main().catch(e=>{status.textContent="模型未能加载："+e.message;});
