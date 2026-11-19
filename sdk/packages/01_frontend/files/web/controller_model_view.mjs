// Exact meta-quest-touch-plus device meshes from the pinned WebXR profile assets.
import * as THREE from "./robot-assets/three/three.module.js";
import {assetCatalog,loadAsset} from "./asset_store.mjs";
export class ControllerModels {
  constructor(canvases) { this.canvases=canvases;this.ready=false;this.error=null;this.lastPaint=-Infinity;this.loading=this.initialize(); }
  async initialize() {
    try {
      const catalog = await assetCatalog();
      const loaded = await Promise.all(["left","right"].map(async side => {
        const pair = await loadAsset(catalog.instances[side].controller);
        return {scene:pair.models[side].clone(true)};
      }));
      this.renderer=new THREE.WebGLRenderer({alpha:true,antialias:true,preserveDrawingBuffer:true});
      this.renderer.setSize(640,300,false);this.renderer.setClearColor(0,0);
      this.scenes=loaded.map(gltf=>{
        const scene=new THREE.Scene(), pivot=new THREE.Group(), mesh=gltf.scene;
        mesh.updateMatrixWorld(true);
        const bounds=new THREE.Box3().setFromObject(mesh),center=bounds.getCenter(new THREE.Vector3());
        mesh.position.sub(center);pivot.add(mesh);scene.add(pivot);
        scene.add(new THREE.HemisphereLight(0xffffff,0x566579,3));
        const light=new THREE.DirectionalLight(0xffffff,3);light.position.set(.4,1,.6);scene.add(light);
        const camera=new THREE.PerspectiveCamera(36,320/300,.005,5);
        const radius=bounds.getBoundingSphere(new THREE.Sphere()).radius;
        camera.position.set(.16,.22,.3).normalize().multiplyScalar(radius/Math.sin(Math.PI/10)*1.12);camera.lookAt(0,0,0);
        const materials=[];mesh.traverse(node=>{if(node.isMesh){node.material=node.material.clone();materials.push(node.material);}});
        const trigger = Object.fromEntries(["min","max","value"].map(key =>
          [key,mesh.getObjectByName("xr_standard_trigger_pressed_"+key)]));
        return {scene,pivot,camera,materials,trigger};
      });
      this.ready=true;
    } catch(e) { this.error=e.message; }
  }
  paint(states, now) {
    if(now-this.lastPaint<100)return;
    this.lastPaint=now;
    if(this.ready){
      this.renderer.setScissorTest(true);
      states.forEach((state,i)=>{
        const s=this.scenes[i];
        const live=state.tracked && state.matchingModel;
        s.pivot.visible=!state.tracked || state.matchingModel;
        s.pivot.quaternion.fromArray(live ? state.quaternion : [0,0,0,1]);
        const value = state.preview ? state.previewTrigger : live && state.buttonsValid ? state.buttons[0]?.value : 0;
        const t = s.trigger;
        if (t.min && t.max && t.value && Number.isFinite(value) && value >= 0 && value <= 1) {
          t.value.position.lerpVectors(t.min.position,t.max.position,value);
          t.value.quaternion.slerpQuaternions(t.min.quaternion,t.max.quaternion,value);
        }
        for(const m of s.materials){m.transparent=false;m.opacity=1;}
        this.renderer.setViewport(i*320,0,320,300);this.renderer.setScissor(i*320,0,320,300);
        this.renderer.clear();this.renderer.render(s.scene,s.camera);
      });
      this.renderer.setScissorTest(false);
    }
    this.canvases.forEach((canvas,i)=>{
      if(canvas.width!==320){canvas.width=320;canvas.height=300;}
      const c=canvas.getContext("2d");c.clearRect(0,0,320,300);
      if(this.ready)c.drawImage(this.renderer.domElement,i*320,0,320,300,0,0,320,300);
      c.textAlign="center";c.font="21px system-ui";c.fillStyle="#546376";
      const label=this.error ? "设备模型加载失败" : !this.ready ? "加载 Touch Plus 模型…" :
        states[i].preview ? "Touch Plus · 离线扳机" :
        states[i].tracked && !states[i].matchingModel ? "型号不匹配，仅显示数据" :
        states[i].tracked ? "实时朝向 · 居中显示" : "型号参考 · 非当前姿态";
      c.fillText(label,160,285);
    });
  }
}
