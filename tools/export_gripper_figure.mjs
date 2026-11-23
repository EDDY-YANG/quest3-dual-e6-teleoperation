// Export the SDK's real GLB assembly in its flange frame for the system figure.
import {readFile,writeFile} from "node:fs/promises";
import {pathToFileURL} from "node:url";
import path from "node:path";
const root=path.resolve(import.meta.dirname,"..");
const web=path.join(root,"sdk/app/web");
const {GLTFLoader}=await import(pathToFileURL(path.join(web,"robot-assets/three/GLTFLoader.js")));
const THREE=await import(pathToFileURL(path.join(web,"robot-assets/three/three.module.js")));
const {mountGripper}=await import(pathToFileURL(path.join(web,"gripper_model.mjs")));
const catalog=JSON.parse(await readFile(path.join(web,"asset_catalog.json"),"utf8"));
const id=catalog.instances.right.tool,def=catalog.assets[id],dir=path.join(web,"robot-assets",id);
const bytes=await readFile(path.join(dir,def.model));
const model=await new GLTFLoader().parseAsync(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength),"");
const mechanism=JSON.parse(await readFile(path.join(dir,def.mechanism),"utf8"));
const flange=new THREE.Group();
const grip=mountGripper({links:new Map([["Link6",flange]])},
  {side:"right",mount:catalog.instances.right.mount,tool:{scene:model.scene,mechanism}});
flange.updateMatrixWorld(true);
const floats=[],point=new THREE.Vector3();
grip.root.traverse(n=>{
  if(!n.isMesh)return;
  const g=n.geometry, pos=g.attributes.position, index=g.index;
  for(let i=0;i<(index?index.count:pos.count);i++){
    point.fromBufferAttribute(pos,index?index.getX(i):i).applyMatrix4(n.matrixWorld);
    floats.push(point.x,point.y,point.z);
  }
});
await writeFile(path.join(root,"runtime/gripper-figure-triangles.bin"),Buffer.from(new Float32Array(floats).buffer));
console.log("SDK flange-mounted gripper triangles",floats.length/9);
