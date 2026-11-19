import * as THREE from "./robot-assets/three/three.module.js";

// Rest mesh vertices are expressed in the CAD mounting frame, metres.
export function mountGripper(arm, assembly) {
  const {tool, mount} = assembly, parent = arm.links.get(mount.parent);
  if (!parent) throw new Error(`Missing flange link: ${mount.parent}`);
  const root = tool.scene.clone(true);
  root.name = `${assembly.side}_gripper`;
  root.position.fromArray(mount.xyz_m);
  root.quaternion.setFromEuler(new THREE.Euler(...mount.rpy_rad,"ZYX"));
  parent.add(root);
  const groups = Object.fromEntries(tool.mechanism.groups.map(k => {
    const node = root.getObjectByName(k); if (!node) throw new Error("Missing gripper group "+k);
    return [k,node];
  }));
  const setPose = pose => {
    if (!pose || ![pose.crank_rad,pose.slide_mm,pose.rod_rad,...(pose.rod_offset_mm || [])].every(Number.isFinite)
        || pose.rod_offset_mm?.length !== 2) return false;
    groups.crank.rotation.z = pose.crank_rad;
    groups.slide_negative.position.x = -pose.slide_mm / 1000;
    groups.slide_positive.position.x = pose.slide_mm / 1000;
    for (const [name,sign] of [["rod_negative",1],["rod_positive",-1]]) {
      groups[name].rotation.z=pose.rod_rad;
      groups[name].position.set(sign*pose.rod_offset_mm[0]/1000,sign*pose.rod_offset_mm[1]/1000,0);
    }
    root.updateWorldMatrix(true,true);
    return true;
  };
  return {root,groups,setPose};
}
