// One typed resource loader for the arm, both controller models and end effectors.
import {STLLoader} from "./robot-assets/three/STLLoader.js";
import {GLTFLoader} from "./robot-assets/three/GLTFLoader.js";
let catalogPromise;
const cache = new Map();

async function jsonAt(url) {
  const response = await fetch(url);
  if (!response.ok) throw new Error(`Asset request ${response.status}: ${url}`);
  return response.json();
}
export function assetCatalog() {
  if (!catalogPromise) catalogPromise = jsonAt("/api/assets").then(c => {
    if (c.schema !== 1) throw new Error("Unsupported asset catalog");
    return c;
  }).catch(e => { catalogPromise = undefined; throw e; });
  return catalogPromise;
}
function urlFor(id, descriptor, name) {
  if (!Object.hasOwn(descriptor.files, name)) throw new Error(`Unknown asset file ${id}/${name}`);
  return `/api/assets/${encodeURIComponent(id)}/${encodeURIComponent(name)}`;
}
export async function loadAsset(id) {
  const catalog = await assetCatalog(), descriptor = catalog.assets[id];
  if (!descriptor) throw new Error(`Unknown asset ${id}`);
  if (!cache.has(id)) cache.set(id, (async () => {
    const url = n => urlFor(id, descriptor, n);
    if (descriptor.kind === "urdf-stl") {
      const definition = await jsonAt(url(descriptor.model));
      const meshes = [...new Set(definition.links.flatMap(l => l.visuals.map(v => v.mesh)))];
      const geometry = new Map(), loader = new STLLoader();
      await Promise.all(meshes.map(async name => {
        const response = await fetch(url(name));
        if (!response.ok) throw new Error(`Mesh ${name}: HTTP ${response.status}`);
        const g = loader.parse(await response.arrayBuffer()); g.computeBoundingBox(); geometry.set(name,g);
      }));
      return {id, descriptor, definition, geometry};
    }
    const loader = new GLTFLoader();
    if (descriptor.kind === "controller-pair") {
      const sides = await Promise.all(["left","right"].map(s => loader.loadAsync(url(descriptor.models[s]))));
      return {id, descriptor, models: {left:sides[0].scene, right:sides[1].scene}};
    }
    if (descriptor.kind === "parallel-gripper") {
      const [g, mechanism] = await Promise.all([loader.loadAsync(url(descriptor.model)), jsonAt(url(descriptor.mechanism))]);
      return {id, descriptor, scene:g.scene, mechanism};
    }
    throw new Error(`No model loader for ${descriptor.kind}`);
  })().catch(e => { cache.delete(id); throw e; }));
  return cache.get(id);
}
export async function loadArmAssembly(side) {
  const c = await assetCatalog(), instance = c.instances[side];
  if (!instance) throw new Error("Unknown arm side");
  const [arm, tool] = await Promise.all([loadAsset(instance.arm),loadAsset(instance.tool)]);
  return {arm,tool,mount:instance.mount,side};
}
