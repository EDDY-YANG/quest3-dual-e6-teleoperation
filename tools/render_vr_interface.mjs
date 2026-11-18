/**
 * Render the real SDK's 2D VR panels without a browser, server or robot.
 *
 * node tools/render_vr_interface.mjs --sdk-web <SDK web directory>
 * Optional: --canvas-module <installed @napi-rs/canvas directory> --font <font file>
 *
 * SDK modules are imported unchanged. No telemetry is supplied: the actual
 * constructor's disconnected state is retained, so unavailable values stay —.
 * The second output opens the existing calibration UI using its local flag.
 * This exports only the central 2D canvas, not the separate WebGL robot views.
 */

import {createHash} from "node:crypto";
import {readFile, writeFile, mkdir} from "node:fs/promises";
import {existsSync} from "node:fs";
import {createRequire} from "node:module";
import path from "node:path";
import {fileURLToPath, pathToFileURL} from "node:url";

const root = path.dirname(path.dirname(fileURLToPath(import.meta.url)));
const options = {};
for (let i = 2; i < process.argv.length; i += 2) {
  const key = process.argv[i];
  if (!["--sdk-web", "--canvas-module", "--font", "--output-dir"].includes(key)
      || !process.argv[i + 1]) {
    throw new Error("Usage: node tools/render_vr_interface.mjs --sdk-web <directory> " +
      "[--canvas-module <directory>] [--font <file>] [--output-dir <directory>]");
  }
  options[key.slice(2)] = process.argv[i + 1];
}
if (!options["sdk-web"]) throw new Error("--sdk-web must point to the actual SDK web source.");
const web = path.resolve(options["sdk-web"]);
const output = path.resolve(options["output-dir"] || path.join(root, "assets", "ui"));
const require = createRequire(import.meta.url);
const canvas = require(options["canvas-module"] || "@napi-rs/canvas");
const canvasPackage = require(path.join(
  options["canvas-module"] || "@napi-rs/canvas", "package.json"));
let font = options.font;
if (!font && process.platform === "win32") {
  const systemFont = path.join(process.env.WINDIR || "C:\\Windows", "Fonts", "msyh.ttc");
  if (existsSync(systemFont)) font = systemFont;
}
if (font && !canvas.GlobalFonts.registerFromPath(path.resolve(font), "system-ui")) {
  throw new Error("Could not register the supplied system-ui font.");
}
if (process.platform === "win32") {
  const symbolFont = path.join(process.env.WINDIR || "C:\\Windows", "Fonts", "seguisym.ttf");
  if (existsSync(symbolFont)) canvas.GlobalFonts.registerFromPath(symbolFont, "sans-serif");
}

// Minimal native Canvas adapter. There is no DOM navigation, WebXR or browser.
globalThis.document = {
  createElement(tag) {
    if (tag !== "canvas") throw new Error(`Unsupported offline element: ${tag}`);
    return canvas.createCanvas(1, 1);
  },
};
// The SDK side-model constructor tries to preload meshes. It catches this
// explicit refusal. Those separate canvases are never included in the output.
globalThis.fetch = async () => { throw new Error("Network disabled for offline UI artwork."); };
globalThis.WebSocket = class {
  constructor() { throw new Error("Network disabled for offline UI artwork."); }
};

const sources = {};
async function recordSource(file) {
  const absolute = path.resolve(file);
  const relative = path.relative(web, absolute).split(path.sep).join("/");
  if (relative.startsWith("../") || path.isAbsolute(relative)) {
    throw new Error("Source dependency is outside the supplied SDK web directory.");
  }
  if (sources[relative]) return;
  const bytes = await readFile(absolute);
  sources[relative] = createHash("sha256").update(bytes).digest("hex");
  for (const match of bytes.toString("utf8").matchAll(
    /\b(?:import|export)\s+(?:[\s\S]*?\s+from\s+)?["'](\.\.?\/[^"']+)["']/g)) {
    await recordSource(path.resolve(path.dirname(absolute), match[1]));
  }
}
await recordSource(path.join(web, "quest_hud.mjs"));
const {HeadsetHUD, HUD_WIDTH, HUD_HEIGHT} = await import(
  pathToFileURL(path.join(web, "quest_hud.mjs")).href);
const hud = new HeadsetHUD();
await hud.robotModels.loading;
if (hud.remote !== null || hud.client.linkState !== "DISCONNECTED") {
  throw new Error("SDK no longer starts in the expected disconnected state; inspect before rendering.");
}
await mkdir(output, {recursive: true});
const artifacts = [];
for (const [filename, calibrationOpen] of [
  ["vr_panel.png", false], ["calibration_panel.png", true],
]) {
  hud.calibrationOpen = calibrationOpen;
  hud.paint(0, true);
  const png = hud.canvas.toBuffer("image/png");
  await writeFile(path.join(output, filename), png);
  artifacts.push({
    file: filename, width: HUD_WIDTH, height: HUD_HEIGHT,
    bytes: png.length, sha256: createHash("sha256").update(png).digest("hex"),
    calibration_open: calibrationOpen,
  });
}
console.log(JSON.stringify({
  renderer: `@napi-rs/canvas ${canvasPackage.version}`,
  source: "unchanged SDK HeadsetHUD.paint()",
  state: {remote: null, linkState: hud.client.linkState, visibility: hud.client.visibility},
  font: font ? path.basename(font) : "native system font fallback",
  limitations: "Offline central 2D panel only; native font rasterization can differ from Quest Browser; no WebXR, telemetry or separate WebGL robot panels.",
  sources, artifacts,
}, null, 2));
