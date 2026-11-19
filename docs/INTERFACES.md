# PC and VR interface design

The host console exposes connection, configuration and diagnostics. The headset workspace places **two E6 side views around one central control panel**. Preparation, calibration, Grip control and guarded recovery remain in the operator's view. The project page shows the complete three-panel workspace, then selected functional regions in expandable tabbed cards rather than a long application screenshot.

## Complete headset workspace

The v0.2.0 main figure is a new export of the SDK HUD mirror, with the 28-occurrence gripper attached to both nominal flange frames. The service was running without physical devices. [Gripper setup and inspection](GRIPPER.md).

The archived November 17 figure comes from the actual SDK's read-only HUD mirror captured on **17 November 2026**. `hud_preview.html` instantiates the same `HeadsetHUD` drawing module used in WebXR, with its left model, central panel and right model canvases. This is an authentic interface capture, not a frame extracted from a headset recording.

Both side models in that capture are labelled measured joint poses and show TCP/tool/base-axis information. The central panel shows waiting Quest input, missing controller tracking and an expired precheck. These state labels are retained: the image documents a historical interface with robot feedback, not successful active teleoperation or a current connection.

The deployed application updates each side's model from that arm's measured joint feedback. The display distinguishes measured state, accepted target and unavailable/offline state. The public website is only a project showcase and receives no robot telemetry. See [headset operation and recovery](HEADSET_WORKFLOW.md) for the intended single-operator workflow and implemented limits.

## Host console

- **Joint and TCP monitoring:** independent per-arm models, joint readouts, accepted commands and measured TCP pose.
- **Connection and supervision:** headset/link state, read-only robot feedback connection and shared software Stop.
- **Start positions:** save a measured per-arm start pose and request a guarded return. The software Stop does not replace the physical emergency stop.

The website uses selected CSS viewports into actual historical PC screenshots captured on 17 November 2026. The source images were checked for visible credentials/network identifiers before publication. Their offline/reference-state labels are retained; these figures are not evidence of a currently connected system. The right arm uses the same independently monitored layout.

## Headset interface

The expandable VR status and calibration detail panels are **new renders from the actual SDK `HeadsetHUD.paint()` drawing code**. They are not photographs, headset-video frames or redesigned mockups. Chinese labels are retained from the implemented interface; the accompanying explanation is in English.

The drawing uses the source constructor's disconnected state: `remote=null`, `linkState="DISCONNECTED"`, no supplied telemetry. The calibration view only opens the existing local panel. Consequently unavailable measurements stay as dashes and unavailable actions remain disabled. The default single-arm title is the source's disconnected fallback, not a claim about the capability of the dual-arm workflow.

The export contains the central 1280 × 800 control canvas. It does not include the separate WebGL robot models or reproduce the surrounding passthrough world. Native font rasterization may differ slightly from Quest Browser.

To reproduce it, provide the assembled `app/web` directory from the Windows SDK and an installed `@napi-rs/canvas` version 0.1.100:

```bash
node tools/render_vr_interface.mjs --sdk-web "PATH_TO_SDK_WEB" --canvas-module "PATH_TO_INSTALLED_CANVAS"
```

The script blocks network access, imports the source modules unchanged and prints their hashes. The published artwork provenance is retained in [ui_manifest.json](ui_manifest.json).

## Calibration

The headset provides two-point +X capture, automatic completion of the orthogonal frame using fixed WebXR up, and a separate operator-forward capture. See the [calibration procedure and equations](CALIBRATION.md). These controls define coordinate mappings; solver accuracy and physical tracking remain separate concerns.
