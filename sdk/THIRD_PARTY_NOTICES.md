# Third-party components and references

## Quest 3 Touch Plus visualization assets (2026-11-12)

`web/robot-assets/quest-touch-plus/` contains the matching left/right GLB
device models and profile/registry metadata from
`immersive-web/webxr-input-profiles`, commit
`f4992299601614adbfefd398dc8e281556bb7444`.
The assets package MIT license and repository license notice are preserved as
`ASSETS-LICENSE.md` and `LICENSE.md`. The per-file hashes/source paths are in
`manifest.json`. These are maintained device visualization meshes, not
manufacturing CAD or image-generated approximations. The view is centered;
position is separately shown numerically from actual tracking data.

The matching Three.js 0.187.0 GLTFLoader, BufferGeometryUtils and SkeletonUtils are bundled
under `web/robot-assets/three/`, with local import-path changes only, alongside
the existing library license/integrity manifest. No external CDN is required
when viewing the monitor page.

The MIT license in this repository applies to this project's original source. It does not relicense third-party software, device firmware, specifications or vendor trademarks. This is an independent project, not an official Meta or DOBOT product.

| Component | Use | Distribution / license |
|---|---|---|
| aiohttp | Local HTTP and WebSocket server | Installed from PyPI; Apache-2.0. See https://github.com/aio-libs/aiohttp |
| Python dependencies | HTTP server plus NumPy/Requests required by the pinned DOBOT formatter | Exact versions in `requirements.lock.txt`; each retains its own license. Wheels are not included in the source release. |
| Android Platform-Tools / ADB | USB device discovery and reverse port forwarding | Downloaded separately from Google's official host. Google's SDK terms apply. Binaries and ZIP archives are excluded from the source release. See https://developer.android.com/tools/releases/platform-tools |
| Ruff | Optional Python formatting/linting | Installed as a development dependency; MIT. See https://github.com/astral-sh/ruff |
| WebXR specifications | Pose, reference-space and gamepad API contract | Standards references, not robot telemetry or bundled software. https://www.w3.org/TR/webxr/ and https://www.w3.org/TR/webxr-gamepads-module-1/ |
| Immersive Web samples | Reference for the browser session lifecycle | https://github.com/immersive-web/webxr-samples/blob/main/input-tracking.html (MIT). The scene engine, assets and sample code are not vendored. |
| DOBOT TCP/IP Python V4 | Pinned command formatter used by `dobot_safe_sdk.py`; separate four-command diagnostics remain read-only | https://github.com/Dobot-Arm/TCP-IP-Python-V4 at commit `82bce656c6e1a1c71599901baa93ed45bb50963d` (MIT). The minimal source bundle includes the unmodified `dobot_api.py` and original `LICENSE`; exact hashes are recorded in `vendor_manifest.json`. The adapter blocks other upstream public methods and does not replay commands. |

Vendor APKs, camera images, model weights, participant recordings and private account credentials are not part of the source package. The optional official E6 visual assets added below retain their upstream attribution.

## E6 visual model and local renderer (2026-11-11)

`web/robot-assets/e6/` contains the official `me6_robot.urdf` and visual STL
files from `Dobot-Arm/DOBOT_6Axis_ROS2_V4`, commit
`7807ef8fb31e8f86a76cd474a4a9cddffef6eca9`. The upstream root MIT license is
preserved as `UPSTREAM-LICENSE`; the package XML's placeholder license field
is preserved in `upstream-package.xml`. Exact source paths and hashes are in
that directory's `manifest.json`. These assets describe appearance; they do
not establish collision validation, physical calibration or motion acceptance.

`web/robot-assets/three/` contains Three.js 0.187.0 and STLLoader from the
official npm package, with its MIT license and an integrity/hash manifest.
STLLoader's bare `three` import is changed to the local `./three.module.js`.
Rendering assets are served locally through an explicit filename allowlist;
no CDN or public upload is needed during headset use.

## RangedIK Windows native library

The included patched source under `vendor/rangedik-source-snapshot/` comes from the MIT-licensed
`uwgraphics/relaxed_ik_core` ranged-ik branch at commit `1c48d2ae408b4e024ee037641aac1e728267984e`.
The public DLL is compiled for x86_64-pc-windows-msvc with Rust 1.90.0 from that snapshot.
The build manifest records the source, lockfile and binary hashes. Cargo dependency notices and
license texts are in the distribution root under `licenses/`. The Windows SDK, MSVC, Cargo caches,
Python distribution and Android Platform Tools are separate developer tools.


Runtime paths above are relative to the assembled app. Licenses and inputs are also in their corresponding packages.

## Gripper and Arduino integration

The gripper GLB is a runtime mesh derived from the project-supplied Prototype V1 A STEP assembly. The STEP servo body is labelled MG996R; XINHUI 20KG identifies the actuator in the supplied hardware reference. Third-party marks and supplier CAD rights remain with their respective owners. The original assembly, video and shopping image are omitted. Arduino Servo and pySerial are separately installed dependencies. The catalog preserves the existing E6, Quest Touch Plus and Three.js notices.
