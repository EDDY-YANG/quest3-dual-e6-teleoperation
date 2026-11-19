# Changelog

## v0.2.0 · 2026-11-19

- Added one versioned asset catalog for the E6, Quest 3 Touch Plus controllers and the prototype gripper. The PC monitor, in-headset side panels and the HUD mirror share the Link6 attachment and independent left/right instances.
- Added a 28-occurrence STEP-derived gripper with a crank, two connecting rods and two sliding fingers. Analog Trigger sets each model's opening. The original Grip function is preserved, and HUD trigger use takes priority over jaw input.
- Added an isolated `/asset-preview` page, per-side status/recording fields and an optional USB serial driver for one Arduino Uno per gripper. The firmware provides side identification, range checks, a slew limiter and a watchdog. A future PCB can implement the same driver boundary.
- Default output is virtual. Calibrated servo endpoints, actual installation clocking and physical jaw travel require bench commissioning. PWM acknowledgements are command acceptance, and measured jaw opening remains unavailable.
- For setup, see [GRIPPER.md](GRIPPER.md). The public package contains the runtime mesh and its provenance. Local CAD, video and device settings are not distributed.

## v0.1.0 · 2026-11-18

Initial public Windows x64 SDK with VS Code setup, 11 editable chain packages, RangedIK and DLS software checks and hardware-neutral defaults.
