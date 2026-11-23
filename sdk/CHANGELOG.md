# Changelog

## v0.3.0 · 2026-11-23

- Published two single-arm manipulation clips, including an audio-free everyday-object sequence and an M6 nut pickup.
- Added independent analog-trigger gripper control, full-open acknowledgement before following, latest-target serial updates and release-to-open handling. Both controller pointers can operate the HUD; one right hand can control a single right arm.
- Added configurable Uno signal pin and calibrated pulse range, plus opt-in position PI requiring measured external feedback. Default hardware outputs remain unconfigured.
- Kept return controls visible and added explicit clear-only alarm recovery with controller-state verification.
- Updated the system model using the existing articulated gripper and the original illustration pose. Added the USB, PWM and external power path to the figures.
- Physical evidence consists of the supplied single-arm demonstrations. Public defaults require local device and servo calibration.


## v0.2.0 · 2026-11-19

- Added one versioned asset catalog for the E6, Quest 3 Touch Plus controllers and the prototype gripper. The PC monitor, in-headset side panels and the HUD mirror share the Link6 attachment and independent left/right instances.
- Added a 28-occurrence STEP-derived gripper with a crank, two connecting rods and two sliding fingers. Analog Trigger sets each model's opening. The original Grip function is preserved, and HUD trigger use takes priority over jaw input.
- Added an isolated `/asset-preview` page, per-side status/recording fields and an optional USB serial driver for one Arduino Uno per gripper. The firmware provides side identification, range checks, a slew limiter and a watchdog. A future PCB can implement the same driver boundary.
- Default output is virtual. Calibrated servo endpoints, actual installation clocking and physical jaw travel require bench commissioning. PWM acknowledgements are command acceptance, and measured jaw opening remains unavailable.
- For setup, see [GRIPPER.md](GRIPPER.md). The public package contains the runtime mesh and its provenance. Local CAD, video and device settings are not distributed.

## v0.1.0 · 2026-11-18

Initial public Windows x64 SDK with VS Code setup, 11 editable chain packages, RangedIK and DLS software checks and hardware-neutral defaults.
