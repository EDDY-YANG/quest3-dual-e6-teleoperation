# Changelog

## Website update · 2026-11-24

- Added two full-length virtual-environment recordings dated 24 November: dual-arm spoon-and-bowl dexterity and an unsuccessful single-arm powder-topping attempt onto a cake.
- Explicitly identified the topping attempt as unsuccessful and kept simulation footage separate from physical-robot evidence and the existing hardware statistics.
- Preserved all seven earlier videos and their media files. The two new H.264/AAC MP4 files retain the source bytes and audio, with VP9/Opus alternatives and source-frame posters for browser playback.
- Updated the website, README, video provenance and rights notice to cover all nine recordings. The virtual-lab implementation and the control SDK are not changed by this media update.

## v0.3.0 · 2026-11-23

- Published two single-arm manipulation clips, including an audio-free everyday-object sequence and an M6 nut pickup.
- Added independent analog-trigger gripper control, full-open acknowledgement before following, latest-target serial updates and release-to-open handling. Both controller pointers can operate the HUD; one right hand can control a single right arm.
- Added configurable Uno signal pin and calibrated pulse range, plus opt-in position PI requiring measured external feedback. Default hardware outputs remain unconfigured.
- Kept return controls visible and added explicit clear-only alarm recovery with controller-state verification.
- Updated the system model using the existing articulated gripper and the original illustration pose. Added the USB, PWM and external power path to the figures.
- Physical evidence consists of the supplied single-arm demonstrations. Public defaults require local device and servo calibration.


## [v0.2.0](https://github.com/EDDY-YANG/quest3-dual-e6-teleoperation/releases/tag/v0.2.0) · 2026-11-19

### Added

- CAD-derived articulated grippers on the left and right E6 flange. The PC monitor and both headset views use the same assembly. Analog index Trigger sets finger separation, and the middle-finger Grip controls arm following.
- A single versioned asset catalog and resource API for E6, Meta Quest 3 Touch Plus and the gripper. Recordings include the asset revision, side, input sequence and target opening.
- Optional USB serial drive for **one Arduino Uno per gripper**. Each board verifies its side, applies calibrated pulse limits and slew limiting, and times out stale commands. A PWM ACK acknowledges a target. Physical jaw opening requires independent measurement.
- An isolated asset preview, a single-board bench tool, and an [unloaded commissioning guide](docs/BENCH_TEST.md). The runtime source is now browsable in [sdk/](https://github.com/EDDY-YANG/quest3-dual-e6-teleoperation/tree/main/sdk).

### Changed

- Virtual assets moved from the frontend source package to `12_virtual_assets`. Existing assembled application paths and asset URLs continue to work. Gripper control and firmware live in `13_gripper`. Setup also installs pySerial.
- The project page shows an actual SDK-rendered dual-gripper workspace. Historical hardware footage and error comparisons retain their original dates and cohorts.

### Verification

The real WebSocket/backend/monitor path and independent left/right render instances were tested offline. Both AVR Uno firmware builds were exercised in an ATmega328P emulator, including UART commands, D9 pulses, wrong-side rejection and watchdog expiry. The grippers default to virtual output. Servo calibration, physical mounting angle, loaded behavior and real device acceptance remain on the bench checklist.

[Compare v0.1.0…v0.2.0](https://github.com/EDDY-YANG/quest3-dual-e6-teleoperation/compare/v0.1.0...v0.2.0)

## [v0.1.0](https://github.com/EDDY-YANG/quest3-dual-e6-teleoperation/releases/tag/v0.1.0) · 2026-11-18

Initial public Windows x64 SDK, with 11 editable source packages, VS Code setup, paired Quest input and the dual-E6 teleoperation workflow.
