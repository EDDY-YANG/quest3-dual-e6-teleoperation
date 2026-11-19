# Changelog

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
