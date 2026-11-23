# Single-arm gripper control

The two new demonstrations show one E6 handling a screwdriver, a PLA-printed part, a metal spoon and an M6 nut. The first video's audio stream was removed. The second keeps its source audio. Object identities are supplied by the author; these clips are qualitative demonstrations.

## Inputs and execution

The right index Trigger continuously controls gripper position. Middle-finger Grip gates right-arm following. Both controller rays can click the interface, and the right controller alone can operate the system. A button press that starts over an interface control also requests that action. The optional A-button click can operate the panel while preserving the current index-trigger value.

The PC sends the latest calibrated pulse target to the gripper's own Uno over 115200-baud USB serial. In the demonstrated hardware, D2 drives the servo, with 1300 µs open and 400 µs closed. Those endpoints describe the author's calibration; each installation supplies its own verified endpoints and port.

Starting teleoperation first sends the full-open target and waits for the corresponding Uno acknowledgement. Releasing the index trigger replaces an unfinished closing target. The direct mode adds no software slew ramp. STOP, stale tracking, homing and robot faults revoke output according to the existing supervisory logic.

The three-wire servo does not report measured jaw position. ACK confirms the accepted pulse target. The optional PI implementation requires an external position sensor and measured calibration; PI was disabled for these demonstrations.

## Editable source locations

| Component | Files | Change |
|---|---|---|
| Input and headset UI | `01_frontend/files/web/gripper_ui_input.mjs`, `quest.js`, `quest_hud.mjs` | Continuous index input, both cursors, and persistent home controls |
| Backend and start sequence | `02_backend/files/server.py`, `07_motion_control/files/teleop_group.py` | Independent gripper permission and full-open acknowledgement before following |
| Gripper target and serial drive | `13_gripper/files/gripper_control.py`, `gripper_uno.py` | Calibrated target, release handling, side/session checks and feedback fields |
| Uno firmware | `13_gripper/files/firmware/quest_gripper/` | Configurable pin and pulse endpoints, direct updates, watchdog and optional measured-position PI |
| Virtual assets | `12_virtual_assets/files/web/asset_catalog.json` and the SDK assembly modules | Shared E6 flange mount and articulated gripper across views |

The system illustration preserves its previous six-joint pose and camera direction while adding the existing flange-mounted gripper. Its wiring adds the USB/Uno/PWM channel and external servo supply. The data-flow figure now includes the independent index-trigger branch.

The arm solver in this deployment is RangedIK v7 direct. The DLS hybrid remains an offline research implementation. The existing historical comparison table covers its original arm sessions.

## Download and configure

The v0.3.0 Windows package includes these source changes. Extract it and run `OPEN_IN_VSCODE.cmd`. Device addresses, serial ports, home positions and raw experiment logs are configured locally. See [gripper commissioning](GRIPPER.md).
