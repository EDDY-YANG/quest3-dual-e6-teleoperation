# Teleoperation frame calibration

The current implementation uses **X-only yaw calibration with a fixed up direction**. The operator measures one horizontal direction for each arm; WebXR's existing up direction supplies the second constraint. This aligns hand displacement with each E6 base frame without requiring the hand controller to begin with the same orientation as the TCP.

This page describes the separately maintained SDK snapshot reviewed on 17 November 2026. It explains the implemented mapping; publishing this website does not connect to a robot.

## What is being calibrated?

| Quantity | How it is obtained | Purpose |
|---|---|---|
| Per-arm base direction map | Two controller positions along that arm's physical base +X | Convert XR hand displacement into the correct robot-base directions |
| Shared operator-forward frame | One controller's pointing ray, projected onto the horizontal plane | Define and display a fixed operator reference shared by both arms |
| Grip anchors | Fresh hand pose and measured TCP pose at each Grip engagement | Start relative position and orientation control without an initial pose jump |

These are distinct operations. The method does not calibrate a camera, estimate a robot's URDF, identify the physical TCP offset, or align an AR model's position with the physical robot.

## Procedure in the headset

1. **Connect and recheck.** Keep the Quest session and controller tracking active. Release Grip. Use **连接 / 重检**. Obtain valid robot feedback; complete a guarded return to the chosen start pose first if needed. Stop motion before recording calibration samples.
2. **Select an arm.** Open **选择/标定**, then choose **左臂/左手** or **右臂/右手**. Each side has its own map.
3. **Record the first point.** Press **1 开始记录X**. The backend stores the selected controller's current validated position, p₀.
4. **Move along that robot's base +X.** Translate the corresponding controller horizontally in the physical positive-X direction of the selected E6 base. Use at least **80 mm of horizontal displacement**. The robot itself need not move during this sampling step. Do not substitute the current tool-axis direction or your viewing direction for base +X.
5. **Finish and apply.** Press **2 完成X标定 · 自动应用**. The backend captures p₁ and derives the frame. No return to p₀, separate +Y sample or additional Apply press is required. A too-short or nearly vertical motion is rejected; total displacement above 1.5 m is also rejected.
6. **Repeat for the other arm** when operating both arms. Its physical base +X may point in a different room direction; do not copy the first arm's matrix.
7. **Record operator forward.** Point either currently tracked controller toward the operator's intended forward direction and press **一键记录操作者前方**. The current implementation permits clicking with that same controller; the other controller may be offline. This stores the controller's pointing ray, not the headset's gaze.
8. **Arm, then re-engage Grip.** Check that the participating arms' calibration and operator-forward frame are current. Release all participating Grips, press **开始遥操作**, then hold Grip as instructed. Fresh hand/TCP anchors are taken at engagement.
9. **Verify direction with small movements.** Check forward/back, left/right and up/down against the actual robot base directions before larger motion. Releasing Grip pauses following. Re-gripping takes new anchors rather than replaying hand motion accumulated while released.

The direction map depends on an accurate physical identification of each base's +X and on level robot bases. The interface's success status cannot prove that the human sampled the intended real-world direction; the small-motion check remains necessary.

## Why a single measured direction is sufficient here

A single X vector leaves rotation about X undetermined. The code removes this ambiguity by **fixing the vertical direction**: WebXR +Y maps to E6 base +Z. It estimates horizontal heading only, not a general three-dimensional rotation.

Let u = [0, 1, 0]ᵀ be WebXR up, and let p₀ and p₁ be the captured controller positions in metres. The exact construction is:

```text
d   = p₁ − p₀
d_h = d − u (uᵀ d)
x   = d_h / ‖d_h‖
y   = (u × x) / ‖u × x‖
x   = y × u

C = R_base←XR = [ xᵀ ; yᵀ ; uᵀ ]
```

The axes are **rows** of C. Consequently Cx = [1, 0, 0]ᵀ, Cy = [0, 1, 0]ᵀ and Cu = [0, 0, 1]ᵀ. The cross-product order preserves a right-handed orthonormal frame. Removing the vertical component of the sampled motion prevents small hand-height drift from tilting the fitted frame; the drift is still recorded.

The fixed-up assumption is essential. A tilted robot base requires a different calibration procedure that identifies its tilt. This method estimates no XR-to-base translation and is not full six-degree-of-freedom registration.

## Shared operator frame

The separate operator-forward action uses the selected controller's **targetRaySpace −Z** pointing direction. Its horizontal unit projection is f; let l = u × f. The stored operator frame is O = [fᵀ; lᵀ; uᵀ].

For each arm, the base-from-operator map is C Oᵀ. Composing with O gives C Oᵀ O = C. Thus the operator frame provides a common, explicit directional reference while preserving each arm's independently calibrated installation direction. The code does not force both robot bases' +X axes to match the operator's +X. The stored forward direction remains fixed when the wrist later rotates.

## From hand motion to a TCP target

At Grip engagement, the system records hand position p_h,0, hand orientation R_h,0, measured TCP position p_TCP,0 and measured TCP orientation R_TCP,0. The v7 target before downstream orientation projection is:

```text
p*_TCP = p_TCP,0 + 1000 C (p_h − p_h,0)
R*_TCP = C (R_h R_h,0ᵀ) Cᵀ R_TCP,0
```

The factor 1000 converts metres to millimetres; the target position gain is 1:1. Orientation is a relative spatial rotation, not a subtraction of Euler angles. The initial controller and TCP orientations need not coincide. Translation uses the fixed base mapping and does not rotate with the current TCP tool axes.

This constructs the desired TCP target. The actual motion still depends on IK, joint/workspace limits, the configured downward orientation cone, command validation and feedback. Aligned directions do not imply zero tracking error, zero delay or an unrestricted workspace.

## Validity and re-calibration

Calibration requires real controller tracking with non-emulated position, released Grip and input age no greater than 200 ms. The backend checks current valid robot feedback, acceptable stopped-state context and User/Tool = 0. Calibration is rejected while control or a start-return action is active.

Changing the Quest session, reference space, reference epoch, robot identity, frame or User/Tool context invalidates the applicable calibration. `control_ready` requires both a current per-arm base map and a current shared operator-forward frame. Follow the status message and repeat the required capture steps after invalidation.

## Implementation trace

Paths below refer to the separately maintained SDK, not files bundled in this showcase.

| SDK source | Relevant implementation |
|---|---|
| `packages/03_pose_mapping/files/direction_calibration.py` | `begin`, `capture_x`, horizontal projection, cross products, per-arm context and validity |
| `packages/03_pose_mapping/files/operator_frame.py` | `UP_XR`, validated hand sample, horizontal pointing-ray capture |
| `packages/03_pose_mapping/files/relative_pose.py` | `anchored_pose_target`, relative position and orientation mapping |
| `packages/07_motion_control/files/left_z_trial.py` | Fresh Grip anchors, frozen mapping and downstream execution gates |
| `packages/02_backend/files/server.py` | Separate arm calibrators and shared `OperatorFrame` |
| `packages/01_frontend/files/web/quest_hud.mjs` | Actual headset control and calibration panel drawing |

The calibration construction is part of this project's integration. It is separate from the RangedIK and DLS solver foundations acknowledged in the [references](REFERENCES.md).
