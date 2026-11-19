# Headset-first operation and recovery

The interface is designed so that one operator can carry out routine preparation, direction calibration, Grip-controlled teleoperation and guarded return-to-start from inside the Quest headset. Two E6 model views flank the central controls, keeping each arm's feedback and the shared operating state visible together.

The PC still hosts the backend. Initial installation, network/certificate setup and launching the service remain host tasks. Once the session is connected and the work area is prepared, the headset provides the subsequent workflow; the website itself has no robot connection.

## Operating sequence

1. **Connect / recheck.** Confirm the intended arm(s), fresh robot feedback and valid controller tracking. The side models follow measured joint state; their presence alone is not proof of fresh feedback.
2. **Return to the intended start pose when needed.** Request the relevant arm's **回到开始位** action from the headset. The selected target may be a previously measured start or an explicitly configured joint target. Do not treat a saved target as proof that its path is unobstructed.
3. **Calibrate the frames.** Record each participating arm's base +X direction and the shared operator-forward direction. Follow the [calibration guide](CALIBRATION.md).
4. **Start and engage Grip.** Release the participating Grips, start teleoperation, then re-engage Grip as instructed. The backend takes fresh hand/TCP anchors. Release Grip to pause following.
5. **Stop or recover in the headset.** Use the shared Stop, recheck, or a permitted per-arm return request as appropriate. A completed return requires a recheck before teleoperation is started again.

## What the return button actually does

The return workflow first stops the teleoperation group and requires feedback-confirmed stationary state. It checks robot identity, prevents assigning the same robot to both sides, takes command ownership and stops the selected arm again before planning the return.

For the explicitly identified collision-protection state, the backend permits **one recovery-clear attempt** only when TCP control is active, no emergency stop is asserted, the arm is stationary, collision protection is present and no other controller alarm is reported. After clearing, it sends another Stop and requires fresh feedback from the same robot and frame, stationary state and a cleared collision flag.

It then cross-checks User/Tool settings, joint limits and controller forward kinematics for the saved target. The selected arm is enabled if necessary. The joint-space return is segmented and checked against real command/feedback progress; simultaneous return jobs are excluded.

Final arrival requires stable measured feedback: joint error at most 0.5°, final TCP position error at most 1 mm and orientation error at most 1°, with at least three stable frames spanning 0.15 s or more. The final tool-down check is also applied. These are implementation acceptance thresholds, not benchmark results or a guarantee that every return succeeds.

If an intermediate check fails, the workflow requests Stop. If stopping cannot be confirmed, a fault is latched. `AT_START` is reported only after the measured arrival checks; the UI then asks for a recheck before restarting teleoperation.

## Boundaries

- The button does not clear arbitrary controller alarms, release a physical emergency stop, remove an obstruction or eliminate the need for inspection after a physical incident.
- Return-to-start follows a joint-space path. It provides no general collision avoidance and does not guarantee a downward-facing TCP throughout the path.
- During a headset-owned return, the Quest session/input must remain current and the AR session visible, with at least one valid real controller and the selected Grip released. Sustained session/input loss aborts the task.
- Direction calibration is not required merely to return to a verified start target; it is required for subsequent mapped teleoperation.

Source trace in the Windows SDK under `sdk/` includes `server.py` (`begin_home_return`, `quiesce_for_home`), `start_positions.py` (`_run`, `_clear_collision`), `headset_bridge.py`, `start_positions.mjs`, `quest_hud.mjs`, and `e6_urdf_view.mjs`.
