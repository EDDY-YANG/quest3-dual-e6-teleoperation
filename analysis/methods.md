# Real-log comparison methods

This is an **observational analysis of existing hardware recordings**, not a matched-trajectory benchmark, a new robot experiment, or evidence of universal superiority. The main comparison uses **recordings labelled left arm only**. The supplemental summaries preserve both-arm and right-arm results separately. No improvement percentages or statistical-significance claims are made.

## Cohorts and provenance

All 191 recording directories containing an `events.jsonl` file were inventoried. Eligibility is based on metadata and explicit runtime fields, never on whether an outcome looks favorable.

| Cohort | Runtime identification | Available recording dates |
|---|---|---|
| Local DLS IK (no QP) | `LOCAL_BOUNDED_DLS_CONTROLLER_FK_VERIFIED`, no QP motion-reference field and no RangedIK object | 2026-11-04 |
| Local DLS IK + QP | The same local IK solver and `FILTERED_VELOCITY_ACCELERATION_BOX_QP` | 2026-11-04 |
| RangedIK v7 direct | `RANGEDIK_DIRECT` and `e6_world_pose_geodesic_warm_v7` | 2026-11-17 |

Records must originate from an explicit headset Arm attempt (`metadata.source = paired_headset`). Read-only diagnostics, explicit-Stop-only records, missing/ambiguous algorithm tags, and other RangedIK versions are excluded from these three cohorts and listed in `excluded_records.json`. No-outcome Arm attempts remain in the audit; they are not silently removed or automatically labelled failures. Seven IK records, twelve QP records, and twenty-six v7 arm records meet the all-arm cohort definitions. The primary left-only comparison uses seven, twelve, and seventeen records respectively.

The video dates supplied by the operator are 11/04, 11/11, and 11/17. **The available QP cohort is dated 11/04; the inspected 11/11 logs already contain RangedIK direct variants.** Results must therefore be labelled by the verified recorded algorithm, not presented as exact measurements of an unaligned video. No source-video/time synchronization has been established. No real-hardware execution of a RangedIK-plus-DLS cascade was identified; that offline prototype is not a column in this hardware comparison.

`per_record_metrics.csv` and `sessions.json` list recording identifiers, event-file SHA-256, byte counts, recorded controller-source hashes, profile information, selection counts and segment details. They do not publish IP addresses, MAC addresses, absolute workstation paths or raw trajectory coordinates. Original logs remain untouched and local. Public summaries are auditable, but reproducing the full statistics requires access to those original logs.

The cohorts are algorithm families, not frozen parameter-controlled trials. For example, the QP cohort contains 10-degree and 30-degree IK/tracking-window settings, while its reference velocity/acceleration settings are recorded separately. Per-record parameter snapshots and each group's parameter-variant counts are included in the JSON. They must not be described as a single-variable ablation.

## Time and segment handling

- Discard `pre_roll` rows from all quantitative measures. They are context from earlier activity, not new observations of the current attempt.
- Use the embedded **`host_monotonic_s`** for feedback and controller events. Do not subtract `pc_perf_ns` from those timestamps: their origins differ by about 22 seconds in some early Windows recordings. The observed offset is preserved in the audit to make the issue explicit. Quest display timestamps and robot wall-clock fields are not mixed into this timebase.
- A motion segment begins at `GRIP_ENGAGED` and ends at the associated `STOP_RESULT.evidence.requested_at`. Stop-settling tails and idle intervals between grips are excluded from motion metrics. Segments without a verifiable closing event or fresh anchor feedback are explicitly excluded and counted.
- Require valid, connected, finite feedback, with the same connection generation and coordinate-frame identity as the segment anchor. Duplicate generation/timestamp samples retain the last captured value and are counted. Missing feedback is not interpolated or invented.
- A reference/goal must have a timestamp **strictly earlier** than the feedback sample. Equal Windows monotonic ticks cannot establish within-tick ordering and do not qualify as a new causal reference.

## Tracking gaps: two distinct references

1. **Accepted-reference gap** compares measured TCP feedback with the latest causally preceding `TARGET_ACK.pose_mm_deg` in the same Grip segment. It is the difference from the accepted command reference. It includes command queue, execution and sampling effects; it is neither inverse-kinematics numerical residual nor proof that the controller had already completed that reference.
2. **Processed-requested-goal gap** compares feedback with the latest causally preceding `INCREMENT_WINDOW` whose decision is `PLANNED`, using `target_pose_mm_deg`. Where that event is absent, an explicitly recorded `IK_SOLUTION.requested_pose_mm_deg` can be used and the source is reported. No accepted-reference value is substituted for a missing requested goal. The v7 field may already include the documented orientation-cone projection.

Neither measure is labelled **raw hand-to-robot tracking error**. Early recorded goals use measured-base incremental mapping; v7 retains Grip-anchored world goals. That difference, together with the different freehand motions and start configurations, limits cross-cohort causal interpretation.

For position, each observation is the Euclidean difference in millimetres. RMSE is `sqrt(mean(error_mm**2))`; P50 and P95 are empirical percentiles (NumPy's default linear percentile convention). Pose orientation uses the shortest SO(3) rotation-angle difference, with fixed-axis XYZ Euler angles interpreted as `Rz @ Ry @ Rx`, rather than subtracting Euler components. Orientation results are in degrees.

Statistics are pooled over eligible measured samples at the native recording cadence (approximately 20 Hz); they are not averages of per-record medians. Per-record metrics are supplied so another aggregation can be evaluated. Longer records contribute more samples. Temporal samples are correlated, so no independence-based significance test is applied.

## Speed and command continuity

**Measured TCP speed** is estimated as `norm(p[i]-p[i-1]) / (t[i]-t[i-1])`, in mm/s, using consecutive valid feedback samples within the same Grip segment, generation and frame. Intervals must satisfy `0 < dt <= 0.2 s`; larger gaps, duplicate times and connection changes are rejected. Speed is a finite-difference estimate at the recorded cadence, not a native high-rate velocity measurement. No smoothing, speed cap or outlier clipping is applied. Stationary periods within a held Grip remain included. Faster is not automatically better.

**Joint-command step** is `max_j(abs(q_cmd[i,j]-q_cmd[i-1,j]))`, in degrees, between successive accepted targets within a Grip segment. It is a reproducible command-continuity proxy, **not a measure of physical vibration, acceleration or jerk**. The first command of a new Grip is not compared with an old command from a previous Grip. Large increments are retained, including the observed v7 maximum of about 37.36 degrees.

## Stops and data integrity

Normal labelled Stop reasons are `GRIP_RELEASED`, `OPERATOR_DISARMED`, and `SERVICE_CLEANUP`; other reasons are listed as **non-normal Stop events**. Such a reason is not automatically proof of an unintended physical failure: for example, stale-input events can be caused by leaving the input session. Counts are per recorded arm, not independent dual-arm failures.

Stop counts cover all eligible Arm-attempt records, including attempts without valid Grip tracking. Consequently **no stop-per-Grip-minute rate is published**, because that would mix a broader numerator with a narrower denominator. The Arm-record count and tracked Grip duration are reported separately. Explicit action rejections and writer errors are also retained. A record with zero accepted targets is not automatically counted as a failed experiment.

Queue-drop counts come from actual saved `session_end` fields; missing counts are reported as unknown, not silently converted to complete data. Zero queue drops does not prove perfect network tracking or complete physical-state coverage.

## Reproduction and checks

Use Python 3.10+ and NumPy with a new output directory:

```powershell
python analyze_real_logs.py --recordings "PATH_TO_E6_TELEOP_RECORDINGS" --output results_new
```

The program refuses to overwrite an existing results directory, checks source-file size/mtime for changes during reading, hashes the exact read bytes, and blocks network connections with a Python audit hook. It imports no robot driver, SDK control class or native solver.

Eleven isolated mathematical checks cover position norm, Euler-wrap orientation, constant-speed finite differences, generation/gap/zero-time rejection, causal and equal-tick association, empty values and RMSE. These mathematical checks are never included in the real-record distributions.

The retained raw-metric distributions and per-record summaries permit independent recomputation. The observed values describe tradeoffs in this historical dataset; they cannot establish that one controller is globally more accurate, safer or smoother under identical commands.
