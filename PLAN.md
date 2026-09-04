# Feature-completion plan: reBot B601-DM module

Goal: bring `devrel:rebot-b601` to parity with the `viam:ufactory` xArm module for every
feature that software can provide on this hardware. The gap analysis this plan is based on
lives in the README under "Comparison with the uFactory xArm module".

Status legend: `[ ]` not started, `[~]` in progress, `[x]` done.

**Status 2026-09-04:** every software item below is implemented and unit-tested against the
in-memory motorbridge fake (`tests/fake_bus.py`); see `CHANGELOG.md` for the 0.2.0 summary.
Still open: hardware validation of the new motion, safety, reconnect, and manual-mode paths;
measuring the sustainable `move_hz`; the upstream Python SDK issue. Notes on what was built
differently from the plan are inline, marked *Implementation note*.

## Out of scope (hardware or vendor limitations)

These uFactory features have no B601 counterpart and are deliberately excluded:

- **UFactory Studio proxy.** The B601 has no vendor web UI to proxy.
- **Force/torque sensor, vacuum grippers, gripper lite.** No such hardware ships for the B601.
- **Gripper object-detected register (G2).** Damiao motors have no such bit; the existing
  stall-based hold detection stays.
- **Firmware-level collision detection with sensitivity levels.** Damiao motors only have
  over-current protection. A software torque-threshold stop is in scope (Phase 3).
- **Hardware variant auto-detection.** There is one B601-DM variant.
- **Dedicated gripper bus.** Arm and gripper share one USB-CAN bridge; the SharedBus lock stays.
- **Manual mode with exact gravity compensation.** True zero-G mode needs a dynamics model the
  vendor does not publish. An approximation using MIT mode with the URDF inertials is a stretch
  goal in Phase 3.

## Key findings that shape the plan

- **Python SDK 0.80.0 does not serve `MoveThroughJointPositions`, `MoveThroughJointPositionsStreamed`,
  or `Get3DModels`.** `viam/components/arm/service.py` only implements the older RPCs, so viam-server
  gets UNIMPLEMENTED when the motion service executes a plan. The RDK arm client's `GoToInputs`
  calls `MoveThroughJointPositions` with no fallback. Fix: subclass `ArmRPCService`, add the
  missing handlers, and swap the registration (`Registry._APIS[Arm.API]`) before
  `Module.run_from_registry()`. `Registry.register_api` refuses duplicates, so this needs the
  private dict. Pin `viam-sdk` to a tested range and file an upstream issue.
- **URDF meshes can be shipped over the wire.** `get_kinematics` may return a 3-tuple
  `(format, urdf_bytes, {mesh_path: Mesh})`; the servicer puts the dict in
  `meshes_by_urdf_filepath` and the RDK resolves `<mesh filename=...>` against it. Content types
  are `"stl"` or `"ply"`. The RDK also parses `<box>`, `<sphere>`, `<cylinder>`, and detects
  capsules from a sphere+cylinder pattern. Joint `<limit velocity>` is ignored by the RDK.
- **Seeed publishes collision and visual STLs plus inertials** in
  `Seeed-Projects/reBot-DevArm/Rebot_Arm_description/DM` (URDF `ReBot_Arm_DM.urdf`, meshes up to
  5 MB per link; hardware license CERN-OHL-W-2.0). Decimate before bundling.
- **motorbridge `MotorState` exposes `status_code`, `t_mos`, `t_rotor`** in addition to
  pos/vel/torq, and Damiao RW registers include `OC_Value`, `OT_Value`, `TIMEOUT`, `MAX_SPD`,
  `ACC`, `DEC`. Error decoding, temperature diagnostics, and a CAN watchdog are all software.

## Phase 0: Test harness and CI (prerequisite for everything else)

Effort: S-M. No hardware needed.

- [x] **Mock controller.** `tests/fake_bus.py`: a `SharedBus` stand-in with fake `Motor` objects
      that record commands and simulate first-order motion toward the last setpoint. Inject via a
      `SharedBus.acquire` monkeypatch or a `bus_factory` hook on the components.
- [x] **pytest suite.** Move `tests/test_spatial.py` to pytest; add tests for arm config parsing,
      `move_to_joint_positions` settle/timeout paths, `stop`, DoCommand dispatch, gripper
      grab/hold detection. Target: every public method has at least one test against the mock.
- [x] **Lint/format.** `ruff` config in `pyproject.toml`; `make lint test` targets.
- [x] **GitHub Actions.** `.github/workflows/test.yml` (pytest + ruff on PRs and main),
      `.github/workflows/deploy.yml` on tag push using `viamrobotics/upload-module` with
      `platform: any`. Secrets: `VIAM_DEV_API_KEY_ID` / `VIAM_DEV_API_KEY` on the
      `viam-devrel/rebot-b601` repo.
- [x] **Build script.** `make module` produces `module.tar.gz` with
      `--exclude __pycache__ --exclude '*.pyc'`; v0.1.0 shipped stray `.pyc` files.
- [x] **Bootstrap test.** CI job that runs `run.sh` on a clean checkout in a container with only
      `python3` (no `uv`) and one with `uv`, and asserts the module starts and answers
      `--dump-resources` style readiness (or simply exits cleanly on SIGTERM).

Acceptance: green CI on a PR; a tag push publishes to the registry without manual tarring.

## Phase 1: Motion-service compatibility (the blocker)

Effort: L. Hardware needed for final validation.

- [x] **Custom arm RPC servicer.** `src/rebot_b601/arm_service.py`:
      `class B601ArmRPCService(ArmRPCService)` implementing `MoveThroughJointPositions`,
      `MoveThroughJointPositionsStreamed`, and `Get3DModels`; handlers call new methods on
      `B601Arm` (`move_through_joint_positions`, `move_through_joint_positions_streamed`,
      `get_3d_models`). In `src/main.py`, replace the arm registration before running the module.
      Unit-test the swap so an SDK upgrade that changes `Registry` internals fails loudly.
- [x] **Trajectory execution.** Replace "send one target, block until settled" with a paced
      setpoint stream, matching the xArm module's servo-mode approach:
      - Interpolate between current position and each waypoint at a fixed `move_hz`
        (default 50 Hz; measure what the 921600-baud bridge sustains with 6 motors and feedback
        polling, and document the ceiling).
      - Respect per-joint velocity and acceleration limits (trapezoidal or S-curve per segment).
      - `waitAtEnd` semantics: return once the last waypoint is within `tolerance_deg` or on
        timeout; `stop()` cancels the stream immediately (reuse `_stop_requested`).
      - `move_to_joint_positions` becomes a one-waypoint call into the same path.
      - Streamed variant: pace by each `TrajectoryPoint.time` relative to the first point, as the
        xArm module does; ignore `direct`.
- [x] **Move options.** Honor `MoveOptions.max_vel_rads` / `max_acc_rads` from the request, and
      `extra` keys `speed_d`/`speed_r`/`acceleration_d`/`acceleration_r`/`direct`/`waitAtEnd`/
      `interpolate` for parity with the xArm module. Add `acceleration_deg_s2` to config.
- [x] **Joint limit enforcement.** Reject out-of-range targets with an error instead of clipping.
      Planners rely on the error; silent clipping produces wrong poses. Keep a `clip_targets`
      attribute (default `false`) for teleop users who prefer the old behavior. Make the URDF
      limits and the module's soft limits agree, since the motion service plans against the URDF.
- [x] **`move_to_position` via the motion service.** Add an optional `motion` attribute (service
      name); when set, declare it as a dependency in `validate_config`, obtain a `MotionClient`
      from `dependencies`, and call `move` with the destination in frame `<arm-name>_origin`.
      When unset, keep the current explanatory `NotImplementedError`.
- [ ] **Hardware validation.** *(open: no arm was attached while 0.2.0 was built)* Configure the arm in the frame system on the bench machine, run a
      motion-service `move` to a cartesian pose, and record: plan execution succeeds, no
      per-waypoint stalls, `stop()` during a plan halts within one tick.

Acceptance: motion service `Move` completes on hardware; `is_moving` is true during the plan and
false after; a target outside the limits is rejected with a clear error.

## Phase 2: Geometry, kinematics, and 3D models

Effort: M. Mostly offline asset work.

- [x] **Asset pipeline.** `tools/build_assets.py`: pull the Seeed description package at a pinned
      commit, decimate collision STLs with `trimesh` (target under 200 KB per link), convert
      visual STLs to per-link GLB for the app viewer, and write everything under
      `src/rebot_b601/assets/`. Record source commit and license in `assets/ATTRIBUTION.md`.
- [x] **Collision geometry in the URDF.** *Implementation note: mesh decimation happens at asset-build time (`tools/build_assets.py`), not at runtime; STLs are capped at 150 KB per link.* Two tiers, selectable by a `collision_geometry`
      attribute:
      - `primitives` (default): hand-fit boxes/capsules per link derived from the collision mesh
        bounding volumes. Cheapest for the planner and parses from bytes without meshes.
      - `meshes`: `<mesh filename="meshes/linkN.stl">` per link; `get_kinematics` returns the
        3-tuple with the decimated STL bytes keyed by that filename. Optional
        `mesh_decimation_ratio` applied at build time (documented, not runtime).
- [x] **`get_geometries` on the arm.** Return the per-link geometries posed by the current joint
      state using `spatial.py` FK (extend it to return every link transform, not only the end
      effector). Add tests that compare against the RDK-computed geometries for a few poses.
- [x] **`Get3DModels`.** Serve the per-link GLBs (content type `model/gltf-binary`) keyed by link
      name, through the custom servicer from Phase 1.
- [x] **Gripper kinematics and geometry.** Ship a small gripper URDF (static base box plus two
      finger geometries; optionally a prismatic joint driven by the single input so the planner
      sees the jaws move). Implement `get_kinematics` and `get_geometries` on `B601Gripper` and
      document the recommended `frame.parent = <arm>` config with the correct tool offset.

Acceptance: motion planning avoids a configured obstacle box; the app's 3D view shows the arm
meshes moving with the joints; the gripper shows as a link in the frame system.

## Phase 3: Safety, error handling, and diagnostics

Effort: M-L. Hardware needed for thresholds.

- [x] **Damiao status decoding.** Map `MotorState.status_code` to text per the Damiao manual
      (over-voltage, under-voltage, over-current, MOS over-temperature, rotor over-temperature,
      communication loss, overload). Surface it in `raw_state`, in a new `get_status`/`readings`
      style DoCommand, and as the error message when a move is refused.
- [x] **Pre-command readiness check.** Before each move, read feedback; if any motor reports a
      fault, attempt `clear_error` once for transient faults (communication loss), then re-enable
      and retry; for hard faults (over-current, over-temperature) refuse the move with the decoded
      reason. Mirrors the xArm `checkReadyState` behavior.
- [x] **Software collision stop.** *Implementation note: shipped disabled (`torque_limit_nm` unset) until thresholds are tuned on hardware.* Optional `torque_limit_nm` (per joint or scalar): during a
      streamed move, if measured torque exceeds the limit for N consecutive polls, hold position
      and raise a collision error. Document that this is coarse compared with firmware detection.
      Tune defaults on hardware; ship disabled if false positives cannot be avoided.
- [x] **Temperature guard.** Warn at a configurable `t_mos`/`t_rotor` threshold, refuse moves above
      a hard limit, expose temperatures in `raw_state`.
- [x] **CAN watchdog.** Optionally write the Damiao `TIMEOUT` register so a motor disables itself
      if the module dies mid-move (`can_timeout_ms` attribute, default off because it changes
      persistent motor state; document `store_parameters` implications).
- [x] **Serial reconnect.** Detect `CallError`/OS errors from the bridge, close and reopen the
      `SharedBus` with backoff, re-run `_configure_motors`, and log once per outage. Health check
      in `is_moving`/`get_joint_positions` paths so viam-server sees a clear error, not a hang.
- [x] **`bad_joints`.** Attribute listing joint indices to hold at their current position; exclude
      them from targets and from limit checks, as the xArm module does.
- [x] **Manual mode (approximation).** *Implementation note: shipped behind the `manual_mode` DoCommand with `gravity_scale` (default 1.0); the README tells users to start at 0.3. Unvalidated on hardware.* `{"manual_mode": "enter"|"exit"}` DoCommand: switch to MIT
      mode with low Kp/Kd and a gravity feed-forward torque computed from the URDF inertials and
      current FK. Servos stay enabled so the arm does not slump. Stretch goal; validate on
      hardware before shipping enabled, and keep `torque: disable` as the fallback.
- [x] **Operation manager semantics.** A new move cancels the previous one (single in-flight
      operation), and `is_moving` reflects the in-flight operation rather than only velocity.

Acceptance: unplugging the CAN bridge during idle produces a decoded error and recovers on
replug; pushing a joint against a soft obstacle during a move stops the arm with a collision
error; a hot motor refuses moves with a temperature message.

## Phase 4: Gripper parity

Effort: S-M.

- [x] **Declare the arm dependency.** Optional `arm` attribute; when set, `validate_config`
      returns it as a required dependency so viam-server starts the arm first and the gripper can
      inherit `port`/`baud` from it. Keeps the shared-bus singleton but makes ordering explicit.
- [x] **Runtime force and speed.** DoCommands `set_speed`, `get_speed`, `set_force` (torque
      ratio), `get_force`, and `grab_with_force {position, speed, force}` for parity with the xArm
      gripper's `grab_with_torque`.
- [x] **Position DoCommands.** `get` (current motor degrees mapped to 0..1 open fraction) and
      `set` (fraction or degrees), matching the xArm `get`/`set` shape.
- [x] **Hold detection tuning.** Expose the stall parameters (`stall_polls`, `move_timeout_s`) and
      report the stall position in `is_holding_something` extras for debugging.
- [x] **Geometry and kinematics.** Covered in Phase 2.

## Phase 5: Documentation and release

Effort: S.

- [x] README: new attributes, DoCommand reference, motion-service recipe, frame-system example
      with gripper offset, troubleshooting section for decoded errors.
- [x] `CHANGELOG.md` and semantic versioning: Phase 1 ships as 0.2.0, Phase 2 as 0.3.0, and so on.
- [x] Registry listing: update `meta.json` descriptions; add a `markdown_link` per model.
- [ ] Upstream: open a Python SDK issue (or PR) to implement `MoveThroughJointPositions`,
      `MoveThroughJointPositionsStreamed`, and `Get3DModels` in `ArmRPCService`, so the servicer
      swap in Phase 1 can be removed later.

## Suggested order and rough sizing

| Order | Phase | Size | Needs hardware |
|---|---|---|---|
| 1 | 0: tests and CI | S-M | no |
| 2 | 1: motion-service compatibility | L | yes (final validation) |
| 3 | 2: geometry and 3D models | M | no (planner test yes) |
| 4 | 3: safety and diagnostics | M-L | yes |
| 5 | 4: gripper parity | S-M | yes |
| 6 | 5: docs and release | S | no |

## Risks and open questions

- **Serial bandwidth.** Streaming 6 setpoints plus feedback polls per tick may cap `move_hz` well
  below 50 Hz on the 921600-baud bridge. Measure early in Phase 1; if too low, interpolate in the
  module but send at the achievable rate and lean on POS_VEL's onboard velocity cap.
- **Private SDK API.** Replacing `Registry._APIS` is unsupported. Pin `viam-sdk` and test the
  swap in CI; remove once the SDK implements the RPCs.
- **Mesh licensing.** Seeed's description package is CERN-OHL-W-2.0 (hardware) with Apache-2.0
  code; confirm redistribution terms for the STLs before bundling and keep attribution.
- **Blocking RPCs.** `MoveThroughJointPositions` may run for many seconds; ensure the servicer
  honors the stream deadline and that `stop()` from another RPC preempts it.
