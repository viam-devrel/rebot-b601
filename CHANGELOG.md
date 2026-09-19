# Changelog

## 0.6.0 (2026-09-18)

### Added
- The gripper component runs on the B601-RS. `variant: "rs"` drives motor `0x07` (a RobStride rs-00) in
  profile position (`POS_VEL`), because RobStride has no force-limited position mode: `speed_deg_s` is a
  velocity limit and the firmware current limit is the only squeeze ceiling. RS requires `port` (the CAN
  channel; the `arm` dependency is a gRPC client under viam-server and cannot supply it) and
  `open_position_deg` (nothing records it for the B601-RS and it depends on where the jaws sat when the
  motor was zeroed).
- `tests/smoke_hardware.py --variant rs --port <chan> --gripper` jogs motor `0x07` by a signed step, with
  no clamping, so the fully open angle can be read off against the jaws' hard stop. That reading is
  `open_position_deg`.
- RS finger meshes, GLBs and collision boxes, so the RS gripper serves a real one-DoF kinematic model
  (`get_kinematics`, `get_geometries`, `Get3DModels`).

### Changed
- The gripper takes a `variant` attribute and refuses a port already open for the other vendor's motors.
- Each variant's parallel-jaw geometry comes from `spatial.MODELS[variant].gripper`: finger travel is
  0.0715 m per finger on RS against 0.05 on DM, so the kinematic input range differs by variant. DM's
  served payloads are unchanged.
- Settling is judged by variant: DM by near-zero velocity, RS by the position not changing (RS status
  velocity is not a measurement; a resting motor reads -0.15 rad/s). `stall_polls` counts either.
- The RS gripper's default `speed_deg_s` is 286.5 (5 rad/s, the vendor's limit) instead of DM's 900.
- `torque_ratio` is accepted on RS and ignored, with a warning at configure.
- The arm's served kinematic model ends at the tool mount, `link6`, where a tool bolts on. It used to
  run one fixed joint further, to the mount plate (`end_link`), which sits 155 mm (DM) / 166 mm (RS)
  past `link6`, beyond any hardware. The mount plate stays in the bundled URDF for its mass and its
  collision asset, and is no longer served.
- The end pose `get_end_position` reports, and that motion-service targets are expressed in, is the
  tool mount. It moves back 155 mm (DM) / 166 mm (RS) **and rotates**: at the zero pose DM reads
  x 104.9, z 191.7 mm and RS x 135.5, z 217.7 mm, both with orientation vector (1, 0, 0) and theta
  -180, where the mount plate read (0, 0, 1) and theta 0. The tool mount follows the +Z-as-approach-axis
  convention and the mount plate did not, so an existing pose target has to be re-expressed, not merely
  shifted, and taught poses have to be recaptured.
- The gripper's served model starts at the arm's tool mount and carries the offset to the gripper body
  itself. Gripper frame config is unchanged: parent `arm`, zero translation.
- The app's 3D view no longer shows the gripper. Its visual models were only ever served through
  `include_gripper_geometry`; the gripper's shape is still available through the gripper component's
  own geometries.

### Removed
- `include_gripper_geometry`. It could not survive the trim: the RDK keeps only the first `<collision>`
  per link and `link6` already has its own, so the gripper geometry would have been discarded with no
  warning. A config that still carries the attribute is ignored, with a warning at configure. A tool
  with no gripper component of its own is now described by a `geometry` in the arm's frame config,
  which is the idiomatic Viam answer.

### Fixed
- The RS meshes no longer arrive as floating shards. `tools/build_assets.py` decimated each part as one
  mesh, and because the source STLs store unshared vertices there was no edge to collapse, so the
  simplifier deleted triangles instead: `rs/meshes/link2.stl` was 3062 faces in 2762 loose pieces whose
  largest was 6 triangles. Decimation now welds vertices, works shell by shell, keeps a shell whole
  rather than cutting it below 48 faces, and drops the source's 1- and 2-triangle slivers. link2's
  collision mesh is now 42 shells, the largest 816 faces.
- The GLB byte budget assumed 28 bytes per face where the real figure is about 18, so every visual model
  stopped at ~10,970 faces and left a third of the 300 KB cap unspent. The RS visual models now carry
  15,455-16,998 faces at 280-300 KB. The caps themselves are unchanged.
- The DM meshes were rebuilt with the same fix. `meshes/base_link.stl` was 3062 faces in 2409 loose
  pieces whose largest was 7 triangles and is now 2949 faces in 59 shells, the largest 306; the worst
  remaining DM collision mesh is 58 shells against 2958 faces. The DM visual models now carry
  15,992-16,779 faces at up to 299 KB, where the byte-budget bug had capped them at ~10,970. Every DM
  mesh byte therefore moved, so `tests/test_dm_baseline.py` re-pins its mesh hashes and GLB sizes; the
  served URDF payloads are unchanged.

### Not yet on RS
- Force control: `set_force`/`get_force`, `grab_with_force` and their `torque` aliases are refused with
  an explanation. There is no torque ratio to set.
- Holding detection: `grab` returns `False` and `is_holding_something` reports false. Deciding that the
  jaws hold something needs a force signal RS does not provide, and a threshold tuned on hardware.
- Discovery still finds DM boards only.

### Known open items
- RS finger travel, 0.0715 m, comes from Seeed's CAD export (`ReBot_Arm_RS.csv`). The vendor URDF
  disagrees with itself, giving 0.05 on one finger and 0.0715 on the other, which cannot both describe a
  symmetric jaw. Check with calipers.
- DM finger travel is this module's own 0.05 m, while Seeed's DM vendor URDF says 0.0285 per finger. DM
  is left unchanged pending the same caliper measurement, so its served payload stays byte-identical
  (`kinematics.FINGER_TRAVEL_M` records the discrepancy).

## 0.5.0 (2026-09-18)

### Added
- The B601-RS serves its own kinematic model: a bundled RS URDF (Seeed `reBotArm_control_py` `urdf/RS` @ `76512ea`,
  mount link named `end_link` as on DM), RS collision boxes and decimated meshes, and per-link GLBs coloured from
  the URDF materials. `get_end_position`, `get_kinematics`, `get_geometries`, `Get3DModels` and motion-service
  `move_to_position` describe the RS arm.
- `tools/build_assets.py --variant {dm,rs}`; `make assets` builds RS, `make assets-dm` rebuilds DM deliberately.
- `{"gravity_torques": true}` on RS returns the RS model's torques with an "unverified" note instead of refusing.
- `tests/smoke_hardware.py --gravity-check` prints measured holding torque next to the model's gravity torque
  per joint, the bench check that gates RS gravity compensation.

### Changed
- `spatial` exposes `MODELS["dm"|"rs"]` and the FK, gravity and asset helpers live on the model; the DM-only module-level names are gone. DM's served kinematics
  payload is byte-identical to 0.4.0 (pinned by `tests/test_dm_baseline.py`).
- A model with no collision primitives logs a warning at configure.

### Not yet on RS
- Gravity compensation in manual mode (model exists, pending the bench check), the gripper component, discovery.

### Licence note
- Seeed's `reBotArm_control_py` repository, the source of the RS URDF and meshes, ships no licence file. The RS
  assets are redistributed under the CERN-OHL-W-2.0 terms assumed by analogy with the sibling `reBot-DevArm`
  package (see `src/rebot_b601/assets/rs/ATTRIBUTION.md`); confirm with Seeed before a registry release.

## 0.4.0 (2026-09-18)

### Added
- `variant: "rs"` on the arm drives the reBot Arm **B601-RS** (RobStride rs-06/rs-00 motors over CAN):
  joint reading, joint moves, streamed trajectories, stop, torque enable/disable, damping-only manual mode,
  health with RobStride fault names. `port` is the CAN channel (`can0` on Linux SocketCAN, `can0` or
  `PCAN_USBBUS1` on macOS PCAN). RS defaults: Seeed's MIT gains, RS soft joint limits, and active status
  reporting; when no status frame arrives, positions come from `mechPos` and the joint reports
  `position_only`. With torque off, stopped RobStride motors stop streaming and would report a frozen
  frame, so positions are read by parameter until torque is enabled again.
- On RS the served kinematics file carries the arm's soft `joint_limits_deg` instead of the DM URDF's
  ranges; viam-server checks joint targets against them, and the DM ranges rejected every RS target on
  joints 2 and 3.
- `run.sh` exports `DYLD_LIBRARY_PATH=/usr/local/lib` on macOS when the MacCAN runtime is installed.
- `tests/smoke_hardware.py --variant rs --port can0 [--move]`; the move step is gated behind an explicit
  Enter, refuses on any motor fault, and flag abbreviations are disabled so `--m` cannot move the arm.

### Changed
- `raw_state` returns the same per-joint dict as the health report (superset of the old keys: adds
  `can_id`, `status_code`, `fault`, `position_only`). `load` returns null for a joint known only by position.
- A move whose monitor gets no feedback from any joint now logs a warning instead of staying silent (DM too).

### Not yet on RS
- Kinematics, `get_end_position`, motion-service moves, gravity compensation, the gripper, and discovery
  remain DM-only. The gripper refuses to attach to an RS arm.
- `pos_vel` with the motors' stored gains stops a few degrees short of its target; use `control_mode: "mit"`
  on RS. Writing Seeed's position-loop gains at configure time is the follow-up if `pos_vel` is needed.
- `is_moving` on RS reflects only this module's own moves: the status-frame velocity is not a measurement
  (a resting motor reported -0.15 rad/s on the bench), so the velocity check is disabled.

## 0.3.2 (2026-09-09)

### Fixed
- Startup regression since 0.2.0: a Damiao motor is briefly busy answering `enable()` and misses the
  first `ensure_mode` register read. 0.1.0 retried that; 0.2.0 re-raised link-class errors at once, so
  every build failed with "register 10 not received within 100ms" on hardware a plain scan could see.
  Motor reply timeouts inside the enable/ensure_mode loop are retried again (up to 10 attempts).

## 0.3.1 (2026-09-09)

### Fixed
- The real cause of the serial-port lock-out: every motorbridge `Motor` handle holds its own reference
  to the serial bus, so `Controller.close()` alone left the port open and `Motor` has no destructor.
  `SharedBus` now frees every motor handle before closing the controller, on release, reconnect, and
  garbage collection. Verified against the real library.

## 0.3.0 (2026-09-08)

### Added
- `devrel:rebot-b601:discovery` service: finds every attached B601 by USB id (`2e88:4603`) from sysfs and
  proposes arm + gripper configs with the stable by-id `port` filled in. `{"serial_ports": true}` lists
  all USB serial devices with their identity.

### Changed
- Port auto-detection identifies the board by USB vendor/product id and no longer falls back to
  `/dev/ttyACM0`. With no board attached the component fails with a clear message instead of opening
  whatever device happens to be first.
- A motor that does not answer (`... not received within ...`) is reported as a motor fault; it no
  longer closes and reopens the serial port, which is how the port was lost to another driver.

### Fixed
- `SharedBus.acquire` reopens a bus that a failed reconnect left closed instead of returning a dead bus
  that fails every call with "is not open".
- The serial bridge could be locked out by the module's own process: `SharedBus` now caches by the
  resolved device path (so `/dev/ttyACM0` and its by-id symlink share one controller), a failed
  component build releases the port instead of pinning it, and every controller has a finalizer
  so a dropped bus still closes its descriptor.
- A refused exclusive lock now reports who holds the port (this process, or pid + command line of
  another). A descriptor leaked by this process is closed and the open retried once.
- Asking for a second baud rate on an already-open device is an error instead of being ignored.

## 0.2.0 (2026-09-08)

Feature parity pass against the `viam:ufactory` xArm module; see PLAN.md for the gap analysis.

### Added
- `MoveThroughJointPositions`, `MoveThroughJointPositionsStreamed`, and `Get3DModels` RPCs, via a
  custom arm servicer swapped into the Python SDK registry. The motion service can now execute plans.
- Interpolated, time-paced setpoint streaming with per-joint velocity/acceleration limits
  (`acceleration_deg_s2`, `move_hz`), plus per-call overrides through `MoveOptions` and `extra`
  (`speed_d`, `speed_r`, `acceleration_d`, `acceleration_r`, `direct`, `interpolate`, `waitAtEnd`).
- `move_to_position` through a configured motion service (`motion` attribute).
- Collision geometry in the served URDF (`collision_geometry`: `primitives` | `meshes` | `none`),
  `get_geometries` on arm and gripper, and GLB visual meshes for the app's 3D view.
- Gripper kinematics: one prismatic finger joint plus collision boxes; `arm` dependency attribute.
- Damiao status decoding, automatic clearing of transient faults, temperature warn/limit
  (`temperature_warn_c`, `temperature_limit_c`), software collision stop (`torque_limit_nm`,
  `torque_trip_polls`), CAN watchdog (`can_timeout_ms`), serial reconnect (`reconnect`), `bad_joints`.
- Experimental manual mode with URDF-based gravity compensation (`manual_mode` DoCommand,
  `manual_mode_kd`, `gravity_scale`, `payload_kg`, `gravity_vector`).
- DoCommands: arm `status`/`get_state`, `load`, `set_speed`, `set_acceleration`, `clear_error`,
  `gravity_torques`; gripper `get`, `set`, `set_speed`, `set_force`, `grab_with_force`, `status`.
- Unit-test suite with an in-memory motorbridge fake, ruff config, GitHub Actions for tests and
  tagged-release publishing, `make module` that excludes bytecode, `run.sh --check`.

### Changed (breaking)
- Joint targets outside `joint_limits_deg` are now rejected with an error instead of clipped.
  Set `clip_targets: true` for the old behaviour.
- Gripper `get_current_inputs`/`go_to_inputs` now use left-finger travel in metres (0 to 0.05),
  matching the served kinematics. Motor degrees remain available through the `get`/`set` DoCommands.
- `move_to_joint_positions` streams an interpolated trajectory instead of sending one setpoint.
  Pass `{"direct": true}` in `extra` for the old single-setpoint behaviour.

## 0.1.0 (2026-09-03)

Initial release: arm and gripper components over the USB-CAN bridge, URDF kinematics, FK,
torque enable/disable, zeroing, stall-based hold detection.
