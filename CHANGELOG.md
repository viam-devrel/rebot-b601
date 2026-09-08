# Changelog

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
