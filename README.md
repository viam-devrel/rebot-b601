# rebot-b601

A [Viam](https://www.viam.com) module for the [Seeed Studio reBot Arm B601-DM](https://github.com/Seeed-Projects/reBot-DevArm), a 6 DoF robotic arm with Damiao CAN motors plus a parallel gripper. Talks to the arm through its USB-CAN serial bridge using Seeed's [motorbridge](https://motorbridge.seeedstudio.com) SDK.

## Models

| Model | API | Description |
|---|---|---|
| `devrel:rebot-b601:arm` | `rdk:component:arm` | The 6 arm joints (CAN IDs `0x01`–`0x06`) |
| `devrel:rebot-b601:gripper` | `rdk:component:gripper` | The parallel gripper (CAN ID `0x07`) |

Both components share one serial connection; the module multiplexes them onto the same bus.

## Prerequisites

- The arm's USB-CAN board plugged into the machine (enumerates as an `HDSC CDC Device`, typically `/dev/ttyACM0`).
- Serial-port access for the viam-server user: `sudo usermod -aG dialout $USER` (then re-login), or a udev rule.
- The arm **zeroed** (see Calibration below). Joint angles are relative to the motors' stored zero position.
- `uv` or `python3 -m venv` on the machine; `run.sh` bootstraps a virtualenv on first start.

## Example configuration

```json
{
  "components": [
    {
      "name": "arm",
      "model": "devrel:rebot-b601:arm",
      "type": "arm",
      "attributes": {
        "port": "/dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00",
        "speed_deg_s": 60,
        "motion": "builtin"
      },
      "frame": { "parent": "world" }
    },
    {
      "name": "gripper",
      "model": "devrel:rebot-b601:gripper",
      "type": "gripper",
      "attributes": { "arm": "arm" },
      "frame": { "parent": "arm" }
    }
  ]
}
```

The gripper's frame origin is the arm's end link (the gripper mount), so a zero translation is correct. The arm's kinematics stop at the mount; the gripper component supplies the gripper body and finger geometry.

### Arm attributes

| Attribute | Type | Default | Description |
|---|---|---|---|
| `port` | string | auto-detected | Serial device of the USB-CAN bridge |
| `baud` | int | `921600` | Serial baud rate |
| `control_mode` | string | `"pos_vel"` | `"pos_vel"` (velocity-capped position) or `"mit"` (impedance) |
| `speed_deg_s` | number or [6] | `60` | Max joint speed, deg/s (clamped to 1–180) |
| `acceleration_deg_s2` | number or [6] | `200` | Max joint acceleration, deg/s² (clamped to 1–1000) |
| `move_hz` | number | `50` | Setpoint streaming rate for interpolated moves |
| `mit_kp` / `mit_kd` | number or [6] | Seeed defaults | MIT-mode gains |
| `joint_limits_deg` | [6][2] | conservative defaults | Soft limits. Targets outside them are **rejected** |
| `clip_targets` | bool | `false` | Clip out-of-limit targets (with a warning) instead of rejecting them |
| `bad_joints` | [int] | `[]` | Joint indices (0–5) to hold at their current position; excluded from targets and limit checks |
| `tolerance_deg` | number | `2.0` | Settle tolerance for blocking moves |
| `motion` | string | unset | Name of a motion service (usually `"builtin"`) used by `move_to_position` |
| `collision_geometry` | string | `"primitives"` | Collision bodies in the served URDF: `"primitives"` (one box per link), `"meshes"` (decimated vendor STLs), or `"none"` |
| `include_gripper_geometry` | bool | `false` | Attach the gripper-base box to the arm's end link. Leave off when the gripper component is configured, or the two will self-collide |
| `torque_limit_nm` | number or [6] | unset | Software collision stop: abort and hold when a joint's measured torque exceeds this for `torque_trip_polls` consecutive polls |
| `torque_trip_polls` | int | `3` | Consecutive over-limit polls (at 10 Hz) that count as a collision |
| `temperature_warn_c` | number | `60` | Log a warning when a motor is at or above this temperature |
| `temperature_limit_c` | number | `80` | Refuse and abort moves when a motor is at or above this temperature |
| `can_timeout_ms` | int | unset | Program the Damiao CAN watchdog so a motor disables itself if commands stop arriving |
| `reconnect` | bool | `true` | Reopen the serial bridge and restore motor state after a link failure |
| `enable_on_start` | bool | `true` | Enable torque when the component starts |
| `disable_torque_on_close` | bool | `false` | Let the arm go limp when the component closes (it will slump under gravity!) |
| `manual_mode_kp` / `manual_mode_kd` | number | `0` / `0.5` | MIT gains used in manual mode |
| `gravity_scale` | number | `1.0` | Scale of the gravity-compensation feed-forward in manual mode (`0` disables it) |
| `payload_kg` | number | `0` | Extra mass at the end link for gravity compensation |
| `gravity_vector` | [3] | `[0, 0, -9.81]` | Gravity in the arm's base frame, for non-upright mounts |

### Gripper attributes

| Attribute | Type | Default | Description |
|---|---|---|---|
| `arm` | string | unset | Name of the arm component. Declares the dependency and inherits `port`/`baud` from it |
| `port`, `baud` | | as above | Only needed without `arm` |
| `open_position_deg` | number | `-270` | Motor angle when fully open |
| `closed_position_deg` | number | `0` | Motor angle when fully closed |
| `speed_deg_s` | number | `900` | Gripper motor speed (10–3000) |
| `torque_ratio` | number | `0.07` | Max grip force, `(0, 1]` |
| `holding_threshold_deg` | number | `15` | Stall distance from fully closed that counts as "holding something" |
| `stall_polls` | int | `4` | Consecutive near-zero-velocity polls that count as settled |
| `move_timeout_s` | number | `6` | Give up waiting for a grab/open after this long |
| `collision_geometry` | string | `"primitives"` | Same options as the arm |
| `reconnect` | bool | `true` | As for the arm |

## Motion

- `move_to_joint_positions` and `move_through_joint_positions` take **degrees**. The module plans a
  piecewise-linear joint-space path with a trapezoidal velocity profile bounded by `speed_deg_s` and
  `acceleration_deg_s2`, streams setpoints at `move_hz`, and blocks until the arm settles.
- Per-call overrides: the request's `MoveOptions` (`max_vel_degs_per_sec`, `max_acc_degs_per_sec2`,
  or the per-joint lists), or `extra` keys `speed_d`/`speed_r`, `acceleration_d`/`acceleration_r`,
  `move_hz`, `direct` (send the final target only), `interpolate: false`, `waitAtEnd: false`.
- `move_to_position` is delegated to the motion service named by `motion`; without it the call
  fails with an explanation. The motion service plans against the URDF and collision geometry
  from `get_kinematics` and executes through `MoveThroughJointPositions`, which this module serves
  even though the Python SDK's stock arm servicer does not.
- Streamed trajectories (`MoveThroughJointPositionsStreamed`) are paced by each point's timestamp.
- `stop()` cancels any in-flight move within one setpoint tick and holds the current position.
- A new move cancels a running one. `is_moving` is true while a move or stream is in flight.

## Safety and diagnostics

- Before every move the module reads all six motors. A transient fault (communication loss) is
  cleared automatically; a hard fault (over-current, over-/under-voltage, over-temperature,
  overload) rejects the move with the decoded reason. Fix the cause, then send
  `{"clear_errors": true}`.
- During a move it samples torque, status, and temperature at 10 Hz. A `torque_limit_nm` trip, a
  fault, or an over-temperature reading aborts the move and holds position.
- If the serial bridge disappears mid-session, the module reopens it (with backoff), re-enables
  the motors, and restores their control mode. Set `reconnect: false` to fail fast instead.
- On startup the arm enables torque and holds its current position; it does not move until commanded.
- Default speeds are gentle (60 deg/s). Keep the workspace clear the first time you command a move.

## DoCommand reference

Arm:

| Command | Effect |
|---|---|
| `{"status": true}` (also `get_state`, `health`) | Per-joint decoded status, position, velocity, torque, temperatures, plus manual-mode and torque flags |
| `{"raw_state": true}` | Raw per-joint position/velocity/torque/temperature |
| `{"load": true}` | Per-joint torque (Nm) |
| `{"set_speed": 45}` / `{"set_acceleration": 300}` | Change the default speed / acceleration (number or list of 6) |
| `{"get_speed": true}` / `{"get_acceleration": true}` | Read them back |
| `{"torque": "disable"}` | Go limp so you can move the arm by hand (`"enable"` to re-stiffen) |
| `{"clear_errors": true}` (also `clear_error`) | Clear latched motor faults and re-enable |
| `{"set_zero_position": true}` | Store the current pose as zero (see Calibration) |
| `{"manual_mode": "enter"}` / `"exit"` (also `enter_manual_mode` / `exit_manual_mode`) | Teaching mode: MIT mode with damping and gravity compensation; servos stay on. **Experimental**, see below |
| `{"gravity_torques": true}` | The feed-forward torques manual mode would apply at the current pose |

Gripper:

| Command | Effect |
|---|---|
| `{"get": true}` | Motor degrees and open fraction (0 closed, 1 open) |
| `{"set": -135}` / `{"set": {"fraction": 0.5}}` | Move to a motor angle or an open fraction and wait for stall/arrival |
| `{"set_speed": 500}` / `{"get_speed": true}` | Gripper speed, deg/s |
| `{"set_force": 0.2}` / `{"get_force": true}` | Grip force ratio `(0, 1]` (also `set_torque`/`get_torque`) |
| `{"grab_with_force": {"position": 0, "speed": 800, "force": 0.5}}` | One-shot grab with explicit target, speed, and force; `fraction` may replace `position` |
| `{"status": true}` (also `raw_state`, `health`) | Decoded status, position, temperatures, open fraction, holding flag |
| `{"torque": "disable"}` / `{"clear_errors": true}` / `{"set_zero_position": true}` | As for the arm |

### Manual mode

Manual mode switches the joints to MIT mode with `manual_mode_kp` (default 0) and `manual_mode_kd`
damping, and streams a gravity-compensation torque computed from the vendor URDF's link masses and
centres of mass at 50 Hz. It has **not been validated on hardware yet**: start with
`gravity_scale: 0.3`, keep a hand on the arm, and raise the scale until the arm floats. Set
`gravity_scale: 0` for damping only. `{"torque": "disable"}` remains the fallback.

## Kinematics, geometry, and 3D models

- `get_kinematics` serves the bundled URDF with `<collision>` bodies added per `collision_geometry`.
  In `meshes` mode the decimated STLs are shipped in the same response, so viam-server needs no files.
- `get_geometries` returns the per-link bounding boxes posed by the current joint state.
- `Get3DModels` returns per-link GLB visual meshes for the app's 3D view.
- The gripper serves a one-DoF URDF (left finger on a prismatic joint, right finger as a static
  envelope). Its kinematic input is the left finger's travel in metres, 0 (closed) to 0.05 (open).
- Meshes come from Seeed's `reBot-DevArm` repository (see `src/rebot_b601/assets/ATTRIBUTION.md`).
  Rebuild them with `python tools/build_assets.py`.

## Calibration

Joint angles are relative to each motor's stored zero. To (re)zero:

1. `{"torque": "disable"}` on both components.
2. Manually move the arm to its zero pose (the folded "sit-down" pose from the Seeed manual) and fully close the gripper.
3. Send `{"set_zero_position": true}` to the arm and gripper.

If you already calibrated via Seeed's LeRobot flow, the zeros are stored in the motors and nothing more is needed.

## Development

```sh
make venv            # uv venv + runtime and dev dependencies
make test            # pytest against an in-memory motorbridge fake (no hardware)
make lint            # ruff
make module          # module.tar.gz for the registry (no bytecode)
make check-bootstrap # run.sh on a clean copy, as viam-server would
.venv/bin/python tests/smoke_hardware.py   # read-only hardware check
```

Tags matching `v*` publish to the registry through `.github/workflows/deploy.yml`.

## Comparison with the uFactory xArm module

Analysis date: 2026-09-04, against `viam:ufactory` v1.1.34 (Go) and this module v0.1.0 (Python,
viam-sdk 0.80.0). The items marked *done in 0.2.0* were implemented from that analysis; the work
plan is in [PLAN.md](PLAN.md).

### The blocker that was found: motion-service execution

The RDK's builtin motion service executes planned paths by calling the arm's
`MoveThroughJointPositions` RPC (the RDK arm client's `GoToInputs` delegates to it with no
fallback). The Python SDK 0.80.0 arm servicer does not implement that RPC, nor
`MoveThroughJointPositionsStreamed` or `Get3DModels`, so any Python arm module gets UNIMPLEMENTED
when the motion service runs a plan. This module now swaps in its own servicer
(`src/rebot_b601/arm_service.py`) to provide them.

### Feature comparison (v0.1.0 → v0.2.0)

| Feature | uFactory (`viam:ufactory`) | B601-DM v0.1.0 | B601-DM v0.2.0 |
|---|---|---|---|
| Joint moves | Interpolated multi-waypoint, servo-streamed | Single target, blocks until settled | Interpolated, streamed at `move_hz` |
| MoveThroughJointPositions | Yes, plus streamed variant | No | Yes, plus streamed variant |
| Trajectory generator hookup | Optional external ML model service | No | No (not planned) |
| Per-call speed/accel overrides | MoveOptions and `extra` keys | No | Yes, same keys |
| MoveToPosition | Delegates to a motion service | Raises NotImplemented | Delegates to `motion` |
| Kinematics | URDF, variant auto-detected | Bundled URDF | Bundled URDF, one variant exists |
| Collision geometry | Boxes, URDF meshes opt-in | None | Boxes, meshes opt-in |
| Get3DModels (GLB meshes) | Yes | No | Yes |
| Joint-limit enforcement | Rejects with an error | Clips silently | Rejects (clip opt-in) |
| Error state handling | Decoded error tables, auto-clear | `clear_errors` only | Decoded Damiao status, auto-clear of transient faults |
| Collision detection | Firmware sensitivity levels | None | Software torque threshold |
| Locked or bad joints | `bad-joints` | No | `bad_joints` |
| Manual / teaching mode | Gravity-compensated, servos on | Torque disable | Experimental gravity compensation from URDF inertials |
| Torque readout | `load` | `raw_state` | `load`, `status`, `raw_state` |
| Gripper models | Two-finger, lite, vacuum, vacuum lite, F/T sensor | One parallel gripper | One parallel gripper (hardware has no others) |
| Gripper force control | Set/get torque, grab with torque | Fixed config | `set_force`, `grab_with_force` |
| Gripper geometry / kinematics | Yes | No | Yes (one prismatic DoF) |
| Gripper attaches to arm | `arm` dependency | Shared bus singleton | `arm` dependency + shared bus |
| Vendor UI access | UFactory Studio proxy | n/a | n/a (no vendor UI) |
| Build and release | Cloud build, deploy action, tests, lint | Hand-built tarball | Tests, lint, bootstrap check, tagged-release deploy |
| Tests | About 1250 lines of Go tests | Spatial math only | Unit suite against an in-memory motorbridge fake |

### Not applicable to this hardware

UFactory Studio proxy, force/torque sensor, vacuum grippers, G2 object-detected register,
firmware collision sensitivity, hardware variant detection, dedicated gripper bus.

### Still open after 0.2.0

- Hardware validation of streamed moves, the software collision stop, the reconnect path, and
  manual mode. Everything in 0.2.0 was tested against the in-memory fake only.
- Measuring the setpoint rate the 921600-baud bridge sustains and adjusting the `move_hz` default.
- An upstream Python SDK change so the servicer swap can be removed.
