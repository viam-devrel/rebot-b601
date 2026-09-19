# rebot-b601

A [Viam](https://www.viam.com) module for the [Seeed Studio reBot Arm B601](https://github.com/Seeed-Projects/reBot-DevArm), a 6 DoF robotic arm plus a parallel gripper, using Seeed's [motorbridge](https://motorbridge.seeedstudio.com) SDK. It drives both variants: the **B601-DM** (Damiao motors behind a USB-CAN serial bridge, the default) and the **B601-RS** (RobStride motors on plain CAN, `variant: "rs"`).

## Models

| Model | API | Description |
|---|---|---|
| `devrel:rebot-b601:arm` | `rdk:component:arm` | The 6 arm joints (CAN IDs `0x01`–`0x06`) |
| `devrel:rebot-b601:gripper` | `rdk:component:gripper` | The parallel gripper (CAN ID `0x07`) |

On the DM arm both components share one serial connection; the module multiplexes them onto the same bus.

## Prerequisites

- **B601-DM:** the arm's USB-CAN board plugged into the machine. It enumerates as an `HDSC CDC Device`
  (USB `2e88:4603`) and is found by that identity, never by guessing a `/dev/ttyACM*` number.
- **B601-DM:** serial-port access for the viam-server user: `sudo usermod -aG dialout $USER` (then re-login),
  or a udev rule.
- The arm **zeroed** (see Calibration below). Joint angles are relative to the motors' stored zero position.
- `uv` or `python3 -m venv` on the machine; `run.sh` bootstraps a virtualenv on first start.
- **B601-RS:** plain CAN at 1 Mbps instead of serial. `port` is the CAN channel and is required; `baud` is
  ignored and `can_timeout_ms` is rejected. Bringing the interface up: see B601-RS below.

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

B601-RS arm on a CAN interface:

```json
{ "name": "arm", "model": "devrel:rebot-b601:arm", "type": "arm",
  "attributes": { "variant": "rs", "port": "can0", "control_mode": "mit" } }
```

### B601-RS

The RS arm is six RobStride motors on plain CAN at 1 Mbps (rs-06 for joints 1–3, rs-00 for joints 4–6),
addressed by the module as host id `0xFD`. Seeed run the RS arm in MIT mode, and on the bench `pos_vel` with the motors' stored gains stopped about 3° short of its targets ("move timed out" warnings). On RS each `pos_vel` setpoint
costs two parameter writes per joint, so `control_mode: "mit"` is usually the better default although
`pos_vel` remains the module's.

On Linux a CANable/candleLight adapter shows up as SocketCAN: bring it up with
`sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up`, then set `"port": "can0"`.
On macOS a PEAK PCAN-USB goes through motorbridge's PCAN backend, which dlopens the MacCAN PCBUSB runtime by
bare name; install it so `libPCBUSB.dylib` sits in `/usr/local/lib` (`run.sh` exports `DYLD_LIBRARY_PATH`
pointing there) and set `"port": "can0"` or `"PCAN_USBBUS1"`. macOS strips `DYLD_LIBRARY_PATH` when it execs
a SIP-protected binary, so the venv must come from `uv` (which `run.sh` prefers) or another non-system
Python; one built from `/usr/bin/python3` drops the export and motorbridge reports `load PCBUSB failed`.

Joint positions still work when the motors send no status frames (they are read as parameters), including
with torque off, when stopped RobStride motors stop streaming and would otherwise report a frozen frame; the health
report then shows `position_only: true` and the arm moves without fault, temperature or torque checks (see
Safety and Troubleshooting).

The module serves the RS arm's own kinematic model: a bundled RS URDF, RS collision boxes and decimated
meshes, and per-link GLB visuals. `get_end_position`, `get_kinematics`, `get_geometries`, `Get3DModels`
and motion-service `move_to_position` all describe the RS arm. The RS URDF comes from Seeed's
[`reBotArm_control_py`](https://github.com/Seeed-Projects/reBotArm_control_py) repository,
`urdf/RS/urdf/ReBot_Arm_RS.urdf` at commit `76512eab38ba54f11830e7cdfcba68e11629f902`, with Seeed's
`gripper_end`/`j_gripper_end` renamed `end_link`/`end_joint` so the mount frame is named as on DM. Both
mount frames sit ahead of the hardware: DM's at the finger plane, 155 mm from link6; RS's 166 mm from
link6, about 73 mm past the front of the RS gripper body. At zero the RS end mount is at x 301.7,
z 217.7 mm. The RS arm rests in the same folded posture as DM, with positive joint 2/3 angles where DM
uses negative.

Not on RS yet: the gripper component (RS ships `gripper_end` as a mount, not a 1-DoF gripper) refuses to
attach to an RS arm, and discovery finds DM boards only. Manual mode is damping only until gravity
compensation is checked on the bench: the RS mass model exists but is unverified, so `gravity_scale` is
forced to 0 and `{"gravity_torques": true}` reports the model's numbers with an "unverified" note rather
than applying them.

Licence: the upstream `reBotArm_control_py` repository ships no LICENSE file and no SPDX headers at the
pinned commit. `src/rebot_b601/assets/rs/ATTRIBUTION.md` redistributes the RS assets under
CERN-OHL-W-2.0 (hardware) / Apache-2.0 (code), assumed by analogy with the sibling `reBot-DevArm` package
that is licensed that way and is the DM source. Confirm this with Seeed before a registry release.

### Arm attributes

| Attribute | Type | Default | Description |
|---|---|---|---|
| `variant` | string | `"dm"` | `"dm"` for the B601-DM (Damiao motors, USB serial bridge) or `"rs"` for the B601-RS (RobStride motors, CAN). `"rs"` requires `port` |
| `port` | string | auto-detected | Serial device of the USB-CAN bridge (dm), or the CAN channel such as `can0` or `PCAN_USBBUS1` (rs) |
| `baud` | int | `921600` | Serial baud rate (dm; ignored on rs) |
| `control_mode` | string | `"pos_vel"` | `"pos_vel"` (velocity-capped position) or `"mit"` (impedance) |
| `speed_deg_s` | number or [6] | `60` | Max joint speed, deg/s (clamped to 1–180) |
| `acceleration_deg_s2` | number or [6] | `200` | Max joint acceleration, deg/s² (clamped to 1–1000) |
| `move_hz` | number | `50` | Setpoint streaming rate for interpolated moves |
| `mit_kp` / `mit_kd` | number or [6] | Seeed defaults, per variant | MIT-mode gains |
| `joint_limits_deg` | [6][2] | conservative defaults, per variant | Soft limits. Targets outside them are **rejected**. The RS URDF mirrors DM on joints 2 and 3, which span 0–180° there (folding reads positive); the RS default lower edge is -5° because an arm whose zero is a degree off rests slightly negative |
| `clip_targets` | bool | `false` | Clip out-of-limit targets (with a warning) instead of rejecting them |
| `bad_joints` | [int] | `[]` | Joint indices (0–5) to hold at their current position; excluded from targets and limit checks |
| `tolerance_deg` | number | `2.0` | Settle tolerance for blocking moves |
| `motion` | string | unset | Name of a motion service (usually `"builtin"`) used by `move_to_position` |
| `collision_geometry` | string | `"primitives"` | Collision bodies in the served URDF: `"primitives"` (one box per link), `"meshes"` (decimated vendor STLs), or `"none"` |
| `include_gripper_geometry` | bool | `false` | Attach the gripper body box to `end_link` (DM: the `gripper_base` asset; RS: the `gripper_end` body). Finger GLBs exist only for DM, so RS 3D models show the body without fingers. Leave off when the gripper component is configured, or the two will self-collide |
| `torque_limit_nm` | number or [6] | unset | Software collision stop: abort and hold when a joint's measured torque exceeds this for `torque_trip_polls` consecutive polls. Works on RS: a holding joint reports 1–2 Nm; torque reads 0 only while the motor is unpowered or known by position alone |
| `torque_trip_polls` | int | `3` | Consecutive over-limit polls (at 10 Hz) that count as a collision |
| `temperature_warn_c` | number | `60` | Log a warning when a motor is at or above this temperature |
| `temperature_limit_c` | number | `80` | Refuse and abort moves when a motor is at or above this temperature |
| `can_timeout_ms` | int | unset | Program the Damiao CAN watchdog so a motor disables itself if commands stop arriving (dm; rejected on rs) |
| `reconnect` | bool | `true` | Reopen the bus (serial bridge or CAN channel) and restore motor state after a link failure |
| `enable_on_start` | bool | `true` | Enable torque when the component starts |
| `disable_torque_on_close` | bool | `false` | Let the arm go limp when the component closes (it will slump under gravity!) |
| `manual_mode_kp` / `manual_mode_kd` | number | `0` / `0.5` | MIT gains used in manual mode |
| `gravity_scale` | number | `1.0` | Scale of the gravity-compensation feed-forward in manual mode (`0` disables it). Ignored on RS: forced to 0 until the RS mass model is verified on the bench |
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
- A new move cancels a running one. `is_moving` is true while a move or stream is in flight. On the DM arm it
  also reports motion the module did not command (a joint moving faster than 0.05 rad/s); on RS the
  status-frame velocity is not a measurement (a motor at rest reported -0.15 rad/s on the bench), so
  `is_moving` reflects only this module's own moves.

## Safety and diagnostics

- Before every move the module reads all six motors. A transient fault (communication loss) is
  cleared automatically; a hard fault (over-current, over-/under-voltage, over-temperature,
  overload on DM; undervoltage, over-current, over-temperature, magnetic or HALL encoder fault,
  not calibrated on RS) rejects the move with the decoded reason. Fix the cause, then send
  `{"clear_errors": true}`. An RS arm that sends no status frames has nothing to check and moves
  anyway (see B601-RS).
- During a move it samples torque, status, and temperature at 10 Hz. A `torque_limit_nm` trip, a
  fault, or an over-temperature reading aborts the move and holds position.
- If the bus (serial bridge or CAN channel) disappears mid-session, the module reopens it (with
  backoff), re-enables the motors, and restores their control mode. Set `reconnect: false` to fail
  fast instead.
- On startup the arm enables torque and holds its current position; it does not move until commanded.
- Default speeds are gentle (60 deg/s). Keep the workspace clear the first time you command a move.

## DoCommand reference

Arm:

| Command | Effect |
|---|---|
| `{"status": true}` (also `get_state`, `health`) | Per-joint decoded status, position, velocity, torque, temperatures, plus manual-mode and torque flags |
| `{"raw_state": true}` | The same per-joint dict as the health report (position, velocity, torque, temperatures, `can_id`, `status_code`, `fault`, `position_only`) |
| `{"load": true}` | Per-joint torque (Nm) |
| `{"set_speed": 45}` / `{"set_acceleration": 300}` | Change the default speed / acceleration (number or list of 6) |
| `{"get_speed": true}` / `{"get_acceleration": true}` | Read them back |
| `{"torque": "disable"}` | Go limp so you can move the arm by hand (`"enable"` to re-stiffen) |
| `{"clear_errors": true}` (also `clear_error`) | Clear latched motor faults and re-enable |
| `{"set_zero_position": true}` | Store the current pose as zero (see Calibration) |
| `{"manual_mode": "enter"}` / `"exit"` (also `enter_manual_mode` / `exit_manual_mode`) | Teaching mode: MIT mode with damping and gravity compensation; servos stay on. **Experimental**, see below |
| `{"gravity_torques": true}` | The feed-forward torques manual mode would apply at the current pose. On DM, the list of scaled, clamped torques; on RS, `{"torques_nm": [...], "note": ...}` with the model's six unscaled torques, since nothing is applied until the bench check passes |

Each joint dict from `get_state` / `health` / `raw_state` carries `position_only`: true means the joint was read
through its `mechPos` parameter rather than a status frame, so its velocity, torque, temperature and fault are
unknown. `{"load": true}` returns null for such a joint.

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
damping, and streams a gravity-compensation torque computed from the variant URDF's link masses and
centres of mass at 50 Hz. It has **not been validated on hardware yet**: start with
`gravity_scale: 0.3`, keep a hand on the arm, and raise the scale until the arm floats. Set
`gravity_scale: 0` for damping only, which is all an RS arm does (see B601-RS).
`{"torque": "disable"}` remains the fallback.

## Kinematics, geometry, and 3D models

Each variant serves its own bundled URDF (`rebot_b601_dm.urdf`, `rebot_b601_rs.urdf`) with its own
collision boxes, meshes and GLBs; the arm picks the model from `variant`.

- Served joint limits differ. DM serves its URDF's own ranges, so its `get_kinematics` payload is
  byte-identical to 0.2.0 (pinned by `tests/test_dm_baseline.py`). RS serves the module's soft
  `joint_limits_deg`: the RS arm rests about a degree below its URDF's 0 on joints 2 and 3, and
  viam-server rejects any target outside the served limits.
- `get_kinematics` serves the bundled URDF with `<collision>` bodies added per `collision_geometry`.
  In `meshes` mode the decimated STLs are shipped in the same response, so viam-server needs no files.
- `get_geometries` returns the per-link bounding boxes posed by the current joint state.
- `Get3DModels` returns per-link GLB visual meshes for the app's 3D view.
- The gripper serves a one-DoF URDF (left finger on a prismatic joint, right finger as a static
  envelope). Its kinematic input is the left finger's travel in metres, 0 (closed) to 0.05 (open).
- DM meshes come from Seeed's `reBot-DevArm` repository (`src/rebot_b601/assets/ATTRIBUTION.md`), RS
  meshes from `reBotArm_control_py` (`src/rebot_b601/assets/rs/ATTRIBUTION.md`; see the licence note under
  B601-RS). Rebuild with `python tools/build_assets.py --variant {dm,rs}`.
- Every collision STL is decimated to a 150 KB cap. The RS source meshes are dense (164k triangles on
  link3, reduced to about 3k), which leaves roughly 2 mm feature resolution. On RS, `base_link`, `link1`
  and `link6` have no separate visual parts upstream, so their GLBs come from the shared collision mesh
  and look coarse.

## Calibration

Joint angles are relative to each motor's stored zero. To (re)zero:

1. `{"torque": "disable"}` on both components.
2. Manually move the arm to its zero pose (the folded "sit-down" pose from the Seeed manual) and fully close the gripper.
3. Send `{"set_zero_position": true}` to the arm and gripper.

If you already calibrated via Seeed's LeRobot flow, the zeros are stored in the motors and nothing more is needed.

## Discovery

Add the `devrel:rebot-b601:discovery` service (no attributes) and open its **Test** panel: it lists a
ready-to-paste arm and gripper config for every attached B601-DM board, with `port` set to its stable
`/dev/serial/by-id/...` path. Discovery identifies boards by USB vendor/product id from sysfs and never
opens a serial port, so it is safe to run next to other serial devices.

```json
{ "name": "rebot-discovery", "api": "rdk:service:discovery", "model": "devrel:rebot-b601:discovery" }
```

`{"serial_ports": true}` via DoCommand lists every USB serial device on the machine with its USB id,
which is the quickest way to see who owns which `/dev/ttyACM*`.

Leaving `port` unset is fine on the DM arm: it resolves to the B601 the same way (`variant: "rs"`
requires it). If no board is
attached the component fails with `no B601 USB-CAN board found` and the list of serial devices that
are present, rather than opening something else.

## Troubleshooting

**`Unable to acquire exclusive lock on serial port`** means the USB-CAN board is present but another
open file descriptor holds it. The error names the holder when it can be seen from `/proc`:

- *held by this module process itself*: a previous controller leaked its descriptor. The module
  closes it and retries automatically; if it still fails, restart the module (`sudo systemctl
  restart viam-server`).
- *held by pid N (...)*: another program has the port (a LeRobot or motorbridge session, a stale
  hot-reloaded module, a second viam-server). Stop it, or `sudo fuser -v /dev/ttyACM*` to find it.

Both the arm and the gripper resolve `port` to the real device before opening it, so
`/dev/ttyACM0` and its `/dev/serial/by-id/...` symlink share one connection.

**`motor did not reply (... not received within 100ms)`** means the serial link is fine but a motor
stayed silent: check arm power and the CAN daisy chain. The module no longer reopens the port for this,
since another driver probing the same tty can produce exactly this symptom.

- **RS: `joint2 target -1.5 deg is outside the limits` at or near the rest pose** — the arm rests a little
  below 0° when its zero is not exact. Re-zero it at rest (Calibration), or widen `joint_limits_deg`. The
  default lower edge on joints 2 and 3 is -5°.
- **RS: `move timed out after 2.0s; position error [..., 2.9]`** in `pos_vel` — the motors' stored
  position-loop gains are soft. Use `control_mode: "mit"`.
- **RS: `joint1 (0x01) reports undervoltage`** — the rs-06 motors flag a low supply in their status frame and the
  module refuses to move until it clears. Check the supply voltage, then `{"clear_errors": true}`.
- **RS: log says `no status frames from motor(s) ...; positions read from mechPos`** — the motors are not
  streaming status; positions still work through parameter reads but velocity, torque, temperature and faults
  are unknown, and the arm moves without those checks. Power-cycle the motors if it persists after a restart.

## Development

```sh
make venv            # uv venv + runtime and dev dependencies
make test            # pytest against an in-memory motorbridge fake (no hardware)
make lint            # ruff
make module          # module.tar.gz for the registry (no bytecode)
make check-bootstrap # run.sh on a clean copy, as viam-server would
make assets          # rebuild the RS meshes, GLBs and collision boxes (= make assets-rs)
make assets-dm       # rebuild DM's, deliberately: decimation output drifts with the trimesh/numpy
                     # build, so a rebuild changes the bytes tests/test_dm_baseline.py pins and
                     # that baseline has to be recaptured. requirements-dev.txt pins trimesh==5.1.0
.venv/bin/python tests/smoke_hardware.py                                  # DM, read-only
DYLD_LIBRARY_PATH=/usr/local/lib .venv/bin/python tests/smoke_hardware.py --variant rs --port can0          # RS, read-only (macOS prefix)
DYLD_LIBRARY_PATH=/usr/local/lib .venv/bin/python tests/smoke_hardware.py --variant rs --port can0 --move   # MOVES joint 6 after an explicit Enter
DYLD_LIBRARY_PATH=/usr/local/lib .venv/bin/python tests/smoke_hardware.py --variant rs --port can0 --gravity-check   # torque on, holds; measured vs model torque per joint (stop viam-server first)
```

The smoke script prints FK for the variant it ran against (`end mount (FK, <variant>)`). The RS read-only
run is interactive: it pauses once for a staleness check (move a joint by hand, then press
Enter) before reading the joints again. `--move` enables torque and moves the arm; the arm refuses with its
own fault message (undervoltage, for example) if any motor reports a fault. Flag abbreviations are disabled,
so `--m` is an error, not a move.

Creating a GitHub release (tag `0.5.0` or `v0.5.0`) publishes to the registry through
`.github/workflows/deploy.yml`; the workflow can also be run by hand with a version input.

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
| Kinematics | URDF, variant auto-detected | Bundled URDF | Bundled URDF (DM only) |
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
firmware collision sensitivity, dedicated gripper bus.

### Still open after 0.2.0

- Hardware validation of streamed moves, the software collision stop, the reconnect path, and
  manual mode. Everything in 0.2.0 was tested against the in-memory fake only.
- Measuring the setpoint rate the 921600-baud bridge sustains and adjusting the `move_hz` default.
- An upstream Python SDK change so the servicer swap can be removed.
- Hardware variant detection: `variant` is set by hand today, and discovery finds DM boards only.
