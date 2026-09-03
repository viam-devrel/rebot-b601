# rebot-b601

A [Viam](https://www.viam.com) module for the [Seeed Studio reBot Arm B601-DM](https://github.com/Seeed-Projects/reBot-DevArm) — a 6 DoF robotic arm with Damiao CAN motors plus a parallel gripper. Talks to the arm through its USB-CAN serial bridge using Seeed's [motorbridge](https://motorbridge.seeedstudio.com) SDK.

## Models

| Model | API | Description |
|---|---|---|
| `devrel:rebot-b601:arm` | `rdk:component:arm` | The 6 arm joints (CAN IDs `0x01`–`0x06`) |
| `devrel:rebot-b601:gripper` | `rdk:component:gripper` | The parallel gripper (CAN ID `0x07`) |

Both components may share the same `port`; the module multiplexes them onto one serial connection.

## Prerequisites

- The arm's USB-CAN board plugged into the machine (enumerates as an `HDSC CDC Device`, typically `/dev/ttyACM0`).
- Serial-port access for the viam-server user: `sudo usermod -aG dialout $USER` (then re-login), or a udev rule.
- The arm **zeroed** (see Calibration below). Joint angles are relative to the motors' stored zero position.

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
        "control_mode": "pos_vel",
        "speed_deg_s": 60
      }
    },
    {
      "name": "gripper",
      "model": "devrel:rebot-b601:gripper",
      "type": "gripper",
      "attributes": {
        "port": "/dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00"
      }
    }
  ]
}
```

### Arm attributes

| Attribute | Type | Default | Description |
|---|---|---|---|
| `port` | string | auto-detected | Serial device of the USB-CAN bridge |
| `baud` | int | `921600` | Serial baud rate |
| `control_mode` | string | `"pos_vel"` | `"pos_vel"` (velocity-capped position) or `"mit"` (impedance) |
| `speed_deg_s` | number or [6] | `60` | Max joint speed in `pos_vel` mode, deg/s |
| `mit_kp` / `mit_kd` | number or [6] | Seeed defaults | MIT-mode gains |
| `joint_limits_deg` | [6][2] | conservative defaults | Soft limits; every target is clipped |
| `tolerance_deg` | number | `2.0` | Settle tolerance for blocking moves |
| `enable_on_start` | bool | `true` | Enable torque when the component starts |
| `disable_torque_on_close` | bool | `false` | Let the arm go limp when the component closes (it will slump under gravity!) |

### Gripper attributes

| Attribute | Type | Default | Description |
|---|---|---|---|
| `port`, `baud` | | as above | Shared with the arm |
| `open_position_deg` | number | `-270` | Motor angle when fully open |
| `closed_position_deg` | number | `0` | Motor angle when fully closed |
| `speed_deg_s` | number | `900` | Gripper motor speed |
| `torque_ratio` | number | `0.07` | Max grip force, `(0, 1]` |
| `holding_threshold_deg` | number | `15` | Stall distance from fully closed that counts as "holding something" |

## Usage notes

- `move_to_joint_positions` takes **degrees**, clips to the soft joint limits, and blocks until the arm settles.
- `get_end_position` computes forward kinematics from the bundled URDF (validated against `pytransform3d`).
- `move_to_position` (cartesian) is intentionally not implemented on the component. Add the arm to the frame system and use the **motion service**; it plans with the URDF served by `get_kinematics`.
- `DoCommand` extras on both components:
  - `{"torque": "disable"}` — go limp so you can move the arm by hand (`"enable"` to re-stiffen)
  - `{"set_zero_position": true}` — store the current pose as zero (see Calibration)
  - `{"raw_state": true}` — position/velocity/torque per joint
  - `{"clear_errors": true}` (arm only)

## Calibration

Joint angles are relative to each motor's stored zero. To (re)zero:

1. `{"torque": "disable"}` on both components.
2. Manually move the arm to its zero pose (the folded "sit-down" pose from the Seeed manual) and fully close the gripper.
3. Send `{"set_zero_position": true}` to the arm and gripper.

If you already calibrated via Seeed's LeRobot flow, the zeros are stored in the motors and nothing more is needed.

## Safety

- On startup the arm enables torque and holds its current position; it does not move until commanded.
- Default speeds are gentle (60 deg/s). Keep the workspace clear the first time you command a move.
- The default soft limits are slightly conservative relative to the URDF limits; tighten them further for constrained installs.

## Development

```sh
uv venv .venv && uv pip install -p .venv/bin/python -r requirements.txt
.venv/bin/python tests/test_spatial.py      # kinematics + orientation math (no hardware)
.venv/bin/python tests/smoke_hardware.py    # read-only hardware check
```
