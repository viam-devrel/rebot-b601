"""Hardware smoke test for the B601 (DM or RS).

Read-only by default: prints joint state and forward kinematics (both variants) and never
enables torque. With --move it nudges joint 6 by +5 deg and back, then stops; torque is
enabled only after you confirm at an explicit prompt, and the move is refused with the
arm's own message if any motor reports a fault, a collision or an over-temperature.

Run:
  .venv/bin/python tests/smoke_hardware.py                          # DM, port auto-detected
  .venv/bin/python tests/smoke_hardware.py --variant rs --port can0  # RS, read-only
  .venv/bin/python tests/smoke_hardware.py --variant rs --port can0 --move

macOS + PEAK PCAN-USB: motorbridge dlopens libPCBUSB.dylib by bare name, so run with
DYLD_LIBRARY_PATH=/usr/local/lib (run.sh exports it for the module itself).
"""

import argparse
import asyncio
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.rebot_b601 import spatial  # noqa: E402
from src.rebot_b601.arm import VARIANT_VENDOR  # noqa: E402
from src.rebot_b601.bus import BusError, SharedBus, detect_port  # noqa: E402
from src.rebot_b601.damiao import JointHealth  # noqa: E402

NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]

# allow_abbrev=False: without it "--m"/"--mo"/"--mov" all mean --move and would move the arm.
ap = argparse.ArgumentParser(
    description=__doc__, allow_abbrev=False, formatter_class=argparse.RawDescriptionHelpFormatter
)
ap.add_argument("--port", help="serial device (dm, default: auto-detect) or CAN channel (rs, required)")
ap.add_argument("--variant", choices=("dm", "rs"), default="dm")
ap.add_argument("--move", action="store_true", help="enable torque and nudge joint 6 by 5 deg (MOVES THE ARM)")
args = ap.parse_args()

port = args.port
if not port:
    if args.variant == "rs":
        ap.error("--port is required for --variant rs (e.g. can0 or PCAN_USBBUS1)")
    port = detect_port()
vendor = VARIANT_VENDOR[args.variant]


def show(bus) -> list:
    states = bus.poll_feedback(list(range(1, 8)))
    positions = []
    for i, cid in enumerate(range(1, 8)):
        s = states[cid]
        if s is None:
            print(f"  {NAMES[i]} (0x{cid:02x}): NO FEEDBACK")
            continue
        h = JointHealth.from_state(cid, s, vendor)
        if h.position_only:
            # vel/torq/t_mos are placeholder zeros here, not measurements: don't print them.
            print(f"  {NAMES[i]} (0x{cid:02x}): pos={h.pos_deg:8.2f} deg  ({h.status})")
        else:
            print(
                f"  {NAMES[i]} (0x{cid:02x}): pos={h.pos_deg:8.2f} deg  vel={h.vel_rad_s:6.3f} rad/s  "
                f"torq={h.torque_nm:6.3f} Nm  t_mos={h.t_mos_c:.0f}C  status={h.status}"
            )
        if i < 6:
            positions.append(h.pos_deg)
    return positions


print(f"connecting to {port} ({args.variant}, {vendor}) ...")
try:
    bus = SharedBus.acquire(port, vendor=vendor)
except BusError as e:
    print(f"cannot open {port}: {e}")
    sys.exit(1)
if vendor == "robstride":
    # Ask the motors to stream status (a comms setting, not torque) so faults and
    # temperatures show up here too; without it every row is "position only".
    for cid in range(1, 8):
        bus.motor(cid).robstride_set_active_report(True)
    time.sleep(0.3)
positions = show(bus)
if len(positions) == 6:
    x, y, z, ox, oy, oz, theta = spatial.MODELS[args.variant].end_position(positions)
    print(
        f"\nend mount (FK, {args.variant}): x={x:.1f} y={y:.1f} z={z:.1f} mm  "
        f"o=({ox:.3f},{oy:.3f},{oz:.3f}) theta={theta:.1f} deg"
    )
if args.variant == "rs":
    # With torque off the motors do not hold, so a hand-move shows up in the next read; the
    # stream is on (see above) so the rows carry real status too.
    input("\nstaleness check: torque is off; move any joint by hand a little, then press Enter ... ")
    show(bus)

if not args.move:
    bus.release()
    print("done (torque untouched)")
    sys.exit(0)

# Keep the bus held across the move: the arm acquires the same port/baud/vendor and so
# reuses this cached instance instead of closing and reopening the CAN channel.
input("\n--move: the arm will now enable torque and move joint 6. Enter to continue, Ctrl-C to abort ... ")

from viam.proto.app.robot import ComponentConfig  # noqa: E402
from viam.proto.component.arm import JointPositions  # noqa: E402
from viam.utils import dict_to_struct  # noqa: E402

from src.rebot_b601.arm import B601Arm  # noqa: E402
from src.rebot_b601.damiao import CollisionError, MotorFault, OverTemperatureError  # noqa: E402

attrs = {"variant": args.variant, "port": port, "speed_deg_s": 20}
arm = B601Arm.new(ComponentConfig(name="smoke", attributes=dict_to_struct(attrs)), {})


async def nudge():
    print("\nhealth before:", arm._health_report())
    start = list((await arm.get_joint_positions()).values)
    target = list(start)
    target[5] += 5.0
    print(f"moving joint6 {start[5]:.2f} -> {target[5]:.2f} deg and back ...")
    await arm.move_to_joint_positions(JointPositions(values=target))
    await arm.move_to_joint_positions(JointPositions(values=start))
    await arm.stop()
    print("after:", [round(v, 2) for v in (await arm.get_joint_positions()).values])
    print("health after (holding against gravity):", arm._health_report())


try:
    asyncio.run(nudge())
except (MotorFault, CollisionError, OverTemperatureError) as e:
    print(f"\nREFUSED: {e}")
    sys.exit(1)
finally:
    asyncio.run(arm.close())
    bus.release()
print("done")
