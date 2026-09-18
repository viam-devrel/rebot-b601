"""Hardware smoke test for the B601 (DM or RS).

Read-only by default: prints joint state (and, for DM, forward kinematics) and never
enables torque. With --move it enables torque, nudges joint 6 by +5 deg and back, and
stops. The move step is refused by the arm itself if any motor reports a fault.

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
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.rebot_b601 import spatial  # noqa: E402
from src.rebot_b601.arm import VARIANT_VENDOR  # noqa: E402
from src.rebot_b601.bus import SharedBus, detect_port  # noqa: E402
from src.rebot_b601.damiao import JointHealth  # noqa: E402

NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("port_pos", nargs="?", metavar="port", help="serial device (dm) or CAN channel (rs)")
ap.add_argument("--port", help="same as the positional port")
ap.add_argument("--variant", choices=("dm", "rs"), default="dm")
ap.add_argument("--move", action="store_true", help="enable torque and nudge joint 6 by 5 deg (MOVES THE ARM)")
args = ap.parse_args()

port = args.port or args.port_pos
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
        print(
            f"  {NAMES[i]} (0x{cid:02x}): pos={h.pos_deg:8.2f} deg  vel={h.vel_rad_s:6.3f} rad/s  "
            f"torq={h.torque_nm:6.3f} Nm  t_mos={h.t_mos_c:.0f}C  status={h.status}"
        )
        if i < 6:
            positions.append(h.pos_deg)
    return positions


print(f"connecting to {port} ({args.variant}, {vendor}) ...")
bus = SharedBus.acquire(port, vendor=vendor)
positions = show(bus)
if args.variant == "dm" and len(positions) == 6:
    x, y, z, ox, oy, oz, theta = spatial.end_position(positions)
    print(
        f"\nend effector (FK): x={x:.1f} y={y:.1f} z={z:.1f} mm  o=({ox:.3f},{oy:.3f},{oz:.3f}) theta={theta:.1f} deg"
    )
elif args.variant == "rs":
    # No status stream here (motors are unconfigured), so RS positions come from mechPos
    # parameter reads and report "position only (no status frame)" -- expected, and why
    # this check works with torque off.
    input("\nstaleness check: torque is off; move any joint by hand a little, then press Enter ... ")
    show(bus)
bus.release()

if not args.move:
    print("done (torque untouched)")
    sys.exit(0)

from viam.proto.app.robot import ComponentConfig  # noqa: E402
from viam.proto.component.arm import JointPositions  # noqa: E402
from viam.utils import dict_to_struct  # noqa: E402

from src.rebot_b601.arm import B601Arm  # noqa: E402

attrs = {"variant": args.variant, "port": port, "speed_deg_s": 20}
arm = B601Arm.new(ComponentConfig(name="smoke", attributes=dict_to_struct(attrs)), {})


async def nudge():
    print("\nhealth:", arm._health_report())
    start = list((await arm.get_joint_positions()).values)
    target = list(start)
    target[5] += 5.0
    print(f"moving joint6 {start[5]:.2f} -> {target[5]:.2f} deg and back ...")
    await arm.move_to_joint_positions(JointPositions(values=target))
    await arm.move_to_joint_positions(JointPositions(values=start))
    await arm.stop()
    print("after:", [round(v, 2) for v in (await arm.get_joint_positions()).values])
    await arm.close()


asyncio.run(nudge())
print("done")
