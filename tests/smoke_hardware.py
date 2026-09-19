"""Hardware smoke test for the B601 (DM or RS).

Read-only by default: prints joint state and forward kinematics (both variants) and never
enables torque. With --move it nudges joint 6 by +5 deg and back, then stops; torque is
enabled only after you confirm at an explicit prompt, and the move is refused with the
arm's own message if any motor reports a fault, a collision or an over-temperature.

With --gravity-check the arm is built in MIT mode, told to hold exactly where it already is,
and then left alone: it prints, per joint, the torque the motors report while holding next to
the gravity torque the model predicts, so you can see whether the signs agree before turning
gravity compensation on. MIT is not optional here. The torque field is not a sensor reading,
it is inferred from the position loop's tracking error, and profile position drives that error
to zero, so every joint reports about 0 Nm however hard it is working. Stop viam-server first:
the CAN channel cannot be shared.

Run:
  .venv/bin/python tests/smoke_hardware.py                          # DM, port auto-detected
  .venv/bin/python tests/smoke_hardware.py --variant rs --port can0  # RS, read-only
  .venv/bin/python tests/smoke_hardware.py --variant rs --port can0 --move
  .venv/bin/python tests/smoke_hardware.py --variant rs --port can0 --gravity-check

macOS + PEAK PCAN-USB: motorbridge dlopens libPCBUSB.dylib by bare name, so run with
DYLD_LIBRARY_PATH=/usr/local/lib (run.sh exports it for the module itself).
"""

import argparse
import asyncio
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.rebot_b601 import spatial  # noqa: E402
from src.rebot_b601.bus import VARIANT_VENDOR, BusError, SharedBus, detect_port  # noqa: E402
from src.rebot_b601.damiao import JointHealth  # noqa: E402

NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]

# allow_abbrev=False: without it "--m"/"--mo"/"--mov" all mean --move and would move the arm.
ap = argparse.ArgumentParser(
    description=__doc__, allow_abbrev=False, formatter_class=argparse.RawDescriptionHelpFormatter
)
ap.add_argument("--port", help="serial device (dm, default: auto-detect) or CAN channel (rs, required)")
ap.add_argument("--variant", choices=("dm", "rs"), default="dm")
ap.add_argument("--move", action="store_true", help="enable torque and nudge joint 6 by 5 deg (MOVES THE ARM)")
ap.add_argument(
    "--gravity-check",
    action="store_true",
    help="holds without moving: enables MIT torque at the arm's current pose, then compares the "
    "holding torque each joint reports with the model's gravity torque. MIT is required, the "
    "torque figure is derived from tracking error and reads ~0 in profile position "
    "(stop viam-server first; the CAN channel cannot be shared)",
)
ap.add_argument(
    "--gripper",
    action="store_true",
    help="MOVES THE GRIPPER: jog motor 0x07 by a step you type and print its position, to find "
    "open_position_deg (stop viam-server first; the CAN channel cannot be shared)",
)
args = ap.parse_args()

port = args.port
if not port:
    if args.variant == "rs":
        ap.error("--port is required for --variant rs (e.g. can0 or PCAN_USBBUS1)")
    port = detect_port()
vendor = VARIANT_VENDOR[args.variant]


def show(bus) -> list:
    # positions_only: every show() here runs with torque off, and a stopped RobStride motor
    # stops streaming while motorbridge keeps serving the frame it cached at the stop. Without
    # this the staleness check below compares a frozen frame with itself and always "passes".
    states = bus.poll_feedback(list(range(1, 8)), positions_only=True)
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


def jog_gripper():
    """Type a signed step in motor degrees, Enter to repeat, 'q' to stop. Nothing is clamped:
    this is how open_position_deg is discovered, so the jaws' own hard stop is the limit."""
    from motorbridge import Mode  # deferred like the other hardware imports below

    motor = bus.motor(7)
    motor.enable()
    motor.ensure_mode(Mode.POS_VEL if vendor == "robstride" else Mode.FORCE_POS)
    step = 10.0
    start = bus.poll_feedback([7])[7]
    if start is None:
        print("no feedback from gripper motor 0x07; check power and wiring")
        return
    target = math.degrees(start.pos)

    def deg(state):
        return math.degrees(state.pos) if state is not None else float("nan")

    print(f"gripper at {target:.2f} deg; positive/negative steps, 'q' to quit")
    # Each step prints the streamed status frame and the mechPos parameter read side by side,
    # with the time since the command went out, then mechPos again half a second later. Both
    # answer the question the single "actual" column could not: if 'param' tracks the target
    # while 'stream' trails it, the streamed read is stale; if both trail together, the motor
    # is genuinely slow -- and then 'later' has climbed past 'param' because it is still moving.
    print("  stream = streamed status frame, param = mechPos round trip, later = mechPos again")
    while True:
        raw = input(f"step [{step:+.1f}] > ").strip()
        if raw.lower() == "q":
            break
        if raw:
            try:
                step = float(raw)
            except ValueError:
                print("  not a number")
                continue
        target += step
        sent = time.monotonic()
        with bus.lock:
            if vendor == "robstride":
                motor.send_pos_vel(math.radians(target), math.radians(90.0))
            else:
                motor.send_force_pos(math.radians(target), math.radians(90.0), 0.07)
        time.sleep(0.5)
        streamed = bus.poll_feedback([7], positions_only=False)[7]
        t_stream = time.monotonic() - sent
        param = bus.poll_feedback([7], positions_only=True)[7]
        t_param = time.monotonic() - sent
        time.sleep(0.5)
        later = bus.poll_feedback([7], positions_only=True)[7]
        t_later = time.monotonic() - sent
        print(
            f"  target {target:8.2f}  stream {deg(streamed):8.2f} (+{t_stream:.2f}s)  "
            f"param {deg(param):8.2f} (+{t_param:.2f}s)  later {deg(later):8.2f} (+{t_later:.2f}s)"
        )
    print(f"\nrecord this as open_position_deg once the jaws are fully open: {target:.1f}")


print(f"connecting to {port} ({args.variant}, {vendor}) ...")
try:
    bus = SharedBus.acquire(port, vendor=vendor)
except BusError as e:
    print(f"cannot open {port}: {e}")
    sys.exit(1)
if vendor == "robstride":
    # Ask the motors to stream status (a comms setting, not torque) so that frames flow as
    # soon as torque comes on. It does nothing for the read-only rows below: a disabled
    # RobStride motor sends no frames at all, so those rows are position-only by necessity.
    for cid in range(1, 8):
        bus.motor(cid).robstride_set_active_report(True)
    time.sleep(0.3)
positions = show(bus)
if len(positions) == 6:
    x, y, z, ox, oy, oz, theta = spatial.MODELS[args.variant].end_position(positions)
    print(
        f"\ntool mount (FK, {args.variant}): x={x:.1f} y={y:.1f} z={z:.1f} mm  "
        f"o=({ox:.3f},{oy:.3f},{oz:.3f}) theta={theta:.1f} deg"
    )
if args.variant == "rs":
    # With torque off the motors do not hold, so a hand-move shows up in the next read; the
    # stream is on (see above) so the rows carry real status too.
    input("\nstaleness check: torque is off; move any joint by hand a little, then press Enter ... ")
    print("  (positions below must differ from the ones above; identical rows mean a stale read)")
    show(bus)

if args.gripper:
    input("\nthe gripper motor will be enabled and jogged. Enter to continue, Ctrl-C to abort ... ")
    jog_gripper()
    bus.release()
    sys.exit(0)

if not (args.move or args.gravity_check):
    bus.release()
    print("done (torque untouched)")
    sys.exit(0)

# Keep the bus held across the move: the arm acquires the same port/baud/vendor and so
# reuses this cached instance instead of closing and reopening the CAN channel.
input(
    "\nthe arm will now enable torque"
    + (", and move joint 6" if args.move else "")
    + ". Enter to continue, Ctrl-C to abort ... "
)

from viam.proto.app.robot import ComponentConfig  # noqa: E402
from viam.proto.component.arm import JointPositions  # noqa: E402
from viam.utils import dict_to_struct  # noqa: E402

from src.rebot_b601.arm import ARM_CAN_IDS, B601Arm  # noqa: E402
from src.rebot_b601.damiao import CollisionError, MotorFault, OverTemperatureError  # noqa: E402

attrs = {"variant": args.variant, "port": port, "speed_deg_s": 20}
if args.gravity_check:
    # MIT, and torque left off at build: gravity_check() enables it only once it has read
    # where the arm is resting, so the motors never go live without a setpoint to hold.
    attrs["control_mode"] = "mit"
    attrs["enable_on_start"] = False
arm = B601Arm.new(ComponentConfig(name="smoke", attributes=dict_to_struct(attrs)), {})


def gravity_check():
    # Measured 2026-09-19 in profile position: every joint read under 0.17 Nm while the model
    # wanted up to 5.9, and the elbow sat 20 C hotter than its neighbours while reporting
    # 0.16 Nm. The torque field is inferred from tracking error, which that mode drives to
    # zero, so the column was meaningless. MIT holds with a standing error set by the load.
    resting = arm._read_positions_deg()
    arm._configure_motors()  # enable + MIT; also flips the flag that lets full frames through
    arm._send_targets_deg(resting)  # hold where it already is: no lurch, and no limp window
    time.sleep(1.0)  # let each joint settle into the sag the estimate is read from
    print("\nhealth:", arm._health_report())
    positions = arm._read_positions_deg()
    states = arm._read_states()
    model = spatial.MODELS[args.variant]
    g = model.gravity_torques([math.radians(d) for d in positions], arm.gravity_vector, arm.payload_kg)
    print(f"\ngravity check at {[round(p, 1) for p in positions]} deg (MIT, holding)")
    print(f"  {'joint':7s} {'measured':>9s} {'model':>9s} {'expect':>9s}  sign")
    ok = True
    for i, cid in enumerate(ARM_CAN_IDS):
        s = states.get(cid)
        expect = -g[i]  # the motor torque that cancels gravity
        if s is None or getattr(s, "position_only", False):
            # No status frame, so no torque measurement: not a mismatch, just nothing to judge.
            measured, flag = float("nan"), "no data"
        elif abs(expect) < 0.5 or s.torq * expect > 0:
            measured, flag = s.torq, "ok"
        else:
            measured, flag = s.torq, "MISMATCH"
        ok &= flag != "MISMATCH"
        print(f"  joint{i + 1:<2d} {measured:9.3f} {g[i]:9.3f} {expect:9.3f}  {flag}")
    print("  (expect = -model; a joint with |expect| < 0.5 Nm is too lightly loaded to judge)")
    print(
        "gravity check:",
        "signs agree on every loaded joint" if ok else "SIGN MISMATCH, do not enable gravity compensation",
    )


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
    if args.gravity_check:
        gravity_check()
    if args.move:
        asyncio.run(nudge())
except (MotorFault, CollisionError, OverTemperatureError) as e:
    print(f"\nREFUSED: {e}")
    sys.exit(1)
finally:
    asyncio.run(arm.close())
    bus.release()
print("done")
