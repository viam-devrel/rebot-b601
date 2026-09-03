"""Read-only hardware smoke test: connects to the B601 and prints joint state.

Does NOT enable torque and does NOT move the arm.

Run: .venv/bin/python tests/smoke_hardware.py [port]
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.rebot_b601 import spatial
from src.rebot_b601.bus import SharedBus, detect_port

port = sys.argv[1] if len(sys.argv) > 1 else detect_port()
print(f"connecting to {port} @ 921600 ...")
bus = SharedBus.acquire(port)

names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]
states = bus.poll_feedback(list(range(1, 8)))

positions = []
for i, cid in enumerate(range(1, 8)):
    s = states[cid]
    if s is None:
        print(f"  {names[i]} (0x{cid:02x}): NO FEEDBACK")
    else:
        print(f"  {names[i]} (0x{cid:02x}): pos={math.degrees(s.pos):8.2f} deg  vel={s.vel:6.3f} rad/s  torq={s.torq:6.3f} Nm  t_mos={s.t_mos:.0f}C")
        if i < 6:
            positions.append(math.degrees(s.pos))

if len(positions) == 6:
    x, y, z, ox, oy, oz, theta = spatial.end_position(positions)
    print(f"\nend effector (FK): x={x:.1f} y={y:.1f} z={z:.1f} mm  o=({ox:.3f},{oy:.3f},{oz:.3f}) theta={theta:.1f} deg")

bus.release()
print("done (torque untouched)")
