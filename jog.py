import curses, json, pathlib, time
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

CAL = pathlib.Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json"
MARGIN = 3.0
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

def load_limits():
    cal = json.loads(CAL.read_text())
    limits = {}
    for name, c in cal.items():
        if name == "gripper":
            limits[name] = (2.0, 98.0)
        else:
            half = (c["range_max"] - c["range_min"]) / 2 * 360 / 4095
            limits[name] = (-half + MARGIN, half - MARGIN)
    limits["wrist_roll"] = (-150.0, 150.0)
    return limits

LIMITS = load_limits()

def clamp(j, v):
    lo, hi = LIMITS[j]
    return max(lo, min(hi, v))

KEYS = {
    "q": ("shoulder_pan", 1),  "a": ("shoulder_pan", -1),
    "w": ("shoulder_lift", 1), "s": ("shoulder_lift", -1),
    "e": ("elbow_flex", 1),    "d": ("elbow_flex", -1),
    "r": ("wrist_flex", 1),    "f": ("wrist_flex", -1),
    "t": ("wrist_roll", 1),    "g": ("wrist_roll", -1),
    "y": ("gripper", 1),       "h": ("gripper", -1),
}

SPEED = 20.0   # deg/s (gripper: %/s); change live with [ and ]
ACCEL = 80.0   # deg/s^2 ramp up/down
HOLD = 0.15    # seconds a key press keeps its joint moving
FPS = 50

def main(stdscr):
    cfg = SO101FollowerConfig(port="/dev/ttyACM0", id="my_follower",
                              max_relative_target=None)
    robot = SO101Follower(cfg)
    robot.connect()
    obs = robot.get_observation()
    pos = {j: clamp(j, obs[f"{j}.pos"]) for j in JOINTS}
    vel = {j: 0.0 for j in JOINTS}
    direction = {j: 0 for j in JOINTS}
    deadline = {j: 0.0 for j in JOINTS}
    speed = SPEED
    bus_ms = 0.0
    stdscr.nodelay(True)
    prev = next_t = last_ui = time.perf_counter()
    running = True
    try:
        while running:
            now = time.perf_counter()
            while True:
                ch = stdscr.getch()
                if ch == -1:
                    break
                if ch >= 256:
                    continue
                c = chr(ch).lower()
                if c == "x":
                    running = False
                elif c == "[":
                    speed = max(5.0, speed - 5.0)
                elif c == "]":
                    speed = min(60.0, speed + 5.0)
                elif c in KEYS:
                    j, s = KEYS[c]
                    direction[j] = s
                    deadline[j] = now + HOLD

            dt = now - prev
            prev = now
            for j in JOINTS:
                target_v = direction[j] * speed if now < deadline[j] else 0.0
                max_dv = ACCEL * dt
                vel[j] += max(-max_dv, min(max_dv, target_v - vel[j]))
                new = pos[j] + vel[j] * dt
                lo, hi = LIMITS[j]
                if new <= lo or new >= hi:
                    new = clamp(j, new)
                    vel[j] = 0.0
                    deadline[j] = 0.0
                pos[j] = new

            t0 = time.perf_counter()
            robot.send_action({f"{j}.pos": pos[j] for j in JOINTS})
            bus_ms = (time.perf_counter() - t0) * 1000

            if now - last_ui > 0.1:
                last_ui = now
                stdscr.erase()
                stdscr.addstr(0, 0, "hold: q/a pan  w/s lift  e/d elbow  r/f wrist  t/g roll  y/h gripper")
                stdscr.addstr(1, 0, f"[ ] speed ({speed:.0f})   x quit   bus {bus_ms:.1f} ms")
                for i, j in enumerate(JOINTS):
                    lo, hi = LIMITS[j]
                    stdscr.addstr(3 + i, 0, f"{j:14s} {pos[j]:8.1f}  v={vel[j]:6.1f}  [{lo:.0f}, {hi:.0f}]")
                stdscr.refresh()

            next_t += 1 / FPS
            wait = next_t - time.perf_counter()
            if wait > 0:
                time.sleep(wait)
            else:
                next_t = time.perf_counter()
    finally:
        robot.disconnect()

curses.wrapper(main)