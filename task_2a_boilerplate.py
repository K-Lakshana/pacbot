"""PacBot Task 2A - solver (boilerplate with the TODOs filled in).

The task: the robot spawns in a random cell facing a random open direction.
Drive through the pellet's cell, then leave the maze through the exit on
the -y (south) side of cell (14, 14). Score: 20 for solving + 10 for the pellet
- 5 per wall collision.

What you get over MQTT:
  pacbot/poses    once at startup (retained): the robot's spawn pose
                  -> Solver.on_poses()
  pacbot/pellets  once at startup (retained): the pellet's pose
                  -> Solver.on_pellets()
  pacbot/sensors  every physics step (10 ms): ToF distances, gyro,
                  accelerometer, wheel encoder ticks -> Solver.step()
  pacbot/result   once, when the run ends: {"solved", "collisions"}
What you send:
  pacbot/wheel_vel  wheel speeds in rad/s - the return value of Solver.step()

Run:
    ./task_2a_launch                 # terminal 1
    python3 task_2a_solution.py      # terminal 2
"""
import heapq
import json
import math

import paho.mqtt.client as mqtt

# ------------------------------------------------------------------ MQTT
MQTT_HOST = "localhost"
MQTT_PORT = 1883
TOPIC_SENSORS = "pacbot/sensors"      # sim -> us, every physics step
TOPIC_POSES = "pacbot/poses"          # sim -> us, once, retained: robot spawn pose
TOPIC_PELLETS = "pacbot/pellets"      # sim -> us, once, retained: pellet pose
TOPIC_RESULT = "pacbot/result"        # sim -> us, once, when the run ends
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"  # us -> sim

# ------------------------------------------------------------------ maze
N = 15            # cells per side
PITCH = 0.22      # m, cell centre to cell centre
WALL_HALF = 0.005  # m, half the wall thickness
# Cells are (row, col), as the maze drawing and pacbot/poses use: row 0 at
# the top, column 0 at the left. As in Task 2B, cell (row, col) is centred
# at world (col * PITCH, (N - 1 - row) * PITCH) - see cell_centre().
EXIT = (N - 1, N - 1, 3)  # the only opening in the border: -y side of (14, 14)

# Directions: 0 = +x, 1 = +y, 2 = -x, 3 = -y (the numbering pacbot/poses uses).
# STEP[d] is the (d_row, d_col) of one cell in direction d: +x is the next
# column, +y the row above.
STEP = ((0, 1), (-1, 0), (0, -1), (1, 0))

# Copy of kLayout in maze_2a.cpp - the maze is the same on every launch.
# Drawn in the Task 2B format (see task2b/maze_2b.py): line 2 * row + 1 is
# a row of cells and the four chars from 4 * col are one cell. '---' is a
# wall on the +y / -y side of a cell (the lines above / below it), '|' a
# wall on its -x / +x side (the chars left / right of it). ' E ' in the
# bottom line marks the exit, on the -y side of (14, 14).
LAYOUT = (
    "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+",
    "|                           |       |               |       |",
    "+---+---+   +---+   +   +   +   +   +---+   +---+   +---+   +",
    "|           |           |   |   |       |       |           |",
    "+   +   +   +   +---+---+---+   +---+   +   +   +---+---+   +",
    "|   |           |               |   |   |   |   |   |       |",
    "+   +---+---+---+   +---+   +---+   +   +---+   +   +   +---+",
    "|   |               |               |       |   |           |",
    "+   +   +---+---+---+   +   +---+   +---+   +   +---+   +   +",
    "|   |       |           |       |   |       |       |       |",
    "+   +---+   +   +   +---+---+   +   +   +---+---+   +---+   +",
    "|           |   |       |       |   |       |       |   |   |",
    "+   +   +   +   +---+   +   +---+   +---+   +   +---+   +   +",
    "|   |       |       |   |   |   |       |       |       |   |",
    "+   +   +   +---+---+   +   +   +   +   +   +---+   +   +   +",
    "|   |   |           |       |       |           |   |       |",
    "+   +   +---+---+   +   +---+---+   +---+---+   +   +---+---+",
    "|   |   |           |           |       |       |           |",
    "+   +   +   +---+---+---+---+   +   +   +   +---+---+---+   +",
    "|   |   |               |       |   |   |           |       |",
    "+   +---+---+   +---+   +   +---+   +   +---+   +   +   +---+",
    "|   |       |   |           |           |       |   |   |   |",
    "+   +   +   +   +   +---+   +---+---+   +   +---+   +   +   +",
    "|   |   |           |   |               |   |               |",
    "+   +   +---+---+---+   +---+---+---+   +   +---+   +---+   +",
    "|   |       |                   |       |       |       |   |",
    "+   +   +   +   +   +   +---+   +   +---+---+   +---+   +   +",
    "|   |   |       |   |           |   |       |   |       |   |",
    "+   +   +---+---+   +   +   +   +   +---+   +   +---+---+   +",
    "|               |           |               |               |",
    "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+ E +",
)


def _wall_table():
    """{(row, col, d): True if side d of cell (row, col) is walled}."""
    walls = {}
    for row in range(N):
        for col in range(N):
            r, c = 2 * row + 1, 4 * col
            walls[row, col, 0] = LAYOUT[r][c + 4] == "|"                # +x: right
            walls[row, col, 1] = LAYOUT[r - 1][c + 1:c + 4] == "---"    # +y: above
            walls[row, col, 2] = LAYOUT[r][c] == "|"                    # -x: left
            walls[row, col, 3] = LAYOUT[r + 1][c + 1:c + 4] == "---"    # -y: below
    return walls


# WALLS[row, col, d] is True if there is a wall on side d of cell (row, col).
WALLS = _wall_table()


def cell_centre(cell):
    """World (x, y) of the centre of cell (row, col)."""
    return cell[1] * PITCH, (N - 1 - cell[0]) * PITCH


def inside(row, col):
    return 0 <= row < N and 0 <= col < N


def neighbours(cell):
    """Open neighbouring cells of `cell`, as (direction, cell) pairs.
    The exit leads outside the maze and is not included."""
    row, col = cell
    out = []
    for d in range(4):
        n = (row + STEP[d][0], col + STEP[d][1])
        if not WALLS[row, col, d] and inside(*n):
            out.append((d, n))
    return out


# ----------------------------------------------------------------- robot
WHEEL_R = 0.017           # m, wheel radius
TRACK = 0.078             # m, wheel centre to wheel centre
TICKS_PER_REV = 360       # encoder ticks per wheel revolution
M_PER_TICK = 2 * math.pi * WHEEL_R / TICKS_PER_REV
WHEEL_MAX = 30.0          # rad/s, the simulator's clamp
AXLE_BACK = 0.033         # m, the wheel axle is this far behind the chassis centre
NO_HIT = 2.0              # m, what a ToF reads when nothing is within its 2 m range

# ToF sensors: name -> (x, y) from the chassis centre, and the direction
# the ray points relative to the robot's heading. Note the names:
# "sl" / "sr" face FORWARD; "fl" / "fr" face the SIDES.
SENSORS = (
    ("sl", (-0.005, 0.034), 0.0),
    ("sr", (-0.005, -0.034), 0.0),
    ("fl", (0.036, 0.005), math.pi / 2),
    ("fr", (0.036, -0.005), -math.pi / 2),
)


def wrap(a):
    """Fold an angle into (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def wheels(v, omega):
    """(left, right) wheel speeds in rad/s for a forward speed v (m/s) and
    a turn rate omega (rad/s, positive = counter-clockwise / left)."""
    left = (v - omega * TRACK / 2) / WHEEL_R
    right = (v + omega * TRACK / 2) / WHEEL_R
    return clamp(left, -WHEEL_MAX, WHEEL_MAX), clamp(right, -WHEEL_MAX, WHEEL_MAX)


# ---------------------------------------------------- extra geometry helpers
FACE = PITCH / 2 - WALL_HALF          # 0.105 m: cell centre -> face of a wall
_SENS = {name: (pos, ang) for name, pos, ang in SENSORS}
FRONT_X = AXLE_BACK + _SENS["sl"][0][0]   # sl/sr: how far ahead of the AXLE they sit
FRONT_Y = _SENS["sl"][0][1]               # sl/sr: sideways offset from the centre line
SIDE_Y = abs(_SENS["fl"][0][1])           # fl/fr: sideways offset from the centre line


def nearest_cell(x, y):
    """(row, col) of the cell whose centre is closest to world point (x, y)."""
    return round((N - 1) - y / PITCH), round(x / PITCH)


def first_wall(row, col, d):
    """Walking from (row, col) in direction d, the first cell that has a wall
    on its side d (None if we walk out of the maze, e.g. through the exit)."""
    while inside(row, col):
        if WALLS[row, col, d]:
            return row, col
        row += STEP[d][0]
        col += STEP[d][1]
    return None


class PID:
    """Textbook PID. `rate` (d error / dt) can be supplied from a sensor
    (the gyro) instead of differentiating the error numerically."""

    def __init__(self, kp, ki=0.0, kd=0.0, i_max=1.0, i_zone=float("inf")):
        self.kp, self.ki, self.kd, self.i_max = kp, ki, kd, i_max
        self.i_zone = i_zone    # only accumulate the integral when |error| is this small
        self.reset()

    def reset(self):
        self.i = 0.0
        self.prev = None

    def update(self, err, dt, rate=None):
        if abs(err) < self.i_zone:                      # anti-windup
            self.i = clamp(self.i + err * dt, -self.i_max, self.i_max)
        else:
            self.i = 0.0
        if rate is None:
            rate = 0.0 if self.prev is None else (err - self.prev) / dt
        self.prev = err
        return self.kp * err + self.ki * self.i + self.kd * rate


# ============================================================== planning

TURN_COST = 2.0     # one 90-degree turn counts as this many cells of driving


def successors(cell, allow_exit):
    """Moves out of `cell`: the open neighbours, plus (if allowed) the exit
    from (14, 14). Leaving the maze is only allowed on the final leg."""
    out = list(neighbours(cell))
    if allow_exit and cell == EXIT[:2]:
        out.append((EXIT[2], (cell[0] + STEP[EXIT[2]][0], cell[1] + STEP[EXIT[2]][1])))
    return out


def search(start, heading, goal, allow_exit=False):
    """Cheapest path of cells from `start` (facing `heading`) to `goal`.
    Cost = 1 per cell moved + TURN_COST per 90 degrees turned (Dijkstra over
    (cell, heading)). With TURN_COST = 0 this is exactly BFS.
    Returns (cells, final_heading)."""
    best = {(start, heading): 0.0}
    parent = {}
    tie = 0
    heap = [(0.0, tie, start, heading)]
    while heap:
        cost, _, cell, h = heapq.heappop(heap)
        if cost > best.get((cell, h), float("inf")):
            continue
        if cell == goal:
            path, state = [cell], (cell, h)
            while state in parent:
                state = parent[state]
                path.append(state[0])
            return path[::-1], h
        for d, nxt in successors(cell, allow_exit):
            quarter_turns = min((d - h) % 4, (h - d) % 4)      # 0, 1 or 2
            new_cost = cost + 1 + TURN_COST * quarter_turns
            if new_cost < best.get((nxt, d), float("inf")):
                best[(nxt, d)] = new_cost
                parent[(nxt, d)] = (cell, h)
                tie += 1
                heapq.heappush(heap, (new_cost, tie, nxt, d))
    raise ValueError(f"no path from {start} to {goal}")


def cells_to_actions(cells, heading):
    """Cell list -> [("turn", d), ("run", n), ...] with straight moves merged."""
    actions = []
    for a, b in zip(cells, cells[1:]):
        d = STEP.index((b[0] - a[0], b[1] - a[1]))
        if d != heading:
            actions.append(("turn", d))
            heading = d
        if actions and actions[-1][0] == "run":
            actions[-1] = ("run", actions[-1][1] + 1)
        else:
            actions.append(("run", 1))
    return actions


def plan_route(start, heading, pellet):
    """Plan the route: from `start` facing `heading`, through `pellet`, out
    of the exit.

    start, pellet: (row, col) cells. heading: 0-3.
    Returns a list of actions for the solver to drive, for example
    [("run", 3), ("turn", 1), ("run", 2), ...] - "run n" drives n cells
    straight on, "turn d" pivots to face direction d. The last run should
    carry the robot out through the exit.
    """
    outside = (EXIT[0] + STEP[EXIT[2]][0], EXIT[1] + STEP[EXIT[2]][1])   # (15, 14)
    leg1, h = search(start, heading, pellet)      # spawn  -> pellet
    leg2, _ = search(pellet, h, outside, allow_exit=True)   # pellet -> through the exit
    return cells_to_actions(leg1 + leg2[1:], heading)


# ================================================================ solver

# --- driving parameters (tune these) ---
V_MAX = 1         # m/s, top straight-line speed
V_MIN = 0.05          # m/s, creep speed so we never stall short of the target
ACC = 1.0             # m/s^2, speed ramp-up
KV = 3.0              # 1/s, speed = KV * distance_left (slows down on approach)
ARRIVE_TOL = 0.004    # m, a run is finished when this close to the target
K_LAT = 5.0           # rad per m of sideways error -> heading offset
LAT_LIM = 0.30        # rad, largest heading offset used to get back on line
W_TURN = 4.0          # rad/s, largest turn rate when pivoting
W_RUN = 1.5           # rad/s, largest turn rate when driving
TURN_TOL = 0.02       # rad, pivot is done within this heading error ...
TURN_RATE_TOL = 0.10  # rad/s ... and slower than this ...
SETTLE_FRAMES = 5     # ... for this many frames in a row

# --- ToF correction parameters ---
ALIGN_MAX = 0.10      # rad, only trust ToF when this well aligned with the corridor
FRONT_RANGE = 0.45    # m, use the forward sensors only when the wall is this close
FRONT_AGREE = 0.012   # m, sl and sr must agree (both see the same flat wall)
SIDE_RANGE = 0.20     # m, use the side sensors only below this reading
SIDE_ZONE = 0.07      # m, side sensors only used this close to a cell centre
GATE = 0.05           # m, ignore corrections bigger than this (bad reading)
BLEND = 0.20          # fraction of the measured error applied per frame


class Solver:
    """One sensor frame in, wheel speeds out."""

    def __init__(self):
        self.t = 0.0            # s of sim time since the first sensor frame
        self.spawn = None       # (dir, cell) of the robot at spawn
        self.pellet = None      # (row, col) cell of the pellet
        self.actions = None     # the plan, once made
        self.prev_ticks = None  # (left, right) encoder ticks last frame
        self.done = False
        # Pose estimate: position of the AXLE (the point the wheels pivot
        # about) in world metres, and heading in radians (0 = +x, ccw +).
        self.x = self.y = self.theta = 0.0
        # Which action we are executing, and the logical cell / heading.
        self.i = 0
        self.started = False
        self.cell = None
        self.heading = 0
        self.target_cell = None
        self.target_xy = (0.0, 0.0)
        self.v_cmd = 0.0
        self.settled = 0
        self.finished_logged = False
        # Controllers.
        self.turn_pid = PID(kp=8.0, ki=0.5, kd=0.3, i_max=0.2, i_zone=0.1)
        self.head_pid = PID(kp=5.0, ki=0.0, kd=0.3)

    def log(self, text):
        print(f"[{self.t:7.2f}s] {text}", flush=True)

    def on_poses(self, msg):
        """pacbot/poses:
        {"bot": [row, col], "dir": 0-3}
        The chassis centre spawns on cell_centre(cell) (the axle is
        AXLE_BACK behind it)."""
        bot, d = msg.get("bot"), msg.get("dir")
        if isinstance(bot, list) and d is not None and self.actions is None:
            self.spawn = (d, tuple(bot))
            self.log(f"spawn {self.spawn[1]} facing {self.spawn[0]}")

    def on_pellets(self, msg):
        """pacbot/pellets:
        {"pellet": [row, col]} - the pellet's cell; it sits on the cell centre."""
        pellet = msg.get("pellet")
        if pellet and self.actions is None:
            self.pellet = tuple(pellet)
            self.log(f"pellet {self.pellet}")

    # ------------------------------------------------------ pose tracking
    def init_pose(self):
        d, cell = self.spawn
        cx, cy = cell_centre(cell)
        self.theta = d * math.pi / 2
        # chassis centre is on the cell centre; the axle is AXLE_BACK behind it
        self.x = cx - AXLE_BACK * STEP[d][1]
        self.y = cy + AXLE_BACK * STEP[d][0]
        self.cell, self.heading = cell, d

    def update_pose(self, d_left, d_right, turn_rate, dt):
        """Dead reckoning: gyro for heading, encoders for distance."""
        mid = self.theta + 0.5 * turn_rate * dt          # heading half-way through the frame
        self.theta += turn_rate * dt
        dist = 0.5 * (d_left + d_right)                  # axle travel this frame
        self.x += dist * math.cos(mid)
        self.y += dist * math.sin(mid)

    def tof_correct(self, s, d):
        """Nudge the pose estimate using the walls the ToF sensors can see.
        d = the direction (0-3) of the straight run we are on."""
        dth = d * math.pi / 2
        a = wrap(self.theta - dth)                       # heading error vs the corridor
        if abs(a) > ALIGN_MAX:
            return
        dx, dy = STEP[d][1], -STEP[d][0]                 # unit vector along the run
        nx, ny = -dy, dx                                 # unit vector to the left

        # (1) Wall ahead: fixes the position along the run, and the heading.
        sl, sr = s["sl"], s["sr"]
        if max(sl, sr) < FRONT_RANGE and abs(sl - sr) < FRONT_AGREE:
            wall = first_wall(*nearest_cell(self.x, self.y), d)
            if wall is not None:
                wx, wy = cell_centre(wall)
                s_face = wx * dx + wy * dy + FACE        # where that wall's face is
                s_meas = s_face - 0.5 * (sl + sr) - FRONT_X
                delta = s_meas - (self.x * dx + self.y * dy)
                if abs(delta) < GATE:
                    self.x += BLEND * delta * dx
                    self.y += BLEND * delta * dy
                alpha = math.atan2(sl - sr, 2 * FRONT_Y)  # robot's angle to the wall normal
                self.theta += BLEND * wrap(dth + alpha - self.theta)

        # (2) Walls beside us: fixes the sideways position.
        sx = self.x + (AXLE_BACK + _SENS["fl"][0][0]) * dx
        sy = self.y + (AXLE_BACK + _SENS["fl"][0][0]) * dy
        row, col = nearest_cell(sx, sy)
        if not inside(row, col):
            return
        cx, cy = cell_centre((row, col))
        if abs((sx - cx) * dx + (sy - cy) * dy) > SIDE_ZONE:
            return                                       # too close to a cell boundary
        est = []                                         # chassis offset to the left of the centre line
        if WALLS[row, col, (d + 1) % 4] and s["fl"] < SIDE_RANGE:
            est.append(FACE - SIDE_Y - s["fl"])
        if WALLS[row, col, (d + 3) % 4] and s["fr"] < SIDE_RANGE:
            est.append(s["fr"] - (FACE - SIDE_Y))
        if est:
            e = sum(est) / len(est)
            n_meas = (cx * nx + cy * ny) + e - AXLE_BACK * math.sin(a)   # axle's sideways coordinate
            delta = n_meas - (self.x * nx + self.y * ny)
            if abs(delta) < GATE:
                self.x += BLEND * delta * nx
                self.y += BLEND * delta * ny

    # ---------------------------------------------------------- actions
    def start_action(self):
        kind, arg = self.actions[self.i]
        self.turn_pid.reset()
        self.head_pid.reset()
        self.settled = 0
        self.v_cmd = 0.0
        if kind == "turn":
            self.heading = arg
        else:
            r = self.cell[0] + arg * STEP[self.heading][0]
            c = self.cell[1] + arg * STEP[self.heading][1]
            self.target_cell = (r, c)
            self.target_xy = cell_centre(self.target_cell)
        self.started = True
        self.log(f"action {self.i + 1}/{len(self.actions)}: {kind} {arg}")

    def finish_action(self):
        if self.actions[self.i][0] == "run":
            self.cell = self.target_cell
        self.i += 1
        self.started = False

    def do_turn(self, turn_rate, dt):
        """Pivot on the spot (wheels in opposite directions) to self.heading."""
        err = wrap(self.heading * math.pi / 2 - self.theta)
        w = clamp(self.turn_pid.update(err, dt, -turn_rate), -W_TURN, W_TURN)
        if abs(err) < TURN_TOL and abs(turn_rate) < TURN_RATE_TOL:
            self.settled += 1
        else:
            self.settled = 0
        if self.settled >= SETTLE_FRAMES:
            self.finish_action()
            return 0.0, 0.0
        return wheels(0.0, w)

    def do_run(self, s, turn_rate, dt):
        """Drive straight along self.heading to the centre of the target cell."""
        d = self.heading
        self.tof_correct(s, d)
        dx, dy = STEP[d][1], -STEP[d][0]
        tx, ty = self.target_xy
        remaining = (tx - self.x) * dx + (ty - self.y) * dy      # distance still to go
        lateral = (self.x - tx) * (-dy) + (self.y - ty) * dx     # + = left of the line
        if remaining < ARRIVE_TOL:
            self.finish_action()
            return 0.0, 0.0

        # forward speed: ramp up, cruise, slow down on approach
        v_goal = clamp(KV * remaining, V_MIN, V_MAX)
        self.v_cmd = min(v_goal, self.v_cmd + ACC * dt)

        # heading: aim parallel to the corridor, tilted back toward the line
        desired = d * math.pi / 2 - clamp(K_LAT * lateral, -LAT_LIM, LAT_LIM)
        err = wrap(desired - self.theta)
        w = clamp(self.head_pid.update(err, dt, -turn_rate), -W_RUN, W_RUN)
        v = self.v_cmd * clamp(1.0 - abs(err) / 0.4, 0.2, 1.0)   # slower while badly aimed
        return wheels(v, w)

    def step(self, s):
        """pacbot/sensors, one physics step:
        {"fl": m, "fr": m, "sl": m, "sr": m,      ToF distances (see SENSORS)
         "gyro": [x, y, z],                        rad/s; z is the turn rate
         "accel": [x, y, z],                       m/s^2
         "ticks_l": int, "ticks_r": int,           running encoder counts
         "dt": 0.01}                               s
        Returns (left, right) wheel speeds in rad/s."""
        self.t += s["dt"]
        ticks = (s["ticks_l"], s["ticks_r"])
        if self.prev_ticks is None:
            self.prev_ticks = ticks
        # Distance each wheel rolled since the last frame.
        d_left = (ticks[0] - self.prev_ticks[0]) * M_PER_TICK
        d_right = (ticks[1] - self.prev_ticks[1]) * M_PER_TICK
        self.prev_ticks = ticks
        turn_rate = s["gyro"][2]

        if self.done or self.spawn is None or self.pellet is None:
            return 0.0, 0.0
        if self.actions is None:
            self.actions = plan_route(self.spawn[1], self.spawn[0], self.pellet)
            self.log(f"plan: {self.actions}")
            self.init_pose()

        # Track the robot's pose from d_left, d_right and turn_rate.
        self.update_pose(d_left, d_right, turn_rate, s["dt"])

        # Drive self.actions one by one.
        if self.i >= len(self.actions):
            if not self.finished_logged:
                self.log("route complete")
                self.finished_logged = True
            return 0.0, 0.0
        if not self.started:
            self.start_action()
        kind = self.actions[self.i][0]
        if kind == "turn":
            return self.do_turn(turn_rate, s["dt"])
        return self.do_run(s, turn_rate, s["dt"])


# ================================================================== main

def _client():
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:  # paho-mqtt < 2
        return mqtt.Client()


def main():
    solver = Solver()
    client = _client()

    def send(cli, left, right):
        # The simulator parses exactly this shape - keep json's defaults.
        cli.publish(TOPIC_WHEEL_VEL, json.dumps({"left": float(left), "right": float(right)}))

    def on_message(cli, _userdata, msg):
        if msg.topic == TOPIC_SENSORS:
            send(cli, *solver.step(json.loads(msg.payload)))
        elif msg.topic == TOPIC_POSES:
            if msg.payload:  # an empty retained message just clears the topic
                solver.on_poses(json.loads(msg.payload))
        elif msg.topic == TOPIC_PELLETS:
            if msg.payload:
                solver.on_pellets(json.loads(msg.payload))
        elif msg.topic == TOPIC_RESULT:
            solver.log(f"result: {msg.payload.decode()}")
            solver.done = True
            send(cli, 0.0, 0.0)
            cli.disconnect()

    client.on_message = on_message
    client.connect(MQTT_HOST, MQTT_PORT)
    for topic in (TOPIC_POSES, TOPIC_PELLETS, TOPIC_RESULT, TOPIC_SENSORS):
        client.subscribe(topic)
    print("connected, waiting for the simulator", flush=True)
    client.loop_forever()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
