"""PacBot Task 2A - simple solver.

What the robot has to do: start in a random cell, drive through the cell with the
pellet, then leave the maze through the exit at the bottom of cell (14, 14).

How this program works (in plain words):
  1. PLAN   - find the shortest route (a list of cells) from the start, through the
              pellet, to the exit. Then turn it into easy actions:
              ("run", 3) = drive 3 cells straight,  ("turn", 1) = turn to face north.
  2. DRIVE  - do one action at a time. Between two actions the robot stops
              completely, so it never turns while it is still moving.
  3. TRACK  - keep a guess of where we are using the wheel encoders (distance)
              and the gyro (angle), and fix that guess with the wall sensors.

Run:
    ./task_2a_launch                 # terminal 1
    python3 task_2a_simple.py        # terminal 2
"""
import json
import math

import paho.mqtt.client as mqtt

# ------------------------------------------------------------------ MQTT
MQTT_HOST = "localhost"
MQTT_PORT = 1883
TOPIC_SENSORS = "pacbot/sensors"      # simulator -> us, every 10 ms
TOPIC_POSES = "pacbot/poses"          # simulator -> us, once: where the robot starts
TOPIC_PELLETS = "pacbot/pellets"      # simulator -> us, once: where the pellet is
TOPIC_RESULT = "pacbot/result"        # simulator -> us, once, when the run ends
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"  # us -> simulator: wheel speeds

# ------------------------------------------------------------------ maze
N = 15              # the maze is 15 x 15 cells
PITCH = 0.22        # metres from one cell centre to the next
WALL_HALF = 0.005   # half the thickness of a wall (metres)

# A cell is written (row, col). Row 0 is the top row, column 0 is the left column.
EXIT = (N - 1, N - 1, 3)   # the only hole in the outer wall: bottom side of cell (14, 14)

# Directions: 0 = east (+x), 1 = north (+y), 2 = west (-x), 3 = south (-y).
# STEP[d] = (change in row, change in column) when moving one cell in direction d.
STEP = ((0, 1), (-1, 0), (0, -1), (1, 0))

# The maze drawing. '---' is a wall above/below a cell, '|' is a wall left/right of it.
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

# WALLS[row, col, d] is True when cell (row, col) has a wall on its side d.
WALLS = {}
for row in range(N):
    for col in range(N):
        line = 2 * row + 1      # the text line that holds this cell
        pos = 4 * col           # the text column where this cell starts
        WALLS[row, col, 0] = LAYOUT[line][pos + 4] == "|"                # east
        WALLS[row, col, 1] = LAYOUT[line - 1][pos + 1:pos + 4] == "---"  # north
        WALLS[row, col, 2] = LAYOUT[line][pos] == "|"                    # west
        WALLS[row, col, 3] = LAYOUT[line + 1][pos + 1:pos + 4] == "---"  # south


def cell_centre(cell):
    """The world position (x, y) of the middle of a cell."""
    return cell[1] * PITCH, (N - 1 - cell[0]) * PITCH


def nearest_cell(x, y):
    """The cell (row, col) whose middle is closest to the world point (x, y)."""
    return round((N - 1) - y / PITCH), round(x / PITCH)


def inside(row, col):
    return 0 <= row < N and 0 <= col < N


# ----------------------------------------------------------------- robot
WHEEL_R = 0.017           # wheel radius (m)
TRACK = 0.078             # distance between the two wheels (m)
TICKS_PER_REV = 360       # encoder ticks for one full wheel turn
M_PER_TICK = 2 * math.pi * WHEEL_R / TICKS_PER_REV   # metres rolled per tick
WHEEL_MAX = 30.0          # the simulator cannot spin a wheel faster than this (rad/s)
AXLE_BACK = 0.033         # the wheels' axle is this far BEHIND the middle of the robot
NO_HIT = 2.0              # what a distance sensor reads when it sees nothing

# Distance sensors: (name, (x, y) from the robot's middle, direction of the beam).
# "sl" and "sr" look FORWARD, "fl" and "fr" look to the SIDES (left / right).
SENSORS = (
    ("sl", (-0.005, 0.034), 0.0),
    ("sr", (-0.005, -0.034), 0.0),
    ("fl", (0.036, 0.005), math.pi / 2),
    ("fr", (0.036, -0.005), -math.pi / 2),
)

WALL_FACE = PITCH / 2 - WALL_HALF   # 0.105 m: from a cell's middle to the face of its wall
FRONT_X = 0.028      # the forward sensors are this far ahead of the axle (m)
FRONT_Y = 0.034      # ... and this far to the left / right of the centre line
SIDE_X = 0.069       # the side sensors are this far ahead of the axle (m)
SIDE_Y = 0.005       # ... and this far to the side of the centre line


def wrap(angle):
    """Turn any angle into the range -pi .. +pi."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(x, low, high):
    """Keep x between low and high."""
    return max(low, min(high, x))


def wheels(v, omega):
    """Wheel speeds (left, right) in rad/s for a forward speed v (m/s) and a
    turning speed omega (rad/s, positive = turn left)."""
    left = (v - omega * TRACK / 2) / WHEEL_R
    right = (v + omega * TRACK / 2) / WHEEL_R
    # If one wheel would go too fast, slow BOTH down by the same amount,
    # so the robot still drives the curve we asked for.
    biggest = max(abs(left), abs(right))
    if biggest > WHEEL_MAX:
        left = left * WHEEL_MAX / biggest
        right = right * WHEEL_MAX / biggest
    return left, right


# ============================================================== planning

# Here we are using djikstra, which means each turn has a cost equal to the cost of going straight on n cells
TURN_COST = 2.0


def find_route(start, heading, pellet):
    """Find the cheapest list of cells: start -> pellet -> exit.

    It is like a normal breadth-first search, but a move costs 1 for the cell
    plus TURN_COST for every 90-degree turn it needs. We remember three things
    about every place we have been: the cell, the way we face there, and whether
    we already passed the pellet.
    """
    outside = (EXIT[0] + STEP[EXIT[2]][0], EXIT[1] + STEP[EXIT[2]][1])   # the cell (15, 14)

    first = (start, heading, start == pellet)
    best_cost = {first: 0.0}       # cheapest cost found so far for each place
    came_from = {first: None}      # where we came from (to rebuild the route)
    todo = [(0.0, 0, first)]       # places still to look at: (cost, number, place)
    number = 0

    while len(todo) > 0:
        todo.sort()                # the cheapest place goes first
        cost, _, place = todo.pop(0)
        if cost > best_cost[place]:
            continue               # we already found a cheaper way here
        cell, facing, got_pellet = place

        if cell == outside:
            if got_pellet:
                # Finished! Walk backwards to collect the route.
                route = []
                while place is not None:
                    route.append(place[0])
                    place = came_from[place]
                route.reverse()
                return route
            continue               # left the maze without the pellet: useless

        # Find the cells we can drive to from here.
        moves = []
        for d in range(4):
            if not WALLS[cell[0], cell[1], d]:
                nxt = (cell[0] + STEP[d][0], cell[1] + STEP[d][1])
                if inside(nxt[0], nxt[1]):
                    moves.append((d, nxt))
        if cell == (EXIT[0], EXIT[1]):
            moves.append((EXIT[2], outside))          # the hole in the outer wall

        for d, nxt in moves:
            quarter_turns = min((d - facing) % 4, (facing - d) % 4)    # 0, 1 or 2
            new_cost = cost + 1 + TURN_COST * quarter_turns
            new_place = (nxt, d, got_pellet or nxt == pellet)
            if new_place not in best_cost or new_cost < best_cost[new_place]:
                best_cost[new_place] = new_cost
                came_from[new_place] = place
                number += 1
                todo.append((new_cost, number, new_place))
    raise ValueError("no route found")


def plan_route(start, heading, pellet):
    """Turn the route into actions like [("run", 3), ("turn", 1), ("run", 2)].
    ("run", n) = drive n cells straight, ("turn", d) = turn on the spot to face d."""
    cells = find_route(start, heading, pellet)

    actions = []
    for i in range(len(cells) - 1):
        row_now, col_now = cells[i]
        row_next, col_next = cells[i + 1]

        # Which way is this one-cell move?
        if col_next > col_now:
            new_dir = 0         # east
        elif row_next < row_now:
            new_dir = 1         # north (row 0 is the top)
        elif col_next < col_now:
            new_dir = 2         # west
        else:
            new_dir = 3         # south

        # Not facing that way yet? Turn first.
        if new_dir != heading:
            actions.append(("turn", new_dir))
            heading = new_dir

        # Drive one cell. If we were already driving, just make that run longer.
        if len(actions) > 0 and actions[-1][0] == "run":
            actions[-1] = ("run", actions[-1][1] + 1)
        else:
            actions.append(("run", 1))
    return actions


# ================================================================ solver

# --- speeds and gains (the numbers you can tune) ---
V_MAX = 0.65         # fastest straight-line speed (m/s). The wheels cannot do more than 0.51
V_MIN = 0.03         # slowest creep (m/s), so we never stall just before the target
ACC = 1.0            # how fast the speed command may grow (m/s per second): a smooth start
MOTOR_LAG = 0.3      # the motors need about 0.3 s to reach the speed we ask for
KV = 4.0             # driving speed = KV * (distance left - MOTOR_LAG * speed): brakes early
KP_RUN = 2.5         # how hard we steer back onto the straight line while driving
KD_RUN = 0.8        # damping from the gyro so the steering does not wobble
W_RUN_MAX = 0.8      # biggest turning speed allowed while driving (rad/s)
K_SIDE = 1.2         # how much we aim back to the middle of the corridor (rad per metre)
SIDE_AIM_MAX = 0.15  # biggest extra aim angle for that (rad)
KP_TURN = 3.5        # how hard we turn on the spot
KD_TURN = 1.05       # damping from the gyro so we do not overshoot the turn
W_TURN_MAX = 2.5     # fastest turn on the spot (rad/s)
W_TURN_MIN = 0.12    # slowest turn (rad/s), so we never stall just short
ARRIVE = 0.006       # a run is finished when we are this close to its end (m)
TURN_TOL = 0.02      # a turn is finished when the angle error is this small (rad) ...
TURN_RATE_TOL = 0.1  # ... and we are almost not spinning any more (rad/s)
STOP_SPEED = 0.02    # "stopped" means slower than this (m/s)
SETTLE_FRAMES = 5    # we must stay stopped for this many frames before the next action
PIVOT_BACKOFF = 0.03 # before a turn at a wall, stop this far BEFORE the cell middle (m): the nose
                     # swings wider than the corridor when it pivots, so it needs some spare room

# --- using the wall sensors to correct our position guess ---
ALIGN_MAX = 0.10     # only trust the sensors when we point this straight (rad)
FRONT_RANGE = 0.45   # use the front sensors only when the wall ahead is this close (m)
FRONT_AGREE = 0.012  # both front sensors must agree within this (m)
SIDE_RANGE = 0.20    # use a side sensor only when it reads less than this (m)
SIDE_ZONE = 0.07     # use the side sensors only this close to a cell middle (m)
GATE = 0.05          # side sensors: ignore a correction bigger than this (m)
FRONT_GATE = 0.10    # front sensors: a wall ahead is trusted up to this big a correction (m)
FRONT_BLEND = 0.30   # ... and we apply this much of it per frame
BLEND = 0.20         # how much of the correction we apply each frame (0 to 1)

# --- distance scale ---
# The robot really moves only about 82 % of the distance the encoders report.
# We start with that guess and keep improving it while we drive.
ODO_SCALE_START = 0.82
SCALE_MIN = 0.78       # the learned scale may only move inside this narrow band,
SCALE_MAX = 0.86       # so one bad reading can never throw the position guess off
CAL_RANGE = 1.8        # learn only while the front wall is closer than this (m)
CAL_JUMP = 0.03        # a sudden jump bigger than this means another wall: start again (m)
CAL_MIN_TRAVEL = 0.08  # drive at least this far before trusting what we learned (m)


class Solver:
    """Gets one sensor frame, returns the wheel speeds."""

    def __init__(self):
        self.t = 0.0             # time since the start (s)
        self.spawn = None        # (direction, cell) where the robot started
        self.pellet = None       # the pellet's cell
        self.actions = None      # the list of actions to do
        self.prev_ticks = None   # encoder ticks of the last frame
        self.done = False        # True once the simulator says the run is over

        self.x = 0.0             # where the AXLE is (world metres)
        self.y = 0.0
        self.theta = 0.0         # the way we point (0 = east, pi/2 = north)
        self.step_no = 0         # which action we are doing
        self.cell = None         # the cell we are in (updated after every run)
        self.facing = 0          # the direction (0-3) we are facing
        self.speed = 0.0         # real speed from the encoders (m/s)
        self.v_cmd = 0.0         # the speed we are asking for (m/s)
        self.holding = False     # True while we wait standing still between actions
        self.still = 0           # how many frames in a row we have been still
        self.shown = -1          # the last action we printed a line for
        self.finished_logged = False

        # for learning the distance scale
        self.odo_scale = ODO_SCALE_START
        self.cal_tof = 0.0       # distance the front sensor says we moved (finished pieces)
        self.cal_enc = 0.0       # distance the encoders say we moved (finished pieces)
        self.seg_start = None    # front reading at the start of the current piece
        self.seg_last = None     # latest front reading in the current piece
        self.seg_enc = 0.0       # encoder distance in the current piece

    def log(self, text):
        print(f"[{self.t:7.2f}s] {text}", flush=True)

    def on_poses(self, msg):
        """The simulator tells us where the robot starts: {"bot": [row, col], "dir": 0-3}."""
        bot = msg.get("bot")
        d = msg.get("dir")
        if isinstance(bot, list) and d is not None and self.actions is None:
            self.spawn = (d, tuple(bot))
            self.log(f"spawn {self.spawn[1]} facing {self.spawn[0]}")

    def on_pellets(self, msg):
        """The simulator tells us where the pellet is: {"pellet": [row, col]}."""
        pellet = msg.get("pellet")
        if pellet and self.actions is None:
            self.pellet = tuple(pellet)
            self.log(f"pellet {self.pellet}")

    # ---------------------------------------------------------------
    def learn_scale(self, s, raw):
        """Learn how far the robot REALLY moves compared with what the encoders say.
        When we drive straight towards a wall, the front sensor reading gets smaller
        by exactly the distance we really moved. So:
            scale = (how much the reading shrank) / (distance the encoders reported)"""
        sl = s["sl"]
        sr = s["sr"]
        front = (sl + sr) / 2

        # Is this a good moment to learn? (driving straight, pointing straight,
        # a flat wall ahead that both front sensors agree on)
        good = (not self.holding
                and self.step_no < len(self.actions)
                and self.actions[self.step_no][0] == "run"
                and abs(wrap(self.theta - self.facing * math.pi / 2)) < 0.05
                and max(sl, sr) < CAL_RANGE
                and abs(sl - sr) < FRONT_AGREE
                and raw > 0.0)
        if good and self.seg_last is not None and abs(self.seg_last - front) > CAL_JUMP:
            good = False         # the reading jumped: a different wall came into view

        if good:
            if self.seg_last is None:        # start a new piece
                self.seg_start = front
                self.seg_enc = 0.0
            self.seg_enc += raw
            self.seg_last = front
            enc_total = self.cal_enc + self.seg_enc
            if enc_total > CAL_MIN_TRAVEL:
                tof_total = self.cal_tof + (self.seg_start - front)
                self.odo_scale = clamp(tof_total / enc_total, SCALE_MIN, SCALE_MAX)
        elif self.seg_last is not None:      # the piece ended: remember what it taught us
            self.cal_tof += self.seg_start - self.seg_last
            self.cal_enc += self.seg_enc
            self.seg_last = None

    def next_action(self):
        """Go to the next action, but first stand completely still (see step())."""
        self.step_no += 1
        self.holding = True
        self.still = 0
        self.v_cmd = 0.0

    def fix_position(self, s, d):
        """Use the walls we can see to correct our position guess.
        d = the direction (0-3) of the straight run we are on."""
        run_angle = d * math.pi / 2
        angle_error = wrap(self.theta - run_angle)
        if abs(angle_error) > ALIGN_MAX:
            return                                   # not pointing straight: do not trust it

        ax = STEP[d][1]                              # unit vector along the run
        ay = -STEP[d][0]
        lx = -ay                                     # unit vector pointing to the left
        ly = ax

        # (1) A wall in front of us tells us how far along the run we are,
        #     and which way we point.
        sl = s["sl"]
        sr = s["sr"]
        if max(sl, sr) < FRONT_RANGE and abs(sl - sr) < FRONT_AGREE:
            # find the first cell ahead that has a wall on the side we are driving towards
            row, col = nearest_cell(self.x, self.y)
            wall_cell = None
            while inside(row, col):
                if WALLS[row, col, d]:
                    wall_cell = (row, col)
                    break
                row += STEP[d][0]
                col += STEP[d][1]
            if wall_cell is not None:
                wx, wy = cell_centre(wall_cell)
                wall_pos = wx * ax + wy * ay + WALL_FACE          # where that wall is, along the run
                axle_pos = wall_pos - (sl + sr) / 2 - FRONT_X     # where the axle must be
                delta = axle_pos - (self.x * ax + self.y * ay)
                if abs(delta) < FRONT_GATE:
                    self.x += FRONT_BLEND * delta * ax
                    self.y += FRONT_BLEND * delta * ay
                # the difference of the two readings tells us the angle to the wall
                wall_angle = math.atan2(sl - sr, 2 * FRONT_Y)
                self.theta += BLEND * wrap(run_angle + wall_angle - self.theta)

        # (2) Walls beside us tell us how far left / right of the middle we are.
        sx = self.x + SIDE_X * ax                    # where the side sensors are
        sy = self.y + SIDE_X * ay
        row, col = nearest_cell(sx, sy)
        if not inside(row, col):
            return
        cx, cy = cell_centre((row, col))
        if abs((sx - cx) * ax + (sy - cy) * ay) > SIDE_ZONE:
            return                                   # too close to the edge of the cell
        readings = []                                # how far the robot is left of the middle
        if WALLS[row, col, (d + 1) % 4] and s["fl"] < SIDE_RANGE:      # wall on the left
            readings.append(WALL_FACE - SIDE_Y - s["fl"])
        if WALLS[row, col, (d + 3) % 4] and s["fr"] < SIDE_RANGE:      # wall on the right
            readings.append(s["fr"] - (WALL_FACE - SIDE_Y))
        if len(readings) > 0:
            left_of_middle = sum(readings) / len(readings)
            left_pos = (cx * lx + cy * ly) + left_of_middle - AXLE_BACK * math.sin(angle_error)
            delta = left_pos - (self.x * lx + self.y * ly)
            if abs(delta) < GATE:
                self.x += BLEND * delta * lx
                self.y += BLEND * delta * ly

    def turn_on_the_spot(self, direction, turn_rate):
        """Turn without moving forward until we face `direction` (0-3)."""
        error = wrap(direction * math.pi / 2 - self.theta)      # how far we still have to turn

        # Finished? Pointing the right way and almost not spinning.
        if abs(error) < TURN_TOL and abs(turn_rate) < TURN_RATE_TOL:
            self.facing = direction
            self.next_action()
            return 0.0, 0.0

        # Big error -> turn fast. The gyro term slows us down as the spin builds up.
        omega = KP_TURN * error - KD_TURN * turn_rate
        # Almost stopped but not there yet: creep so we do not stall.
        if abs(error) > TURN_TOL and abs(turn_rate) < 0.5 * W_TURN_MIN and abs(omega) < W_TURN_MIN:
            omega = math.copysign(W_TURN_MIN, error)
        omega = clamp(omega, -W_TURN_MAX, W_TURN_MAX)
        return wheels(0.0, omega)

    def drive_straight(self, cells, s, turn_rate):
        """Drive `cells` cells straight ahead."""
        # The cell where this run ends, and its middle.
        end_row = self.cell[0] + cells * STEP[self.facing][0]
        end_col = self.cell[1] + cells * STEP[self.facing][1]
        end_x, end_y = cell_centre((end_row, end_col))

        ax = STEP[self.facing][1]       # unit vector along our direction of travel
        ay = -STEP[self.facing][0]

        # If a turn comes next and there is a wall right ahead of the end cell, stop a
        # little short of the cell middle. When the robot pivots, its front corners swing
        # out almost as far as the wall, so this gives them some room.
        turn_next = self.step_no + 1 < len(self.actions) and self.actions[self.step_no + 1][0] == "turn"
        if turn_next and inside(end_row, end_col) and WALLS[end_row, end_col, self.facing]:
            end_x -= PIVOT_BACKOFF * ax
            end_y -= PIVOT_BACKOFF * ay

        self.fix_position(s, self.facing)

        # How far to the middle of the end cell? How far left (+) or right (-) of the line?
        remaining = (end_x - self.x) * ax + (end_y - self.y) * ay
        sideways = (self.x - end_x) * (-ay) + (self.y - end_y) * ax

        # Arrived? Remember the new cell and go on to the next action.
        if remaining < ARRIVE:
            self.cell = (end_row, end_col)
            self.next_action()
            return 0.0, 0.0

        # Speed: the closer to the end, the slower. The motors are slow to react,
        # so we start braking early (by the distance we still roll: speed * MOTOR_LAG).
        goal = clamp(KV * (remaining - MOTOR_LAG * self.speed), 0.0, V_MAX)
        if self.speed < V_MIN:
            goal = max(goal, V_MIN)                 # nearly stopped: creep on
        self.v_cmd = min(goal, self.v_cmd + ACC * 0.01)       # speed up gently

        # Steering: point along the corridor, tilted a little back to the middle line.
        aim = self.facing * math.pi / 2 - clamp(K_SIDE * sideways, -SIDE_AIM_MAX, SIDE_AIM_MAX)
        error = wrap(aim - self.theta)
        omega = clamp(KP_RUN * error - KD_RUN * turn_rate, -W_RUN_MAX, W_RUN_MAX)

        # Slow down while pointing the wrong way, so the nose does not hit a wall.
        speed = self.v_cmd * clamp(1.0 - abs(error) / 0.4, 0.2, 1.0)
        return wheels(speed, omega)

    def step(self, s):
        """Called for every sensor frame (every 10 ms). `s` holds:
        fl, fr, sl, sr (distance sensors), gyro [x, y, z], ticks_l, ticks_r, dt.
        Returns (left, right) wheel speeds in rad/s."""
        self.t += s["dt"]

        # How far did each wheel roll since the last frame?
        ticks = (s["ticks_l"], s["ticks_r"])
        if self.prev_ticks is None:
            self.prev_ticks = ticks
        d_left = (ticks[0] - self.prev_ticks[0]) * M_PER_TICK
        d_right = (ticks[1] - self.prev_ticks[1]) * M_PER_TICK
        self.prev_ticks = ticks
        turn_rate = s["gyro"][2]

        if self.done or self.spawn is None or self.pellet is None:
            return 0.0, 0.0

        # First frame: make the plan and set our starting position.
        if self.actions is None:
            self.actions = plan_route(self.spawn[1], self.spawn[0], self.pellet)
            # The robot starts with its MIDDLE on the cell middle, but it turns around
            # its AXLE, which is 3.3 cm further back. So if the first action is a turn,
            # we first drive a tiny bit ("run 0") to put the axle on the cell middle.
            if self.actions[0][0] == "turn":
                self.actions.insert(0, ("run", 0))
            self.log(f"plan: {self.actions}")
            start_dir, start_cell = self.spawn
            cx, cy = cell_centre(start_cell)
            self.theta = start_dir * math.pi / 2
            self.x = cx - AXLE_BACK * STEP[start_dir][1]
            self.y = cy + AXLE_BACK * STEP[start_dir][0]
            self.cell = start_cell
            self.facing = start_dir

        # 1) Where are we? The gyro gives the angle, the encoders give the distance.
        self.theta += turn_rate * s["dt"]
        raw = (d_left + d_right) / 2                  # distance the encoders report
        self.learn_scale(s, raw)
        distance = raw * self.odo_scale               # distance the robot really moved
        self.speed = distance / s["dt"]
        self.x += distance * math.cos(self.theta)
        self.y += distance * math.sin(self.theta)

        # 2) Everything done? Stand still.
        if self.step_no >= len(self.actions):
            if not self.finished_logged:
                self.log("route complete")
                self.finished_logged = True
            return 0.0, 0.0

        # 3) Between two actions we wait until the robot has really stopped.
        #    This is what makes it align properly before every turn.
        if self.holding:
            if abs(self.speed) < STOP_SPEED and abs(turn_rate) < TURN_RATE_TOL:
                self.still += 1
            else:
                self.still = 0
            if self.still >= SETTLE_FRAMES:
                self.holding = False
            return 0.0, 0.0

        # 4) Do the current action.
        kind, value = self.actions[self.step_no]
        if self.shown != self.step_no:                # first frame of a new action: print it
            self.shown = self.step_no
            self.log(f"action {self.step_no + 1}/{len(self.actions)}: {kind} {value}"
                     f"   position=({self.x:.3f}, {self.y:.3f})"
                     f"   angle={math.degrees(self.theta):.1f} deg   distance scale={self.odo_scale:.3f}")
        if kind == "turn":
            return self.turn_on_the_spot(value, turn_rate)
        return self.drive_straight(value, s, turn_rate)


# ================================================================== main

def make_client():
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:       # older paho-mqtt
        return mqtt.Client()


def main():
    solver = Solver()
    client = make_client()

    def send(cli, left, right):
        cli.publish(TOPIC_WHEEL_VEL, json.dumps({"left": float(left), "right": float(right)}))

    def on_message(cli, _userdata, msg):
        if msg.topic == TOPIC_SENSORS:
            left, right = solver.step(json.loads(msg.payload))
            send(cli, left, right)
        elif msg.topic == TOPIC_POSES:
            if msg.payload:
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
