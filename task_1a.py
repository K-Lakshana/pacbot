#!/usr/bin/env python3
"""
"""

import json
import time

import paho.mqtt.client as mqtt

MAZE_ROWS = 13
MAZE_COLS = 13
MQTT_BROKER = "localhost" 
MQTT_PORT = 1883
POSE_TOPIC = "robot/pose"

# wall bit per side, OR'd together
WALL_N, WALL_E, WALL_S, WALL_W = 0x1, 0x2, 0x4, 0x8

WALLS = [
    [12, 6, 12, 6, 13, 4, 0, 4, 5, 6, 12, 5, 6],
    [10, 11, 10, 10, 12, 3, 8, 2, 12, 1, 1, 6, 10],
    [8, 5, 3, 8, 1, 6, 9, 2, 9, 6, 13, 2, 10],
    [10, 12, 4, 3, 12, 1, 4, 0, 6, 9, 4, 2, 10],
    [10, 10, 8, 5, 3, 13, 2, 10, 9, 6, 10, 11, 10],
    [8, 3, 9, 4, 5, 6, 8, 1, 6, 10, 9, 5, 2],
    [10, 12, 4, 1, 6, 10, 9, 6, 10, 8, 5, 5, 2],
    [8, 1, 2, 12, 3, 10, 12, 3, 9, 2, 12, 6, 10],
    [9, 6, 10, 10, 12, 1, 2, 12, 5, 1, 0, 1, 3],
    [14, 8, 1, 1, 3, 12, 1, 3, 12, 4, 2, 12, 6],
    [8, 0, 4, 7, 12, 3, 12, 6, 10, 9, 1, 2, 10],
    [10, 10, 9, 6, 10, 12, 2, 8, 1, 7, 12, 0, 2],
    [9, 1, 5, 1, 1, 3, 8, 1, 5, 5, 3, 9, 3],
]
# Maze map -- the same data as WALLS above, drawn. row 0 = SOUTH (bottom),
# col 0 = WEST (left). The gaps in the top and bottom edges are EXIT_CELLS.
#
#            0  1  2  3  4  5  6  7  8  9 10 11 12   <- col
#          +--+--+--+--+--+--+  +--+--+--+--+--+--+
#   row 12 |                 |              |     |
#          +  +  +--+  +  +  +  +  +--+--+  +  +  +
#   row 11 |  |  |     |  |     |        |        |
#          +  +  +  +--+  +--+  +  +  +--+--+  +  +
#   row 10 |           |     |     |  |        |  |
#          +  +  +--+--+--+  +--+--+  +  +  +  +  +
#   row  9 |  |           |        |        |     |
#          +--+  +  +  +  +--+  +  +--+--+  +--+--+
#   row  8 |     |  |  |        |                 |
#          +  +--+  +  +--+  +  +--+--+  +  +  +  +
#   row  7 |        |     |  |     |     |     |  |
#          +  +  +  +--+  +  +--+  +  +  +--+--+  +
#   row  6 |  |           |  |     |  |           |
#          +  +--+--+  +--+  +  +--+  +  +--+--+  +
#   row  5 |     |           |        |  |        |
#          +  +  +  +--+--+--+  +  +--+  +  +--+  +
#   row  4 |  |  |        |     |  |     |  |  |  |
#          +  +  +  +--+  +--+  +  +  +--+  +  +  +
#   row  3 |  |        |              |        |  |
#          +  +--+--+  +--+  +--+  +--+  +--+  +  +
#   row  2 |        |        |     |     |     |  |
#          +  +--+  +  +  +--+  +  +  +--+--+  +  +
#   row  1 |  |  |  |  |     |     |           |  |
#          +  +  +  +  +--+  +  +  +--+  +  +--+  +
#   row  0 |     |     |                 |        |
#          +--+--+--+--+--+--+  +--+--+--+--+--+--+
#            0  1  2  3  4  5  6  7  8  9 10 11 12   <- col

# the 2 known exits: (row, col, facing)
EXIT_CELLS = [
    (0, 6, 'south'),
    (MAZE_ROWS - 1, 6, 'north'),
]

BOT_CMD_TOPIC = "bot/cmd"         
PELLETS_TOPIC = "pellets/pose"    
CMD_VEL_TOPIC = "robot/cmd_vel"  

# yaw -> dr, dc, wall bit. 0=EAST, 90=NORTH, 180=WEST, 270=SOUTH
HEADING_DELTA = {
    0.0:   (0, 1, WALL_E),
    90.0:  (1, 0, WALL_N),
    180.0: (0, -1, WALL_W),
    270.0: (-1, 0, WALL_S),
}


# ============================================================================
# YOUR ALGORITHM GOES HERE. Everything above and below is plumbing.
# ============================================================================

#ed returns the euclidean distance between two cells start and end
def ed(start, end):
    return (((end[0]-start[0])*(end[0]-start[0])+(end[1]-start[1])*(end[1]-start[1]))**(1/2))



#moves=[[(row, col)], direction_wrt_parent_cell]]
#moves contain all the possible moves(out of 4) from the passed cell

def get_reachable_moves(tuplerc):
    r=tuplerc[0]
    c=tuplerc[1]
    cell_value = WALLS[r][c]
    
    # A move is valid if the bitwise AND (&) with the wall constant is 0
    moves = []
    
    if not (cell_value & WALL_N):
        moves.append([(r + 1, c), 90])
        
    if not (cell_value & WALL_E):
        moves.append([(r, c + 1), 0])
        
    if not (cell_value & WALL_S):
        moves.append([(r - 1, c), 270])
        
    if not (cell_value & WALL_W):
        moves.append([(r, c - 1), 180])
        
    return moves



'''

once the closed_list is returned by path_plan function,
the path has to be traced back from it

closed_list = [ [ (row1, col1), (parent_row1, parent_col1), f1, g1, h1, direction_wrt_parent_cell ] ]

the last cell added to closed_list is the destination cell,

it's the first cell to be added to the path,

its parent cell is the next cell, the parent cell of this parent cell is the next cell and so on.

This process is repeated till we reach starting cell, for which parent cell is None,

Finally the path list is reversed and returned

'''

def trace_back(closed_list):
    path = []
    current_cell = closed_list[-1]
    
    while current_cell is not None:
        point = current_cell[0]
        parent_point = current_cell[1]
        
        path.append([point, current_cell[5]])
        
        if parent_point is None:
            break
           
        next_cell = None
        for cell in reversed(closed_list):
            if cell[0] == parent_point:
                next_cell = cell
                break
                
        current_cell = next_cell


    path.reverse()
    return path



'''
we have used A* search algorithm
first the start cell is added to the open_list
f(n) = g(n) + h(n)
where g(n) is the distance covered in travelling from the start cell to the current_cell
h(n) is the euclidean distance between current_cell to end cell

until the dest cell is reached, the open_list is looped over,

the cell with the lowest f value in the open_list is chosen as the currrent_cell,

current_cell is removed from the open_list and added to the closed_list,

the cells which can be reached from the current_cell is found out,

from the moves list, we see each possible next move,

if the move is already on the closed_list, it's ignored,

else if the move is alraedy in the open_list,
it's replaced with the current parameters if current g point is lower than the previous g point,

else if the move is not in the open_list, we calculate f, g, h and add the cell to the open_list.
'''

def pathplan(start, dest):
    f=ed(start, dest)
    g=0
    h=f
    parent_point=None
    open_list=[(start, parent_point, f, g, h, None)]
    closed_list=[]
    current_cell=(start, parent_point, f, g, h, None)
    
    
    end=False
    while (end==False):
        f_lowest=open_list[0]
        for i in open_list:
            if (i[2]<f_lowest[2]):
                f_lowest = i  
        current_cell=f_lowest
        closed_list.append(current_cell)
        open_list.remove(f_lowest)
        
        if current_cell[0]==dest:
            end=True
            return trace_back(closed_list)
            break
            
        moves = get_reachable_moves(current_cell[0])
        
        for j in moves:
            for k in closed_list:
                if k[0]==j[0]:
                    break
            else:
                for index, k in enumerate(open_list):
                    g = current_cell[3]+1
                    if k[0]==j[0]:
                        if g<k[3]:
                            f = g + k[4]
                            parent = current_cell[0]
                            open_list[index]=(k[0], parent, f, g, k[4], j[1])
                        break
                else:
                    h = ed(j[0], dest)
                    g = current_cell[3]+1
                    f = g+h
                    parent = current_cell[0]
                    open_list.append((j[0], parent, f, g, h, j[1]))
  
        

'''
if we are at any of the exit cells and all the pellets_remaining is empty, 
we calculate the target_yaw

else if we just collected the last pellet, the dest is the closest exit
 
else we choose the pellet which is closest and find the path to it using path_plan function
the 2nd cell in the path_plan function should be the next cell

target_yaw is the orientation of next cell wrt the parent cell

the differnce between the target_yaw and pacbot_yaw is calculated to return the final command
'''
def choose_command(pacbot_cell, pacbot_yaw, pellets_remaining):

    """FRONT/LEFT/RIGHT/BACK to send now, or None. pacbot_cell=(row,col),
    pacbot_yaw one of HEADING_DELTA's keys, pellets_remaining=set of
    (row,col). Implement this -- see EXIT_CELLS above for the 2 exits."""
    
    pellets_remaining = list(pellets_remaining) # since set is not subscriptable
    
    for r, c, facing in EXIT_CELLS:
        if pacbot_cell == (r, c) and len(pellets_remaining)==0:

            if facing == 'north':
                target_yaw = 90
            elif facing == 'south':
                target_yaw = 270
            break
             
    else:
        if len(pellets_remaining)==1 and ed(pacbot_cell, pellets_remaining[0])==0 or len(pellets_remaining)==0:
            dest = (EXIT_CELLS[0][0], EXIT_CELLS[0][1])
            for exit in EXIT_CELLS:
                if ed((exit[0], exit[1]), pacbot_cell) < ed((dest[0], dest[1]), pacbot_cell):
                    dest = (exit[0], exit[1])
                     
        else:
            dest = pellets_remaining[0]
            for i in pellets_remaining:
                    dist = ed(i, pacbot_cell)
                    if dist == 0:
                        continue
                    elif ed(i, pacbot_cell) < ed(dest, pacbot_cell):
                        dest = i           
                                          
        path = pathplan(pacbot_cell, dest)
        target_yaw = path[1][1]

    yaw_diff = (target_yaw - pacbot_yaw) % 360
    
    if yaw_diff == 0:
        return "FRONT"
    elif yaw_diff == 90:
        return "LEFT"
    elif yaw_diff == 180:
        return "BACK"
    elif yaw_diff == 270:
        return "RIGHT"
        
    return None
    
# ============================================================================


def parse_pellets(payload):
    return {tuple(cell) for cell in json.loads(payload)}


def main():
    state = {
        "running": False,
        "pellets": set(),
        "cell": (MAZE_ROWS // 2, MAZE_COLS // 2),
        "yaw": 0.0,
        "in_flight": False,
        "got_pose": False,
        "got_pellets": False,
    }

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="Controller")

    def decide_and_send():
        if (not state["running"] or state["in_flight"]
                or not state["got_pose"] or not state["got_pellets"]):
            return
        print(f"[debug] pose={state['cell']} yaw={state['yaw']} pellets={state['pellets']}")
        cmd = choose_command(state["cell"], state["yaw"], set(state["pellets"]))
        if cmd is not None:
            state["in_flight"] = True
            client.publish(CMD_VEL_TOPIC, cmd)
            print(f"[controller] {state['cell']} yaw={state['yaw']} -> {cmd}, "
                  f"pellets_left={len(state['pellets'])}")

    def on_message(client, userdata, msg):
        try:
            if msg.topic == BOT_CMD_TOPIC:
                running = msg.payload.decode().startswith("1")
                was_running = state["running"]
                state["running"] = running
                if running and not was_running:
                    decide_and_send()   # kick off the reactive loop on Start
            elif msg.topic == PELLETS_TOPIC:
                state["pellets"] = parse_pellets(msg.payload.decode())
                state["got_pellets"] = True
                decide_and_send()
            elif msg.topic == POSE_TOPIC:
                data = json.loads(msg.payload.decode())
                state["cell"] = (int(data["col"]), int(data["row"]))   # wire is swapped
                state["got_pose"] = True
                state["yaw"] = float(data.get("yaw", 0.0))
                state["in_flight"] = False   # this pose is the ack for our last command
                decide_and_send()   # every pose/command-ack triggers the next step
        except Exception as e:
            print("[controller] mqtt parse error:", e)

    client.on_message = on_message
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.subscribe([(BOT_CMD_TOPIC, 0), (PELLETS_TOPIC, 0), (POSE_TOPIC, 0)])
    client.loop_start()

    print(f"[controller] ready; sending one '{CMD_VEL_TOPIC}' command at a time, "
          f"reacting to '{POSE_TOPIC}'/'{PELLETS_TOPIC}' feedback")

    try:
        while True:
            time.sleep(0.2)  
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()
