"""Boilerplate for PB Task 1B.

Subscribes to the simulator's sensor topic, logs each reading, and publishes
a wheel velocity command back. Fill in your control logic where marked.

Run (three terminals):
    mosquitto
    ./task_1b_launch
    python3 task_1b_boilerplate.py
"""
import json

import paho.mqtt.client as mqtt

MQTT_HOST = "localhost"
MQTT_PORT = 1883
TOPIC_SENSORS = "pacbot/sensors"      # simulator publishes, this file subscribes
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"  # this file publishes, simulator subscribes


def _mqtt_client():
    # paho-mqtt >= 2.0 requires picking a callback API version explicitly.
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        return mqtt.Client()
        
       
prev_error = 0.0
integral = 0.0
state = "FOLLOW"
turn_angle = 0.0
turn_direction = None
base_speed = 30.0
turn_speed = 10.0
kp= 35.0
ki = 0.02
kd = 0.5
maze_completed = False

def on_message(client, userdata, msg):
    data = json.loads(msg.payload.decode())

    fl = data["fl"]            # Front-left ToF distance readings
    fr = data["fr"]            # Front-right ToF distance readings
    sl = data["sl"]            # Side-left ToF distance readings 
    sr = data["sr"]            # Side-right ToF distance readings 
    yaw_rate = data["gyro"][2]  # rad/s about z
    dt = data["dt"]            # s, simulator timestep

    print(f"fl={fl:.3f} fr={fr:.3f} sl={sl:.3f} sr={sr:.3f} "
          f"yaw_rate={yaw_rate:+.3f} dt={dt:.4f}")
    

    # TODO: compute wheel velocities (rad/s) from the readings above.
    
    #used global so that changes to the variables can be done
    global maze_completed, prev_error, integral, state, turn_angle, turn_direction
    
    if maze_completed==False:
    
        if (sl >= 2 and sr >= 2 and (fr>=2 or fl>=2)):
            print("No walls in range on any side - clear of the maze")
            maze_completed=True
            
        left_vel = 0.0
        right_vel = 0.0

       
        if state == "FOLLOW":
            if (sl < 0.11) or (sr < 0.1): #wall in front
                turn_angle = 0.0
            
                if fl > fr:
                    turn_direction = "LEFT"
                else:
                    turn_direction = "RIGHT"

                state = "TURN"

            else:
                #PID control to centre the bot while moving in a straight path
                error = fl - fr
                integral += error * dt
                derivative = (error - prev_error) / dt

                correction = (kp * error + ki * integral + kd * derivative)
                left_vel = base_speed - correction
                right_vel = base_speed + correction
            
                prev_error = error

        if state == "TURN":    
            # if the magnitude of yaw_rate is integrated, we can get yaw
            # the turn is complete when yaw becomes 90 degrees, i.e, 1.57/2 in rads
        
            turn_angle += abs(yaw_rate) * dt

            if turn_direction == "LEFT":
                left_vel = -turn_speed
                right_vel = turn_speed

            else:
                left_vel = turn_speed
                right_vel = -turn_speed

            if turn_angle >= (1.57/2):
                state = "FOLLOW"
                # Reset PID after turn
                integral = 0.0
                prev_error = 0.0
               
        
    else:
        # stop if maze is completed
        left_vel = 0.0
        right_vel = 0.0
        print("No walls in range on any side - clear of the maze")
        
    client.publish(TOPIC_WHEEL_VEL, json.dumps({
        "left": float(left_vel), "right": float(right_vel),
    }))


def main():
    client = _mqtt_client()
    client.on_message = on_message
    client.connect(MQTT_HOST, MQTT_PORT)
    client.subscribe(TOPIC_SENSORS)
    client.loop_forever()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
