import asyncio
import websockets
import json
import time
import os
import re

def state_message(position, orientation):
    example_state = {
        "pose": {
            "position": list(position),
            "orientation": list(orientation)
        },
        "rotor_states": [
            {"rpm": 0},
            {"rpm": 0},
            {"rpm": 0},
            {"rpm": 0}
        ]
    }

    set_state_message = {
        "channel": "setDroneState",
        "data": {
            "id": "default",
            "data": example_state
        }
    }
    return set_state_message


with open("model.json", "r") as f:
    model = json.load(f)

logfile = "log_149_2023-11-30-14-11-24.ulg"
logs = os.listdir(os.path.join("logs_csv", logfile))
#extract estimator_local_position_0 from logs_csv/log_149_2023-11-30-14-11-24.ulg/log_149_2023-11-30-14-11-24_estimator_local_position_0.csv using regex
regex = re.compile(logfile[:-4] + "_(\w+).csv")
datapoints = [regex.match(log).group(1) for log in logs if regex.match(log) is not None]


exit()

async def connect_to_websocket():
    uri = "ws://localhost:8080"
    async with websockets.connect(uri) as websocket:

        add_drone_message = {
            "channel": "addDrone",
            "data": model
        }

        await websocket.send(json.dumps(add_drone_message))
        for t_ms in range(0, 10000, 10):
            t = t_ms / 1000
            position = [(t % 10)/10, 0, 0]
            orientation = [
                [1.0, 0.0, 0.0],
                [0.0, 0.984807753012208, -0.17364817766693033],
                [0.0, 0.17364817766693033, 0.984807753012208]
            ]
            await websocket.send(json.dumps(state_message(position, orientation)))
            time.sleep(0.01)

asyncio.get_event_loop().run_until_complete(connect_to_websocket())
