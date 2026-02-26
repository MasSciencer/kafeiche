"""
Simple keyboard teleop client for rosbridge.

Arrow keys send velocity commands to the /cmd_vel_server topic.
ESC quits and stops the robot.
"""

import json
import threading
import time
import websocket
from pynput import keyboard

# configuration
ROSBRIDGE_WS = "ws://192.168.1.2:9090"   # rosbridge address

LINEAR_SPEED = 1.0      # m/s
ANGULAR_SPEED = 1.0     # rad/s

current_linear = 0.0
current_angular = 0.0

ws = None

def on_open(ws_instance):
    print("Connected to rosbridge")

def on_error(ws_instance, error):
    print(f"WebSocket error: {error}")

def on_close(ws_instance, close_status_code, close_msg):
    print("rosbridge connection closed")


def send_twist(linear, angular):
    if ws is None or not ws.sock or not ws.sock.connected:
        return

    msg = {
        "op": "publish",
        "topic": "/cmd_vel_server",
        "msg": {
            "header": {
                "stamp": "now",
                "frame_id": "base_link"
            },
            "twist": {
                "linear": {"x": linear, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": angular}
            }
        }
    }
    try:
        ws.send(json.dumps(msg))
    except Exception as e:
        print(f"Failed to send: {e}")


def run_websocket():
    global ws
    ws = websocket.WebSocketApp(
        ROSBRIDGE_WS,
        on_open=on_open,
        on_error=on_error,
        on_close=on_close
    )
    ws.run_forever(ping_interval=30, ping_timeout=10)

# start websocket thread
ws_thread = threading.Thread(target=run_websocket, daemon=True)
ws_thread.start()

# allow time to connect
time.sleep(1.5)

print("Use arrow keys to drive via rosbridge:")
print("      forward")
print("      backward")
print("      turn left")
print("      turn right")
print(" Esc  exit and stop")
print("-" * 60)


def on_press(key):
    global current_linear, current_angular
    changed = False

    try:
        if key == keyboard.Key.up:
            current_linear = LINEAR_SPEED
            changed = True
        elif key == keyboard.Key.down:
            current_linear = -LINEAR_SPEED
            changed = True
        elif key == keyboard.Key.left:
            current_angular = ANGULAR_SPEED
            changed = True
        elif key == keyboard.Key.right:
            current_angular = -ANGULAR_SPEED
            changed = True

        if changed:
            send_twist(current_linear, current_angular)

    except AttributeError:
        pass


def on_release(key):
    global current_linear, current_angular
    changed = False

    if key in (keyboard.Key.up, keyboard.Key.down):
        current_linear = 0.0
        changed = True
    if key in (keyboard.Key.left, keyboard.Key.right):
        current_angular = 0.0
        changed = True

    if changed:
        send_twist(current_linear, current_angular)

    if key == keyboard.Key.esc:
        send_twist(0.0, 0.0)
        print("Exiting. Stopping robot.")
        return False

with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()

# ensure stop on exit
send_twist(0.0, 0.0)
if ws:
    ws.close()
print("Program terminated.")
