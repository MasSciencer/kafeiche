# keyboard_rosbridge.py
import json
import threading
import time
import websocket
from pynput import keyboard

# ────────────────────────────────────────────────
# Настройки
# ────────────────────────────────────────────────

ROSBRIDGE_WS = "ws://192.168.1.2:9090"   # ← ваш rosbridge адрес

LINEAR_SPEED = 1.0      # м/с
ANGULAR_SPEED = 1.0     # рад/с

# ────────────────────────────────────────────────

current_linear = 0.0
current_angular = 0.0

ws = None

def on_open(ws_instance):
    print("Подключено к rosbridge")

def on_error(ws_instance, error):
    print(f"Ошибка WebSocket: {error}")

def on_close(ws_instance, close_status_code, close_msg):
    print("Соединение с rosbridge закрыто")

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
        print(f"Ошибка отправки: {e}")

def run_websocket():
    global ws
    ws = websocket.WebSocketApp(
        ROSBRIDGE_WS,
        on_open=on_open,
        on_error=on_error,
        on_close=on_close
    )
    ws.run_forever(ping_interval=30, ping_timeout=10)  # ping для поддержания соединения

# Запуск WebSocket в отдельном потоке
ws_thread = threading.Thread(target=run_websocket, daemon=True)
ws_thread.start()

# Даём время на подключение
time.sleep(1.5)

# ────────────────────────────────────────────────
# Обработка клавиатуры
# ────────────────────────────────────────────────

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
        print("Завершение. Остановка робота.")
        return False  # завершить listener


# ────────────────────────────────────────────────

print("Управление роботом стрелками (через rosbridge):")
print("  ↑     — вперёд")
print("  ↓     — назад")
print("  ←     — влево (против часовой)")
print("  →     — вправо (по часовой)")
print(" Esc   — выход и остановка")
print("-" * 60)

with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()

# Принудительная остановка при выходе
send_twist(0.0, 0.0)
if ws:
    ws.close()
print("Программа завершена.")