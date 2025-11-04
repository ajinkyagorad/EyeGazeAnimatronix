import time
from machine import Pin, ADC
from servo import Servo
import random

print("controller_control_mapped.py starting")
time.sleep(1)

# Controller pins
LR = ADC(28)       # joystick X
UD = ADC(26)       # joystick Y
trim = ADC(27)     # rotary (optional gain)
push = Pin(9, Pin.IN, Pin.PULL_UP)  # blink button

# Switches
enable = Pin(6, Pin.IN, Pin.PULL_UP)
mode = Pin(7, Pin.IN, Pin.PULL_UP)
led = Pin(25, Pin.OUT)

# Servo setup
servos = {
    "LR": Servo(pin_id=10),
    "UD": Servo(pin_id=11),
    "TL": Servo(pin_id=12),
    "BaseX": Servo(pin_id=13),
    "TR": Servo(pin_id=14),
    "BaseY": Servo(pin_id=15),
}

servo_limits = {
    "LR": (40, 140),
    "UD": (40, 140),
    "TL": (90, 170),
    "TR": (90, 10),
    "BaseX": (10, 170),
    "BaseY": (40, 140),
}

servo_targets = {k: 90 for k in ["LR","UD","BaseX","BaseY"]}
servo_targets["BXTarget"] = 90
servo_targets["BYTarget"] = 90

# Measured joystick ranges
LR_MIN, LR_MAX = 304, 65535
UD_MIN, UD_MAX = 320, 65311
TRIM_MIN, TRIM_MAX = 7217, 15619

# Scaling
magnify = 1.3   # >1 increases movement range, <1 decreases

neck_timer_ms = 1200
neck_trigger_time = 0
neck_flag = False
last_update = time.ticks_ms()
last_servo_write = time.ticks_ms()
stable_delay_ms = 50

def map_range(x, in_min, in_max, out_min, out_max):
    if x < in_min: x = in_min
    if x > in_max: x = in_max
    return out_min + (x - in_min) * (out_max - out_min) // (in_max - in_min)

def calibrate():
    for s in servos.values():
        s.write(90)
    print("[CALIBRATION] all servos centered at 90°")

def blink():
    print("[ACTION] Blink triggered")
    servos["TL"].write(90)
    servos["TR"].write(90)
    time.sleep_ms(80)
    lid_sync()

def lid_sync():
    ud_min, ud_max = servo_limits["UD"]
    tl_min, tl_max = servo_limits["TL"]
    tr_min, tr_max = servo_limits["TR"]

    ud_progress = (servo_targets["UD"] - ud_min) / (ud_max - ud_min)
    tl_target = tl_max - ((tl_max - tl_min)*(0.5*(1-ud_progress))) - 10
    tr_target = tr_max + ((tr_min - tr_max)*(0.5*(1-ud_progress))) + 10
    servos["TL"].write(tl_target)
    servos["TR"].write(tr_target)

def neck_target(LR, UD):
    servo_targets["BXTarget"] = LR
    servo_targets["BYTarget"] = 90 - ((90 - UD) * 0.6)
    print(f"[NECK] BX={servo_targets['BXTarget']} BY={servo_targets['BYTarget']}")

def neck_smooth_move(speed_deg_per_s=60):
    global last_update
    now = time.ticks_ms()
    dt = time.ticks_diff(now, last_update)
    last_update = now
    step = (speed_deg_per_s * dt) / 1000.0

    dx = servo_targets["BXTarget"] - servo_targets["BaseX"]
    dy = servo_targets["BYTarget"] - servo_targets["BaseY"]

    if abs(dx) > step:
        servo_targets["BaseX"] += step if dx > 0 else -step
    else:
        servo_targets["BaseX"] = servo_targets["BXTarget"]

    if abs(dy) > step:
        servo_targets["BaseY"] += step if dy > 0 else -step
    else:
        servo_targets["BaseY"] = servo_targets["BYTarget"]

    servos["BaseX"].write(int(servo_targets["BaseX"]))
    servos["BaseY"].write(int(servo_targets["BaseY"]))

print("[RUNNING] Controller loop starting")

while True:
    mode_state = not mode.value()
    enable_state = not enable.value()

    if mode_state:
        calibrate()
        time.sleep_ms(300)
        continue

    raw_lr = LR.read_u16()
    raw_ud = UD.read_u16()

    # Map joystick range → servo range, with magnification
    lr_mapped = map_range(raw_lr, LR_MIN, LR_MAX,
                          servo_limits["LR"][0], servo_limits["LR"][1])
    ud_mapped = map_range(raw_ud, UD_MIN, UD_MAX,
                          servo_limits["UD"][0], servo_limits["UD"][1])

    # Center offset compensation
    lr_center = (servo_limits["LR"][0] + servo_limits["LR"][1]) / 2
    ud_center = (servo_limits["UD"][0] + servo_limits["UD"][1]) / 2
    lr_angle = int(lr_center + (lr_mapped - lr_center) * magnify)
    ud_angle = int(ud_center + (ud_mapped - ud_center) * magnify)

    # Clamp within valid limits
    lr_angle = max(servo_limits["LR"][0], min(servo_limits["LR"][1], lr_angle))
    ud_angle = max(servo_limits["UD"][0], min(servo_limits["UD"][1], ud_angle))

    now = time.ticks_ms()
    if time.ticks_diff(now, last_servo_write) > stable_delay_ms:
        servos["LR"].write(lr_angle)
        servos["UD"].write(ud_angle)
        last_servo_write = now
        print(f"[SERVO] LR={lr_angle} UD={ud_angle} raw=({raw_lr},{raw_ud})")

    if push.value():
        blink()

    lid_sync()

    if abs(lr_angle - 90) >= 10 or abs(ud_angle - 90) >= 10:
        if not neck_flag:
            neck_trigger_time = time.ticks_ms()
            neck_flag = True
        if neck_flag and (time.ticks_ms() - neck_trigger_time) >= neck_timer_ms:
            neck_target(lr_angle, ud_angle)
            neck_flag = False

    neck_smooth_move()
