from machine import Pin, ADC
import time

# Pin setup
LR = ADC(28)      # Joystick X
UD = ADC(26)      # Joystick Y
trim = ADC(27)    # Rotary
push = Pin(9, Pin.IN, Pin.PULL_UP)  # Joystick push or button

# Normalize 16-bit ADC to 0–1 float
def norm(adc):
    return adc.read_u16() / 65535.0

while True:
    x = norm(LR)
    y = norm(UD)
    r = norm(trim)
    b = 0 if push.value() == 0 else 1  # 0 = pressed, 1 = released
    
    # Use regular print instead of sys.stdout.write
    print(f"{x:.3f},{y:.3f},{r:.3f},{b}")
    
    time.sleep(0.02)  # Single delay for 50 Hz