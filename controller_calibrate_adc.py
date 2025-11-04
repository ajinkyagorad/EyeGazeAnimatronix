from machine import ADC, Pin
import time

print("ADC calibration mode")
time.sleep(1)

LR = ADC(28)       # joystick X
UD = ADC(26)       # joystick Y
trim = ADC(27)     # rotary (optional)
push = Pin(9, Pin.IN, Pin.PULL_UP)

lr_min = 65535
lr_max = 0
ud_min = 65535
ud_max = 0
tr_min = 65535
tr_max = 0

last_print = time.ticks_ms()
print("Move joystick slowly through full range on both axes...")

while True:
    lr = LR.read_u16()
    ud = UD.read_u16()
    tr = trim.read_u16()

    if lr < lr_min: lr_min = lr
    if lr > lr_max: lr_max = lr
    if ud < ud_min: ud_min = ud
    if ud > ud_max: ud_max = ud
    if tr < tr_min: tr_min = tr
    if tr > tr_max: tr_max = tr

    #if not push.value():  # optional to stop calibration with button press
    #    print("Calibration ended by button press")
    #    break

    now = time.ticks_ms()
    if time.ticks_diff(now, last_print) > 1000:  # every second
        print(f"LR: min={lr_min} max={lr_max} | UD: min={ud_min} max={ud_max} | Trim: min={tr_min} max={tr_max}")
        last_print = now

    time.sleep_ms(10)
