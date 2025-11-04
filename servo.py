from machine import Pin, PWM

class Servo:
    """
    Simple servo driver for standard 50 Hz hobby servos.
    Angle range: 0–180 degrees.
    """

    def __init__(self, pin_id, freq=50):
        self.pwm = PWM(Pin(pin_id))
        self.pwm.freq(freq)
        # pulse width 0.5–2.5 ms mapped to 0–180°
        self.min_duty = 1638     # 0.5 ms  (65535 * 0.5 ms / 20 ms)
        self.max_duty = 8192     # 2.5 ms  (65535 * 2.5 ms / 20 ms)

    def write(self, angle):
        """Set servo angle in degrees (0–180)."""
        if angle < 0:
            angle = 0
        elif angle > 180:
            angle = 180
        duty = int(self.min_duty + (angle / 180) * (self.max_duty - self.min_duty))
        self.pwm.duty_u16(duty)
