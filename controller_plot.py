import serial
import time

PORT = "COM4"
ser = serial.Serial(PORT, 115200)

try:
    while True:
        if ser.in_waiting:
            line = ser.readline().decode().strip()
            print(f"\r{line}", end='', flush=True)
        time.sleep(0.01)
except KeyboardInterrupt:
    ser.close()