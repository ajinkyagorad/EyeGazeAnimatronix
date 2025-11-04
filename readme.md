# Camera-Eye Follower Bot  
Built with the NMRobotics “Camera-Eye Follower Bot” kit and custom firmware  

A face-tracking animatronic head mechanism. A webcam detects a human face, sends X/Y offset data to a microcontroller board (e.g., Raspberry Pi Pico), which drives metal-gear MG90S servos for eye pan/tilt, eyelids, and a neck follow motion.

## 🎥 Demonstration Videos

[![Eye Tracking Demo](https://img.youtube.com/vi/8LblFQowTHk/hqdefault.jpg)](https://www.youtube.com/shorts/8LblFQowTHk)
*Real-time face tracking with smooth servo motion.*

[![Full Head Follow Demo](https://img.youtube.com/vi/24ze30ukBDU/hqdefault.jpg)](https://www.youtube.com/shorts/24ze30ukBDU)
*Eyes, eyelids, and neck coordination responding to a moving target.*



## Kit & Hardware  
This project uses the NMRobotics kit titled *Camera-Eye Follower Bot* from NMRobotics. :contentReference[oaicite:2]{index=2}  
Parts include:  
- MG90S metal-gear micro servos (eye pan/tilt, lids, neck)  
- 3D-printed eye mechanism (from NMRobotics)  
- A controller board (or compatible microcontroller)  
- USB camera for face detection  
- Wiring and power supply for servos  

## Firmware & Software  
Two major parts:

### 1. PC Side - Face Detection & Serial Output  
Script: `CameraProcessor.py`  
Requires:  
```bash
pip install opencv-python mediapipe pyserial
````

Operation:

* Opens USB camera using OpenCV
* Uses Mediapipe face-detection to find a face in video frames
* Calculates error offsets (error_x, error_y) relative to frame centre
* Sends these offsets as comma-separated text over USB serial to the microcontroller (e.g., “-45,22\n”)

### 2. Microcontroller Side - Servo Control

The microcontroller (e.g., Raspberry Pi Pico with MicroPython) runs firmware: `FollowerBotPicoCode.py` (or renamed `main.py`)
It:

* Reads serial data lines from USB (“error_x,error_y”)
* Converts these offsets into servo target angles for eye pan/tilt
* After a threshold, moves the neck servos to follow the eyes
* Handles eyelid sync/blink logic
* Uses metal-gear MG90S servos with defined angle limits

## Setup Instructions

### Flash MicroPython to the Pico

1. Hold BOOTSEL on Pico, plug USB in → Pico appears as `RPI-RP2` drive.
2. Download the latest MicroPython UF2 for Pico from [https://micropython.org/download/rp2-pico/](https://micropython.org/download/rp2-pico/)
3. Drag-and-drop the `.uf2` file to the Pico drive → Pico reboots into MicroPython.

### Upload firmware & libraries to Pico

```bash
mpremote connect COM4 cp servo.py :
mpremote connect COM4 cp FollowerBotPicoCode.py :main.py
```

*Replace `COM4` with your actual port.*

### Wiring summary

* Eye pan servo → GPIO GP10
* Eye tilt servo → GPIO GP11
* Left eyelid servo → GP12
* Right eyelid servo → GP14
* Neck pan/base servo → GP13
* Neck tilt servo → GP15
* Servo power → 5 V supply capable of ~2 A total; ground common with Pico
* USB camera to PC; USB serial from Pico to PC

### Run the system

1. Plug in Pico; it should auto-run `main.py`.
2. On PC, run the face detection script:

   ```bash
   python CameraProcessor.py
   ```

   Ensure your serial port (e.g., `COM4`) matches the Pico.
3. Watch the servos: eye mechanism pans/tilts to keep face centred; eyelids react; neck follows after eyes move significantly.

## Calibration & Tuning

* Measure joystick/controller or dial inputs if used for manual control.
* Adjust servo angle limits in firmware (`servo_limits = { "LR": (40,140), ... }`) to match your mechanical range.
* Adjust sensitivity gain (`Kp`) or magnification factor for smoother movement.
* Ensure the face detection is stable: lighting, camera angle, detection confidence affect responsiveness.

## Troubleshooting

| Issue                     | Check                                                                                                                 |
| ------------------------- | --------------------------------------------------------------------------------------------------------------------- |
| No servo movement         | Verify servo power (5V), common ground, correct GPIO pins, and `servo.py` compatibility.                              |
| Serial data not received  | Confirm PC script is using correct COM port, baud rate (115200), and Pico is not occupying port with another process. |
| Face detection unstable   | Check webcam driver, lighting, Mediapipe model confidence threshold.                                                  |
| Neck or eyelid motion odd | Verify mapping of error_x/error_y to servo directions; you may need to invert signs or switch axis mapping.           |

## License & Attribution

This project is based on the NMRobotics Camera-Eye Follower Bot kit and its design ideology. For original design and parts, refer to the NMRobotics website. ([NM Robotics][1])
The firmware and scripts here are custom; feel free to adapt under your preferred open-source license.

## Acknowledgements

* NMRobotics for the base kit and animatronic eye mechanism
* Mediapipe by Google for face-detection tools
* OpenCV community for camera integration
* Raspberry Pi Foundation for the Pico microcontroller platform

