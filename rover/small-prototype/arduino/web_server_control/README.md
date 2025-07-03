#  Arduino Rover - Web Server Control Interface

This project provides a modular and scalable control system for a custom six-wheeled rover using **Wi-Fi access point + web interface**. It allows full directional control via a webpage hosted directly by the rover. The rover uses **Adafruit Motor Shields**, an **LSM6DS3 IMU**, and a Wi-Fi-enabled board (e.g., Arduino WiFi Rev2 or ESP module via UART).


---

##  Main File

The primary Arduino sketch is: web_server_control.ino

It initializes all components (WiFi, IMU, MotorShields, SD), hosts the web interface, and handles HTTP requests for rover motion commands.


##  Features

-  Wi-Fi Access Point mode using `WiFiNINA` or compatible interface
-  Embedded HTTP server to serve control UI (buttons + velocity input)
-  Fully functional directional control (8 directions + Stop)
-  Real-time pitch/roll/yaw tracking via Kalman Filter
-  IMU data logging to SD card or Serial
-  Web UI optimized for smartphone control



##  Getting Started

1. Open `web_server_control.ino` in the Arduino IDE.
2. Make sure all `.ino` files are in the same folder (Arduino treats them as one sketch).
3. Modify any top-level `#define` parameters (like motor shield address or initial speed).
4. Upload the sketch to your board.
5. Connect to the rover's Wi-Fi network from your phone or PC.
6. Open a browser and navigate to `http://192.168.4.1`.


##  License

This code is provided for academic, prototyping, and research purposes.  
Feel free to adapt and reuse it with attribution.

---
