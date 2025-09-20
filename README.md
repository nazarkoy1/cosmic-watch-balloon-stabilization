# cosmic-watch-balloon-stabilization
Arduino gimbal + IMU firmware to stabilize a Cosmic Watch muon detector for high-altitude balloon tests; code, CAD, and docs. Stabilizing a Cosmic Watch payload with an Arduino Nano + MPU6050 (+ servos). Includes firmware, 3D files, and test notes. Balloon-flight stabilization for Cosmic Watch: Arduino control, servo gimbal, and lab pendulum tests.

## Cosmic Watch Firmware — Quick Start

### Prerequisites
- **Arduino IDE** (v2.x recommended)
- **Board:** Arduino Nano (ATmega328P). If upload fails, try **ATmega328P (Old Bootloader)**.
- **Libraries:** Wire (built-in), Servo (built-in). *(Add any others you used.)*

### Wiring (summary)
- Cosmic Watch pulse output → **Nano D2** *(example; change if your sketch uses a different pin)*
- Common **5V** and **GND** between Cosmic Watch and Nano

### Upload
1. Open `cosmic-watch-firmware/cosmic_watch_firmware.ino` in Arduino IDE.
2. **Tools → Board:** Arduino Nano; **Processor:** ATmega328P (or Old Bootloader).
3. **Tools → Port:** select your Nano’s port.
4. **Verify** then **Upload**.

### Run / Observe
- Open **Serial Monitor** or **Serial Plotter**; set **baud** to match your code (e.g., `115200`).
- You should see count lines (e.g., CSV). Press **Reset** if nothing prints after power-up.

### Data Logging (simple)
- In Serial Monitor, use “Save output” to capture CSV;  
  *(or extend the sketch to write to SD if you add a logger).*

### Troubleshooting
- `avrdude` / sync errors → switch **Processor** between ATmega328P and **Old Bootloader**.
- Random symbols → wrong **baud**; match `Serial.begin(...)`.
- No counts → recheck 5V/GND, pulse pin number, and cable/connector.

### Attribution
Portions inspired by the **CosmicWatch Desktop Muon Detector v2** materials by Spencer N. Axani et al.  
Source: https://github.com/spenceraxani/CosmicWatch-Desktop-Muon-Detector-v2

