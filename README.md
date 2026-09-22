# esp32-csi-presence-sensor

A real-time WiFi Channel State Information (CSI) presence sensor using an ESP32 micro-controller and Python.

It analyzes subtle distortions in micro-second WiFi signals (amplitude & phase variations) caused by human movement inside a room, effectively acting as an invisible **"WiFi Radar"** motion and occupancy detector.

---

## Features

- **Home Assistant Auto-Discovery:** Automatically creates a native `binary_sensor` (`occupancy` class) and a `sensor` for the detection score upon startup.
- **Debounced State Detection:** Built-in signal smoothing, noise suppression, and debouncing to prevent false triggers.
- **State-Change Publishing:** MQTT messages are only published when state transitions occur (`ON` / `OFF`), preserving network bandwidth.
- **Real-Time Data Streaming:** Publishes numerical detection scores for advanced automation or telemetry.
- **Integrated Matplotlib Visualizer:** Real-time heatmaps and time-series plots for live monitoring and calibration debugging.

---

## How It Works

1. **Signal Capture:** The ESP32 collects raw WiFi CSI subcarrier data and streams it over USB Serial.
2. **Calibration Phase:** Upon launch, the script collects baseline data (e.g., empty room for ~80 frames) to calculate standard deviation and set a dynamic detection threshold.
3. **Signal Processing:** Raw subcarriers are smoothed, filtered, and compared against the dynamic baseline. A score derived from the top subcarrier differences is evaluated.
4. **State Transition:** If the score exceeds the threshold for a set number of consecutive frames (debouncing), the presence state flips to `ON`. When movement ceases, it drops back to `OFF`.

---

## Installation

### Pre-Installation

This project builds on top of the [esp32-csi-heatmap](https://github.com/Michdo93/esp32-csi-heatmap) base repository. Flash the ESP32 with the corresponding CSI firmware as described in that repo.

```bash
# Clone the base repository
git clone https://github.com/Michdo93/esp32-csi-heatmap.git
cd esp32-csi-heatmap

# Create and activate Python virtual environment
python -m venv .

# Windows
.\Scripts\activate

# Linux / macOS
source bin/activate

# Install base dependencies
pip install -r requirements.txt
```

### Installing the Sensor Script

Download the MQTT-enabled `sensor.py` file and install the required MQTT library:

```bash
cd esp32-csi-heatmap

# Activate environment if not already active
# Windows: .\Scripts\activate  |  Linux/macOS: source bin/activate

pip install paho-mqtt

# Download sensor script
wget https://raw.githubusercontent.com/Michdo93/esp32-csi-presence-sensor/refs/heads/main/sensor.py
```

> **Linux / Raspberry Pi Permission Note:** Ensure your user has access to serial ports by adding it to the `dialout` group: `sudo usermod -a -G dialout $USER` (requires logout or reboot).

---

## Usage

Run the script by providing the USB port and your MQTT broker details:

```bash
python sensor.py --port <usb_port> --mqtt-host <ip_address> [options]
```

### Example Commands

**Basic Usage (Windows):**
```bash
python sensor.py --port COM3 --mqtt-host 192.168.1.100 --mqtt-user homeassistant --mqtt-pass test1234
```

**Basic Usage (Linux / Raspberry Pi):**
```bash
python sensor.py --port /dev/ttyUSB0 --mqtt-host 192.168.1.100 --mqtt-user homeassistant --mqtt-pass test1234
```

**Custom Threshold & Device ID:**
```bash
python sensor.py --port /dev/ttyUSB0 --mqtt-host 192.168.1.100 --device-id livingroom_csi_radar --factor 2.5
```

---

## Command Line Arguments

| Parameter | Short | Default | Description |
| :--- | :--- | :--- | :--- |
| `--port` | `-p` | `COM3` | Serial USB port connected to the ESP32 (`COMx` on Windows, `/dev/ttyUSB0` or `/dev/ttyACM0` on Linux). |
| `--baud` | `-b` | `921600` | Baud rate for serial communication. |
| `--threshold` | `-t` | *Auto* | Manually overrides automatic threshold setting. |
| `--factor` | `-f` | `3.5` | Multiplier for auto-threshold calculation based on baseline noise. Lower = more sensitive, Higher = less sensitive. |
| `--mqtt-host` | | `None` | IP address or hostname of the MQTT Broker. |
| `--mqtt-port` | | `1883` | Port of the MQTT Broker. |
| `--mqtt-user` | | `None` | Username for MQTT authentication. |
| `--mqtt-pass` | | `None` | Password for MQTT authentication. |
| `--device-id` | | `esp32_csi_radar` | Unique identifier for the sensor used in MQTT topics and Home Assistant entities. |
| `--list` | | `false` | Lists available COM / Serial ports and exits. |

---

## MQTT Configuration & Topics

When connected to an MQTT Broker, the sensor communicates over standard MQTT topics. Replacing `<device_id>` with your configured `--device-id` (default: `esp32_csi_radar`):

### Published Topics

1. **Presence State Topic:**
   - **Topic:** `csi_radar/<device_id>/state`
   - **Retained:** `True`
   - **Possible Payload Values:**
     - `ON` : Human movement / presence detected in range.
     - `OFF` : No presence detected (room is empty).

2. **Score Telemetry Topic:**
   - **Topic:** `csi_radar/<device_id>/score`
   - **Retained:** `False`
   - **Payload Value:** `float` (e.g., `12.45`)
   - **Description:** Real-time variance score derived from the median of the top subcarrier amplitude differences. Useful for custom graphing or fine-tuning thresholds.

### Home Assistant Auto-Discovery

If Home Assistant MQTT discovery is enabled on your broker, two entities will be created automatically without editing `configuration.yaml`:

- **Binary Sensor:** `binary_sensor.<device_id>_presence`
  - **Device Class:** `occupancy`
  - **States:** `Detected` (`ON`) / `Clear` (`OFF`)
- **Sensor:** `sensor.<device_id>_score`
  - **Unit:** `score`

---

## Sensor States & Lifecycle

| Internal Phase | Visual GUI Indicator | MQTT Payload (`state`) | Description |
| :--- | :--- | :--- | :--- |
| **Calibration** | `KALIBRIERUNG` (Orange) | *None* | Runs for the first ~80 frames. Do not move in the room during this phase so a clean noise baseline can be sampled. |
| **Idle / Unoccupied**| `LEER` (Dark Green) | `OFF` | Movement score is below the detection threshold. Baseline slowly adjusts to ambient environmental noise. |
| **Occupied** | `ERKANNT` (Cyan Blue) | `ON` | Movement score exceeded threshold for consecutive frames (debounced). Baseline updating is locked to prevent target drift. |

---

## Troubleshooting

- **Permissions error on Linux (`/dev/ttyUSB0: Permission denied`):**
  Run `sudo usermod -a -G dialout $USER` and log out and back in.
- **Too sensitive / False triggers:**
  Increase the threshold multiplier using `--factor 4.5` or set a manual threshold via `--threshold 10.0`.
- **Sluggish detection:**
  Decrease the factor using `--factor 2.0`.
- **MQTT not connecting:**
  Verify paho-mqtt installation (`pip install paho-mqtt`) and double-check IP, credentials, and firewall settings on your broker.
