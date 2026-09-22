# esp32-csi-presence-sensor

A real-time WiFi Channel State Information (CSI) presence sensor using the ESP32

## Installation

### Pre-Installation

In this repo is only one file you will add to an existing repo. So for installing you have to use [this repo](https://github.com/Michdo93/esp32-csi-heatmap). Please have a look at the `setup guide` for the `esp` microcontroller.

```
git clone https://github.com/Michdo93/esp32-csi-heatmap.git
cd esp32-csi-heatmap

python -m venv .

# Windows
.\Scripts\activate

# Linux / macOS
source bin/activate

pip install -r requirements.txt
```

### Installing by adding the sensor file

```
cd esp32-csi-heatmap

# Windows
.\Scripts\activate

# Linux / macOS
source bin/activate

pip install paho-mqtt

wget https://raw.githubusercontent.com/Michdo93/esp32-csi-presence-sensor/refs/heads/main/sensor.py
```

## Usage

You can run it with

```
python sensor.py --port <usb_port> --mqtt-host <ip_address> --mqtt-user <user> --mqtt-pass <password>
```

As example this could be:

```
python sensor.py --port COM3 --mqtt-host 192.168.1.100 --mqtt-user homeassistant --mqtt-pass test1234
```
