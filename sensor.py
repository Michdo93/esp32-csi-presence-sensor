import serial
import serial.tools.list_ports
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import re
import argparse
import threading
import time
import json

# ── MQTT Client Einbindung ───────────────────────────────────────────────────
try:
    import paho.mqtt.client as mqtt
    HAS_MQTT = True
except ImportError:
    HAS_MQTT = False

# ── Konfiguration ────────────────────────────────────────────────────────────
HISTORY_LEN    = 100      # Frames in der Heatmap
SMOOTHING      = 0.25     # Glättung des Rohsignals (0=keine, 1=nur neu)

VALID_SUBCARRIERS = list(range(1, 28)) + list(range(36, 64))
N_VALID = len(VALID_SUBCARRIERS)  # 55

CALIB_FRAMES = 80
BASELINE_DRIFT = 0.002
DEBOUNCE_FRAMES = 8
THRESHOLD_FACTOR = 3.5

# ── Globaler State ────────────────────────────────────────────────────────────
diff_history     = deque(maxlen=HISTORY_LEN)
for _ in range(HISTORY_LEN):
    diff_history.append(np.zeros(N_VALID))

baseline         = None
baseline_buffer  = []
threshold        = None
last_raw         = np.zeros(N_VALID)
presence_history = deque(maxlen=60)
debounce_count   = 0        
state_present    = False    
lock             = threading.Lock()
running          = True
calib_done       = False

# MQTT Globals
mqtt_client      = None
last_mqtt_state  = None   # Verhindert doppeltes Senden desselben Zustands


# ── MQTT Initialisierung & Discovery ─────────────────────────────────────────
def init_mqtt(host, port, user, password, device_id="esp32_csi_radar"):
    global mqtt_client
    if not HAS_MQTT:
        print("[WARN] 'paho-mqtt' ist nicht installiert. Smart Home Publishing ist deaktiviert.")
        return None

    try:
        client = mqtt.Client(client_id=device_id)
        if user and password:
            client.username_pw_set(user, password)

        client.connect(host, port, keepalive=60)
        client.loop_start()
        print(f"[OK] MQTT verbunden mit {host}:{port}")

        # Home Assistant Auto-Discovery payload (Binary Sensor)
        discovery_topic = f"homeassistant/binary_sensor/{device_id}/presence/config"
        discovery_payload = {
            "name": "CSI Radar Anwesenheit",
            "unique_id": f"{device_id}_presence",
            "state_topic": f"csi_radar/{device_id}/state",
            "payload_on": "ON",
            "payload_off": "OFF",
            "device_class": "occupancy",
            "device": {
                "identifiers": [device_id],
                "name": "ESP32 CSI Radar Sensor",
                "model": "ESP32-CSI-Heatmap",
                "manufacturer": "DIY"
            }
        }
        
        # Discovery Payload senden
        client.publish(discovery_topic, json.dumps(discovery_payload), retain=True)

        # Optional: Analoger Sensor für den Score
        score_discovery_topic = f"homeassistant/sensor/{device_id}/score/config"
        score_discovery_payload = {
            "name": "CSI Radar Score",
            "unique_id": f"{device_id}_score",
            "state_topic": f"csi_radar/{device_id}/score",
            "unit_of_measurement": "score",
            "device": {
                "identifiers": [device_id]
            }
        }
        client.publish(score_discovery_topic, json.dumps(score_discovery_payload), retain=True)

        return client
    except Exception as e:
        print(f"[FEHLER] MQTT Verbindung fehlgeschlagen: {e}")
        return None


def publish_state_change(new_state, score, device_id="esp32_csi_radar"):
    global last_mqtt_state, mqtt_client
    if not mqtt_client:
        return

    # 1. Anwesenheitszustand NUR bei Änderung publizieren
    if new_state != last_mqtt_state:
        payload = "ON" if new_state else "OFF"
        topic = f"csi_radar/{device_id}/state"
        mqtt_client.publish(topic, payload, retain=True)
        print(f"[MQTT] Event gesendet -> State: {payload}")
        last_mqtt_state = new_state

    # 2. Score kontinuierlich (z.B. für Graphen) publizieren
    score_topic = f"csi_radar/{device_id}/score"
    mqtt_client.publish(score_topic, f"{score:.2f}")


# ── Parser ────────────────────────────────────────────────────────────────────
def parse_csi_line(line):
    match = re.search(r'"?\[([^\]]+)\]"?', line)
    if not match:
        return None
    try:
        values = list(map(int, match.group(1).split(',')))
        if len(values) < 64:
            return None
        if len(values) >= 128:
            imag = np.array(values[0::2], dtype=float)
            real = np.array(values[1::2], dtype=float)
        else:
            real = np.array(values[0::2], dtype=float)
            imag = np.array(values[1::2], dtype=float)
        return np.sqrt(real**2 + imag**2)[VALID_SUBCARRIERS]
    except Exception:
        return None


# ── Serial-Thread ─────────────────────────────────────────────────────────────
def serial_reader(port, baud, manual_threshold, device_id):
    global baseline, baseline_buffer, threshold, last_raw
    global running, calib_done, debounce_count, state_present

    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"[OK] Verbunden mit {port} @ {baud} Baud")
        print(f"[INFO] Kalibrierung laeuft ({CALIB_FRAMES} Frames) ... Bitte NICHT bewegen!")
    except Exception as e:
        print(f"[FEHLER] Serial: {e}")
        running = False
        return

    while running:
        try:
            raw  = ser.readline()
            line = raw.decode('utf-8', errors='ignore').strip()
            if 'CSI_DATA' not in line:
                continue
            amp = parse_csi_line(line)
            if amp is None:
                continue

            with lock:
                # ── Phase 1: Kalibrierung ──────────────────────────────────
                if not calib_done:
                    baseline_buffer.append(amp.copy())
                    if len(baseline_buffer) >= CALIB_FRAMES:
                        bl_arr   = np.array(baseline_buffer)
                        baseline = np.mean(bl_arr, axis=0)
                        bl_std   = np.std(bl_arr, axis=0)

                        if manual_threshold is not None:
                            threshold = manual_threshold
                        else:
                            threshold = float(np.mean(bl_std) * THRESHOLD_FACTOR)
                            threshold = max(threshold, 0.5)

                        print(f"[OK] Kalibrierung fertig. Schwelle: {threshold:.2f}")
                        last_raw   = baseline.copy()
                        calib_done = True

                # ── Phase 2: Laufbetrieb ───────────────────────────────────
                else:
                    smoothed = SMOOTHING * amp + (1 - SMOOTHING) * last_raw
                    last_raw = smoothed

                    diff_frame = np.abs(smoothed - baseline)
                    diff_history.append(diff_frame.copy())

                    if not state_present:
                        baseline = (1 - BASELINE_DRIFT) * baseline + BASELINE_DRIFT * smoothed

                    top_diffs = np.sort(diff_frame)[-10:]
                    score = float(np.median(top_diffs))

                    # Debouncing
                    if score > threshold:
                        debounce_count = min(debounce_count + 1, DEBOUNCE_FRAMES + 5)
                    else:
                        debounce_count = max(debounce_count - 2, 0)

                    state_present = debounce_count >= DEBOUNCE_FRAMES
                    presence_history.append((score, state_present))

                    # MQTT Triggern
                    publish_state_change(state_present, score, device_id)

        except Exception:
            continue
    ser.close()


def list_ports():
    for p in serial.tools.list_ports.comports():
        print(f"  {p.device} - {p.description}")


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    global running, THRESHOLD_FACTOR, mqtt_client

    parser = argparse.ArgumentParser(description='ESP32 CSI Heatmap & Smart Home Sensor')
    parser.add_argument('--port',      '-p', type=str,   default='COM3')
    parser.add_argument('--baud',      '-b', type=int,   default=921600)
    parser.add_argument('--list',            action='store_true')
    parser.add_argument('--threshold', '-t', type=float, default=None)
    parser.add_argument('--factor',    '-f', type=float, default=THRESHOLD_FACTOR)
    
    # MQTT-Argumente
    parser.add_argument('--mqtt-host', type=str, default=None, help='IP/Hostname des MQTT Brokers (z.B. 192.168.1.50)')
    parser.add_argument('--mqtt-port', type=int, default=1883)
    parser.add_argument('--mqtt-user', type=str, default=None)
    parser.add_argument('--mqtt-pass', type=str, default=None)
    parser.add_argument('--device-id', type=str, default='esp32_csi_radar', help='ID des Sensors im Smart Home')

    args = parser.parse_args()

    THRESHOLD_FACTOR = args.factor

    if args.list:
        list_ports()
        return

    # MQTT Initialisieren (falls Host angegeben)
    if args.mqtt_host:
        mqtt_client = init_mqtt(args.mqtt_host, args.mqtt_port, args.mqtt_user, args.mqtt_pass, args.device_id)

    t = threading.Thread(target=serial_reader,
                         args=(args.port, args.baud, args.threshold, args.device_id), daemon=True)
    t.start()
    time.sleep(1.5)
    if not running:
        return

    # ── Figure (Demo GUI) ─────────────────────────────────────────────────────
    fig = plt.figure(figsize=(14, 8), facecolor='#0d0d0d')
    fig.suptitle(f'ESP32 CSI  |  {args.port}  |  Smart Home Sensor Mode', color='#aaaaaa', fontsize=10, y=0.98)

    ax_hm  = fig.add_axes([0.05, 0.38, 0.62, 0.52])
    ax_amp = fig.add_axes([0.05, 0.07, 0.62, 0.24])
    ax_pr  = fig.add_axes([0.73, 0.07, 0.24, 0.83])

    for ax in [ax_hm, ax_amp, ax_pr]:
        ax.set_facecolor('#0d0d0d')
        for sp in ax.spines.values():
            sp.set_color('#2a2a2a')

    hm_init = np.zeros((HISTORY_LEN, N_VALID))
    im = ax_hm.imshow(hm_init, aspect='auto', origin='lower', cmap='plasma', vmin=0, vmax=15, interpolation='bilinear')
    ax_hm.set_title('Differenz zur Baseline (Heatmap)', color='#cccccc', fontsize=9, pad=5)
    ax_hm.tick_params(colors='#444444', labelsize=7)
    cbar = fig.colorbar(im, ax=ax_hm, pad=0.01, fraction=0.025)
    cbar.ax.tick_params(colors='#444444', labelsize=6)

    x_time = np.arange(HISTORY_LEN)
    score_buf = deque([0.0] * HISTORY_LEN, maxlen=HISTORY_LEN)
    line_score, = ax_amp.plot(x_time, list(score_buf), color='#00ff88', lw=1.0, label='Score')
    line_thr,   = ax_amp.plot([0, HISTORY_LEN], [0, 0], color='#ff4444', lw=1.0, linestyle='--', label='Schwelle')
    ax_amp.set_xlim(0, HISTORY_LEN - 1)
    ax_amp.set_ylim(0, 30)
    ax_amp.grid(True, color='#1a1a1a', lw=0.4)
    ax_amp.legend(fontsize=7, facecolor='#111111', labelcolor='white', loc='upper right')

    circle = plt.Circle((0.5, 0.56), 0.27, color='#0d0d0d', ec='#555500', lw=2.5)
    ax_pr.add_patch(circle)
    txt_state = ax_pr.text(0.5, 0.56, 'KALI-\nBRIERUNG', ha='center', va='center', fontsize=13, fontweight='bold', color='#ffaa00', transform=ax_pr.transAxes)
    txt_pct   = ax_pr.text(0.5, 0.22, '', ha='center', fontsize=8, color='#555555', transform=ax_pr.transAxes)
    txt_score = ax_pr.text(0.5, 0.14, '', ha='center', fontsize=7, color='#444444', transform=ax_pr.transAxes)
    txt_thr2  = ax_pr.text(0.5, 0.08, '', ha='center', fontsize=7, color='#333333', transform=ax_pr.transAxes)
    txt_db    = ax_pr.text(0.5, 0.02, '', ha='center', fontsize=6, color='#2a2a2a', transform=ax_pr.transAxes)
    ax_pr.set_xlim(0, 1)
    ax_pr.set_ylim(0, 1)
    ax_pr.axis('off')

    n_bars = 14
    bars = ax_pr.bar(np.linspace(0.04, 0.96, n_bars), np.zeros(n_bars), width=0.055, bottom=0.76, color='#1a1a1a', alpha=0.9, transform=ax_pr.transAxes)

    def update(frame):
        with lock:
            dh      = np.array(list(diff_history))
            thr     = threshold
            done    = calib_done
            db      = debounce_count
            present = state_present
            ph      = list(presence_history)

        im.set_data(dh)
        vmax = max(float(dh.max()), 2.0)
        im.set_clim(0, min(vmax, 40))

        if not done:
            pct = int(100 * len(baseline_buffer) / CALIB_FRAMES)
            txt_state.set_text('KALI-\nBRIERUNG')
            txt_state.set_color('#ffaa00')
            txt_pct.set_text(f'{pct}% gesammelt')
        else:
            score = ph[-1][0] if ph else 0.0
            score_buf.append(score)
            line_score.set_ydata(list(score_buf))
            if thr:
                line_thr.set_ydata([thr, thr])
                ax_amp.set_ylim(0, max(score * 1.5, thr * 2, 5))

            if present:
                circle.set_facecolor('#00c8ff')
                circle.set_edgecolor('#00ddff')
                txt_state.set_text('ERKANNT')
                txt_state.set_color('#001a22')
            else:
                circle.set_facecolor('#0d0d0d')
                circle.set_edgecolor('#1a3a1a')
                txt_state.set_text('LEER')
                txt_state.set_color('#2a6a2a')

            txt_pct.set_text('Anwesenheit')
            txt_score.set_text(f'Score:    {score:.1f}')
            txt_thr2.set_text(f'Schwelle: {thr:.1f}' if thr else '')
            txt_db.set_text(f'Debounce: {db}/{DEBOUNCE_FRAMES}')

            step = max(1, len(ph) // n_bars)
            for i, bar in enumerate(bars):
                idx = min(i * step, len(ph) - 1)
                s, p = ph[idx]
                bar.set_height(0.09 * (1 if p else 0))
                bar.set_color('#00c8ff' if p else '#1a2a1a')

        return [im, line_score, line_thr, circle, txt_state, txt_pct, txt_score, txt_thr2, txt_db] + list(bars)

    ani = FuncAnimation(fig, update, interval=60, blit=False, cache_frame_data=False)
    plt.show()
    running = False


if __name__ == '__main__':
    main()
