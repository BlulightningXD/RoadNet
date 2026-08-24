# RoadNet Web Dashboard (Bluetooth Integration)

Real-time telemetry and cooperative collision safety monitoring dashboard for RoadNet via **Bluetooth Classic SPP**.

---

## 1. Quick Start Web Dashboard

```powershell
python -m pip install -r requirements.txt
python app.py
```
Open **http://localhost:8080** in your browser.

---

## 2. Hardware Wiring (NodeMCU ESP32)

| Module | Pin | ESP32 Pin | Description |
| :--- | :--- | :--- | :--- |
| **MPU6050** | `SDA` | `GPIO 21` | I2C Data |
| | `SCL` | `GPIO 22` | I2C Clock |
| | `VCC` / `GND` | `3.3V` / `GND` | Power |
| **GPS (NEO-6M)** | `TX` | `GPIO 16` (RX2) | ESP32 Receives GPS Data |
| | `RX` | `GPIO 17` (TX2) | Optional |
| | `VCC` / `GND` | `3.3V` or `5V` / `GND` | Power |
| **LoRa SX1278** | `SCK` | `GPIO 18` | SPI Clock |
| (RA-02 433MHz) | `MISO` | `GPIO 19` | SPI MISO |
| | `MOSI` | `GPIO 23` | SPI MOSI |
| | `NSS` / `SS` | `GPIO 5` | Chip Select |
| | `RST` | `GPIO 14` | Reset |
| | `DIO0` | `GPIO 26` | Packet Interrupt |

---

## 3. Flash the ESP32 Firmware

1. Open [`esp32/roadnet_esp32.ino`](file:///c:/Users/sahaa/Documents/Codex/2026-08-24/files-mentioned-by-the-user-build/outputs/roadnet-web/esp32/roadnet_esp32.ino) in Arduino IDE.
2. In Arduino IDE **Library Manager** (`Ctrl+Shift+I`), install:
   - **LoRa** by Sandeep Mistry
   - **TinyGPSPlus** by Mikal Hart
   - *(Note: `BluetoothSerial` is included with the official ESP32 Arduino core)*
3. Select board `ESP32 Dev Module`, select your USB port, and click **Upload**.

---

## 4. Pair ESP32 Bluetooth in Windows

1. Turn on Bluetooth on your PC.
2. Open Windows **Settings** &rarr; **Bluetooth & devices** &rarr; **Add device** &rarr; **Bluetooth**.
3. Select **`RoadNet-ESP32`** to pair.
4. Go to **More Bluetooth settings** (or *Bluetooth settings* &rarr; *COM Ports* tab).
5. Look for the **Outgoing** COM port assigned to `RoadNet-ESP32` (e.g. `COM7` or `COM8`).

---

## 5. Connect to the Web Dashboard

1. In the RoadNet web header (at `http://localhost:8080`):
   - **Connection**: `Bluetooth (Virtual COM Port)`
   - **Bluetooth COM Port**: Enter your outgoing port (e.g., `COM7`)
   - Click **Connect**
2. The web app connects wirelessly over Bluetooth and displays real-time GPS position on the map, live inertial graphs, active LoRa nodes, and collision alerts.



