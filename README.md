# Water Level Controller for Home

This project implements a smart water tank pump controller using an ESP32 microcontroller. It manages the transfer of water between an Underground Tank (UGT) and an Overhead Tank (OHT), ensuring safe, efficient operation with dry-run protection and a web interface for monitoring and manual control.

## 1. Problem Statement

The system is designed to automatically control a water pump to transfer water from a UGT to an OHT. Key challenges addressed include:

- Preventing pump operation when UGT is empty (dry run).
- Automating pump ON/OFF based on OHT and UGT water levels.
- Providing manual override and web-based visibility/control.

## 1.1 Tank Level Thresholds

- **Overhead Tank (OHT):**
  - `OHT_FULL`: Tank is full.
  - `OHT_EMPTY`: Tank is empty.

- **Underground Tank (UGT):**
  - `UGT_EMPTY`: Tank is empty.
  - `UGT_NOT_EMPTY`: Tank has sufficient water.

## 2. Use Cases

### 2.1 Pump Activation

The pump is activated when:
- OHT level is below `OHT_EMPTY`.
- UGT level is above `UGT_EMPTY`.

### 2.2 Pump Deactivation

The pump is turned off when:
- OHT reaches `OHT_FULL`, or
- UGT falls below `UGT_EMPTY`, or
- A dry-run condition is detected.

If deactivation is due to UGT low level, the pump will only reactivate once UGT is above `UGT_NOT_EMPTY`.

### 2.3 Dry Run Detection & Handling

- Dry-run is detected via motor current using a Current Transformer (CT).
- For a 0.5 HP motor, current < 2A indicates a dry run.
- On dry run:
  - A buzzer alerts every 30 minutes.
  - Alarm can be cleared with a manual switch.
  - Pump enters manual mode (can be toggled via control switch).

## 3. Hardware Implementation

- **Controller**: ESP32
- **Features**:
  - Water level monitoring (UGT & OHT)
  - Motor control (auto/manual)
  - Current sensing
  - Configurable dry-run threshold
  - Web UI for:
    - Displaying water levels
    - Manual motor control
    - Viewing current consumption
    - Threshold configuration

## 4. Web UI

A web-based interface hosted on ESP32 allows real-time visualization and control of the pump system.

---

### Author
Joseph George

### License
This project is open-source and available under the [MIT License](LICENSE).

