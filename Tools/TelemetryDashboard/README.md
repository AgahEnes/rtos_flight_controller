# RTOS Flight Telemetry Dashboard

This dashboard is a jury-facing live telemetry tool for the STM32 RTOS flight-controller project.

## Runtime Principle

The dashboard only visualizes fields that arrive in telemetry packets. It does not estimate attitude, infer flight mode, synthesize servo commands, or run dummy simulation data.

If a packet does not contain a field, the corresponding UI value stays at its initial value. This keeps the PC-side tool aligned with the firmware architecture: Sensor, Navigation, Flight-Control, Actuator, BMP180, and Telemetry tasks own the data; the dashboard only parses and renders it.

## What It Reads Today

### Agah IMU and Vehicle State Frame

- Sync: `0xA5 0x5A`
- Message id: `0x12`
- Length: 63 bytes
- Payload: timestamp, accel XYZ, gyro XYZ, IMU temperature, roll/pitch/yaw, roll-rate/pitch-rate/yaw-rate, estimator-valid flag
- CRC: CRC16/CCITT-FALSE, little-endian

This is the primary firmware packet currently used by Agah's `telemetry_task.c`.

### Agah IMU Calibration Frame

- Sync: `0xA5 0x5A`
- Message id: `0x81`
- Length: 43 bytes
- Payload: timestamp, accelerometer bias XYZ, gyroscope bias XYZ, calibration timestamp, update counter, validity flag
- CRC: CRC16/CCITT-FALSE, little-endian

This frame is parsed for protocol compatibility. It is not used to fabricate missing dashboard values.

### Legacy Akif IMU Frame

- Sync: `0xA5 0x5A`
- Message id: `0x10`
- Length: 38 bytes
- Payload: sequence, timestamp, accel XYZ, gyro XYZ, IMU temperature
- CRC: CRC16/CCITT-FALSE, little-endian

This parser path is kept so older local test firmware can still be inspected. Because this packet has no fused attitude, mode, servo, or BMP180 data, those UI fields remain at their initial values.

### Extended Sensor Frame

- Sync: `0xAA 0x55`
- Message id: `0x10`
- Payload length byte: `65`
- Total length: 71 bytes
- Payload: timestamp, accel XYZ, gyro XYZ, IMU temperature, magnetometer XYZ, barometer pressure, GNSS coordinates, GNSS altitude, fix type
- CRC: CRC16/CCITT-FALSE, little-endian

This parser path is kept for mentor packet experiments. Only explicit fields in the packet are shown.

### ASCII RTOSFUS Frame

ASCII `RTOSFUS,...` lines are supported for temporary wired testing. The parser still follows the same rule: missing fields are not guessed.

## Run

```bash
cd Tools/TelemetryDashboard
npm install
npm run dev
```

Open the printed localhost URL in Chrome.

## Live Data Options

### 1. Direct USB Serial

Use this when STM32 is connected to the same computer that runs the dashboard.

1. Start the dashboard.
2. Open Chrome at `http://127.0.0.1:5173`.
3. Press `Serial`.
4. Select the STM32 virtual COM port.

This path uses Chrome Web Serial and reads the binary packets produced by firmware telemetry.

### 2. Wired Serial-to-WebSocket Relay

Use this while ESP32 is not available. It keeps the dashboard side close to the final WiFi architecture:

```text
STM32 USB/UART -> computer serial relay -> WebSocket -> dashboard
```

List serial ports:

```bash
npm run serial:list
```

Start the relay:

```bash
npm run serial:bridge -- --port=/dev/tty.usbmodemXXXX --baud=115200
```

Open the dashboard with the relay URL prefilled:

```text
http://127.0.0.1:5173/?ws=ws://127.0.0.1:8081/telemetry
```

Then press `WiFi`. The dashboard receives WebSocket binary data exactly like it will receive data from ESP32 later.

### 3. LAN / Phone Test

Run the dashboard on all network interfaces:

```bash
npm run dev:lan
```

If the computer and phone are on the same WiFi or phone hotspot, open this from the phone:

```text
http://<computer-ip>:5173/?ws=ws://<computer-ip>:8081/telemetry
```

For this mode, start the relay with LAN binding:

```bash
npm run serial:bridge -- --port=/dev/tty.usbmodemXXXX --host=0.0.0.0
```

This is a useful rehearsal for the ESP32 path because the dashboard is accessed over the network instead of only localhost.

## Firmware Data Needed Next

For the final demo, firmware telemetry can later add these fields:

- Flight mode: acquiring, balanced, recovering, failure.
- Four servo fin command angles.
- BMP180 pressure, altitude, and barometer temperature.
- Packet health counters or fault bits.

Until those fields exist, the dashboard keeps their initial values.
