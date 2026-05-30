# RTOS Flight Telemetry Dashboard

This dashboard is a jury-facing live telemetry tool for the STM32F411 RTOS flight-controller project.

## What It Reads Today

The dashboard accepts two binary formats:

### Current Repository IMU Frame

- Sync: `0xA5 0x5A`
- Message id: `0x10`
- Length: 38 bytes
- Payload: sequence, timestamp, accel XYZ, gyro XYZ, IMU temperature
- CRC: CRC16/CCITT-FALSE, little-endian

### Mentor Extended IMU & Sensor Frame

- Sync: `0xAA 0x55`
- Message id: `0x10`
- Payload length byte: `65`
- Total length: 71 bytes
- Payload: timestamp, accel XYZ, gyro XYZ, IMU temperature, magnetometer XYZ, barometer pressure, GNSS coordinates, GNSS altitude, fix type
- CRC: CRC16/CCITT-FALSE, little-endian

The dashboard estimates roll and pitch from accelerometer plus gyro using a small complementary filter. Yaw is gyro-integrated because the current packet does not yet include magnetometer or fused yaw.

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

This path uses Chrome Web Serial and reads the same binary packets produced by `telemetry_task.c`.

### 2. Wired ESP32 Simulation Without ESP32

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

## Why This Is Separate From Firmware

The dashboard is intentionally placed under `Tools/TelemetryDashboard` so the embedded firmware remains deterministic and small. Firmware only sends compact binary packets; visualization, parsing, history buffers, and 3D rendering stay on the PC side.

## Firmware Data Needed Next

For the final demo, the dashboard can become more accurate if firmware later adds these fields:

- Fused roll, pitch, yaw in degrees.
- Balance mode: acquiring, balanced, recovering, failure.
- Servo fin A/B command angles.
- BMP180 pressure, altitude, and barometer temperature.
- Packet health counters or fault bits.

Until those fields exist, the dashboard derives mode and servo demand from the live IMU stream and marks the attitude as an `IMU estimate`.
