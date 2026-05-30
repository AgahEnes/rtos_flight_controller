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

Open the printed localhost URL in Chrome. Use `Serial` for the STM32 UART stream or `Sim` for a deterministic demo stream.

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
