# Telemetry Dashboard Packet Plan

The dashboard is a visualization endpoint only. It must not estimate attitude, infer flight mode, invent servo commands, or generate dummy telemetry. Every displayed value must come from a parsed firmware packet; if the packet does not contain a field, the UI keeps the initial value.

## Current Agah IMU and Vehicle State Packet

| Field | Value |
| --- | --- |
| Sync | `0xA5 0x5A` |
| Message id | `0x12` |
| Length | 64 bytes |
| Payload | timestamp, accel XYZ, gyro XYZ, IMU temperature, roll/pitch/yaw, roll-rate/pitch-rate/yaw-rate, estimator-valid flag, flight mode (`te_FlightMode` as `uint8`) |
| CRC | CRC16/CCITT-FALSE over all bytes before CRC |

The dashboard uses this packet as the primary live source for IMU and attitude visualization.

## Current Agah IMU Calibration Packet

| Field | Value |
| --- | --- |
| Sync | `0xA5 0x5A` |
| Message id | `0x81` |
| Length | 43 bytes |
| Payload | timestamp, accelerometer bias XYZ, gyroscope bias XYZ, calibration timestamp, update counter, validity flag |
| CRC | CRC16/CCITT-FALSE over all bytes before CRC |

The dashboard parser accepts this frame for protocol compatibility. It does not use calibration frames to synthesize missing dashboard state.

## Legacy Akif IMU Packet

| Field | Value |
| --- | --- |
| Sync | `0xA5 0x5A` |
| Message id | `0x10` |
| Length | 38 bytes |
| Payload | sequence, timestamp, accel XYZ, gyro XYZ, IMU temperature |
| CRC | CRC16/CCITT-FALSE over all bytes before CRC |

This packet is kept for older local tests. It has no fused attitude, flight mode, servo, or BMP180 fields, so those UI values stay at their initial state.

## Mentor Extended Sensor Packet

| Field | Value |
| --- | --- |
| Sync | `0xAA 0x55` |
| Message id | `0x10` |
| Payload length | `65` |
| Total length | 71 bytes |
| Payload | timestamp, IMU, magnetometer, barometer, GNSS/fix data |
| CRC | CRC16/CCITT-FALSE over bytes 0 through 68 |

The dashboard parser supports this packet as `extended-imu-sensors`. Only explicit fields are rendered.

## Firmware Data Needed Next

For the final balance-platform presentation, extend firmware telemetry with explicit fields for:

| Field | Suggested type | Unit |
| --- | --- | --- |
| Fault bits | `uint16_le` | bit field |
| Servo A | `float32_le` | degree |
| Servo B | `float32_le` | degree |
| Servo C | `float32_le` | degree |
| Servo D | `float32_le` | degree |
| BMP180 temperature | `float32_le` | C |
| BMP180 pressure | `float32_le` | Pa |
| BMP180 altitude | `float32_le` | m |

## Integration Shape

1. Keep `App/Telemetry/telemetry_task.c` as the packet owner.
2. Read state, actuator, and BMP180 topics from `global_data_space`.
3. Add fields to a firmware packet only after the corresponding producer task publishes real data.
4. Keep UART packet generation out of sensor and actuator drivers.
5. Update the dashboard parser only when the firmware packet layout changes.

At 115200 baud, a compact state packet at 10 Hz is safe for this project, but final packet frequency should still be chosen by firmware timing analysis.
