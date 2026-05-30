# Telemetry Dashboard Packet Plan

The current firmware sends only the IMU packet, which is enough for a first dashboard and demo simulation. For the final balance-platform presentation, add one extended state packet instead of overloading the IMU packet.

## Current Repository Packet

| Field | Value |
| --- | --- |
| Sync | `0xA5 0x5A` |
| Message id | `0x10` |
| Length | 38 bytes |
| Payload | sequence, timestamp, accel XYZ, gyro XYZ, IMU temperature |
| CRC | CRC16/CCITT-FALSE over all bytes before CRC |

## Mentor Extended Sensor Packet

The mentor packet sketch uses this structure:

| Field | Value |
| --- | --- |
| Sync | `0xAA 0x55` |
| Message id | `0x10` |
| Payload length | `65` |
| Total length | 71 bytes |
| Payload | timestamp, IMU, magnetometer, barometer, GNSS/fix data |
| CRC | CRC16/CCITT-FALSE over bytes 0 through 68 |

The dashboard parser supports this packet as `extended-imu-sensors`.

## Future State Packet

| Field | Type | Unit |
| --- | --- | --- |
| Sync | `uint8[2]` | `0xA5 0x5A` |
| Message id | `uint8` | `0x20` |
| Sequence | `uint8` | counter |
| Timestamp | `uint32_le` | ms |
| Mode | `uint8` | 0 acquiring, 1 balanced, 2 recovering, 3 failure |
| Fault bits | `uint16_le` | bit field |
| Roll | `float32_le` | degree |
| Pitch | `float32_le` | degree |
| Yaw | `float32_le` | degree |
| Tilt | `float32_le` | degree |
| Servo A | `float32_le` | degree |
| Servo B | `float32_le` | degree |
| Temperature IMU | `float32_le` | C |
| Temperature BMP180 | `float32_le` | C |
| Pressure | `float32_le` | Pa |
| Altitude | `float32_le` | m |
| CRC | `uint16_le` | CRC16/CCITT-FALSE |

## Integration Shape

1. Keep `App/Telemetry/telemetry_task.c` as the packet owner.
2. Add a second pack helper beside `TelemetryTask_prvPackImu`.
3. Read fused attitude and BMP180 topics from `global_data_space`.
4. Send IMU packet at 100 ms and state packet at 100 ms, or state at 50 ms if UART budget allows.
5. Keep UART packet generation out of sensor drivers.

At 115200 baud, a 60-byte packet at 10 Hz is roughly 600 B/s before UART framing, so it is safe for this project.
