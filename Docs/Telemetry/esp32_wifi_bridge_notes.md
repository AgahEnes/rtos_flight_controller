# ESP32 WiFi Telemetry Bridge Notes

The dashboard can receive real telemetry in two ways:

1. Direct STM32 USB/UART through Chrome Web Serial.
2. ESP32 WiFi bridge through WebSocket binary frames.

## Recommended ESP32 Flow

```text
STM32 telemetry_task.c
  -> UART binary packet stream
  -> ESP32 UART RX
  -> WebSocket binary broadcast
  -> Dashboard parser
```

The ESP32 should not convert the telemetry packet to text. It should forward the exact binary bytes produced by `telemetry_task.c`.

## Wiring

| STM32 | ESP32 |
| --- | --- |
| UART TX | GPIO16 RX2 |
| GND | GND |

Use 3.3 V logic only. If bidirectional communication is later needed, connect ESP32 TX2 to an unused STM32 UART RX pin. Avoid two active transmitters driving the same STM32 UART RX line.

## Dashboard URL

The included ESP32 sketch starts an access point:

- SSID: `RTOS-FLIGHT-TELEM`
- Password: `flight1234`
- WebSocket URL: `ws://192.168.4.1:81/telemetry`

## Current Blocking Points

- The dashboard is ready for binary `0xA5 0x5A` legacy packets and `0xAA 0x55` mentor extended packets.
- ESP32 firmware needs the `WebSocketsServer` Arduino library.
- STM32 UART must send the same telemetry stream to the ESP32 UART RX.
- The final packet layout must stay byte-for-byte consistent with the dashboard parser.
- If Chrome blocks a direct private-network WebSocket, run the dashboard while connected to the ESP32 AP or use a tiny local relay.
