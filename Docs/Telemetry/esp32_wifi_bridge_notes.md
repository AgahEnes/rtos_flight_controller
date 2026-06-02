# ESP32 WiFi Telemetry Bridge Notes

The dashboard can receive real telemetry in two ways:

1. Direct STM32 USB/UART through Chrome Web Serial.
2. ESP32 WiFi bridge through WebSocket binary frames.
3. Temporary computer relay: STM32 USB/UART to WebSocket, without ESP32.

## Recommended ESP32 Flow

```text
STM32 telemetry_task.c
  -> UART binary packet stream
  -> ESP32 UART RX
  -> WebSocket binary broadcast
  -> Dashboard parser
```

The ESP32 should not convert the telemetry packet to text. It should forward the exact binary bytes produced by `telemetry_task.c`.

## Temporary Wired Relay While ESP32 Is Missing

This mode rehearses the ESP32 transport on the computer:

```text
STM32 telemetry_task.c
  -> USB/UART binary packet stream
  -> Node serial-to-websocket relay
  -> Dashboard WebSocket input
```

Commands:

```bash
cd Tools/TelemetryDashboard
npm install
npm run serial:list
npm run serial:bridge -- --port=/dev/tty.usbmodemXXXX --baud=115200
```

Dashboard URL:

```text
http://127.0.0.1:5173/?ws=ws://127.0.0.1:8081/telemetry
```

Press `WiFi` in the dashboard. This validates the same parser and WebSocket receiver that will be used by the real ESP32 bridge.

## LAN / Mobile Phone Rehearsal

The mentor's "second IP / phone / Ethernet" idea is useful as a network rehearsal:

1. Connect the computer and phone to the same network, or connect the computer to the phone hotspot.
2. Run the dashboard with LAN binding:

```bash
npm run dev:lan
```

3. Run the relay with LAN binding:

```bash
npm run serial:bridge -- --port=/dev/tty.usbmodemXXXX --host=0.0.0.0
```

4. Find the computer IP address, for example `192.168.1.23`.
5. Open this from the phone:

```text
http://192.168.1.23:5173/?ws=ws://192.168.1.23:8081/telemetry
```

This does not require ESP32. It proves that a browser on another network device can receive the telemetry WebSocket stream.

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

- The dashboard is ready for Agah `0xA5 0x5A / 0x12` state packets, Agah `0x81` calibration packets, legacy `0x10` packets, and `0xAA 0x55` mentor extended packets.
- ESP32 firmware needs the `WebSocketsServer` Arduino library.
- STM32 UART must send the same telemetry stream to the ESP32 UART RX.
- The final packet layout must stay byte-for-byte consistent with the dashboard parser.
- If Chrome blocks a direct private-network WebSocket, run the dashboard while connected to the ESP32 AP or use a tiny local relay.
