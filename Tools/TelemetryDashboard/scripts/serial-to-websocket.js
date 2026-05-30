#!/usr/bin/env node
import { SerialPort } from "serialport";
import { WebSocketServer } from "ws";

const DEFAULT_BAUD_RATE = 115200;
const DEFAULT_WS_HOST = "127.0.0.1";
const DEFAULT_WS_PORT = 8081;
const DEFAULT_WS_PATH = "/telemetry";

function readArg(name, fallback = undefined) {
  const prefix = `--${name}=`;
  const match = process.argv.find((arg) => arg.startsWith(prefix));
  if (match) {
    return match.slice(prefix.length);
  }
  const index = process.argv.indexOf(`--${name}`);
  if (index >= 0 && index + 1 < process.argv.length) {
    return process.argv[index + 1];
  }
  return fallback;
}

async function listPortsAndExit() {
  const ports = await SerialPort.list();
  if (ports.length === 0) {
    console.log("No serial ports found.");
    return;
  }
  ports.forEach((port) => {
    const label = [
      port.path,
      port.manufacturer ? `manufacturer=${port.manufacturer}` : "",
      port.serialNumber ? `serial=${port.serialNumber}` : "",
      port.vendorId ? `vid=${port.vendorId}` : "",
      port.productId ? `pid=${port.productId}` : ""
    ].filter(Boolean).join(" | ");
    console.log(label);
  });
}

function usage() {
  console.log(`Usage:
  npm run serial:list
  npm run serial:bridge -- --port=/dev/tty.usbmodemXXXX

Options:
  --port <path>        Required serial port path.
  --baud <number>     Serial baud rate. Default: ${DEFAULT_BAUD_RATE}.
  --host <address>    WebSocket bind host. Default: ${DEFAULT_WS_HOST}.
  --ws-port <number>  WebSocket port. Default: ${DEFAULT_WS_PORT}.
  --path <path>       WebSocket path. Default: ${DEFAULT_WS_PATH}.

Dashboard URL:
  http://127.0.0.1:5173/?ws=ws://127.0.0.1:${DEFAULT_WS_PORT}${DEFAULT_WS_PATH}
`);
}

async function main() {
  if (process.argv.includes("--help") || process.argv.includes("-h")) {
    usage();
    return;
  }

  if (process.argv.includes("--list")) {
    await listPortsAndExit();
    return;
  }

  const serialPath = readArg("port");
  if (!serialPath) {
    usage();
    process.exitCode = 2;
    return;
  }

  const baudRate = Number(readArg("baud", String(DEFAULT_BAUD_RATE)));
  const wsHost = readArg("host", DEFAULT_WS_HOST);
  const wsPort = Number(readArg("ws-port", String(DEFAULT_WS_PORT)));
  const wsPath = readArg("path", DEFAULT_WS_PATH);

  const wss = new WebSocketServer({
    host: wsHost,
    port: wsPort,
    path: wsPath
  });

  const serial = new SerialPort({
    path: serialPath,
    baudRate,
    autoOpen: false
  });

  let bytesForwarded = 0;
  let chunksForwarded = 0;

  wss.on("connection", (socket, request) => {
    const remote = `${request.socket.remoteAddress}:${request.socket.remotePort}`;
    console.log(`[ws] client connected: ${remote}`);
    socket.on("close", () => console.log(`[ws] client disconnected: ${remote}`));
  });

  serial.on("data", (chunk) => {
    bytesForwarded += chunk.length;
    chunksForwarded += 1;
    for (const client of wss.clients) {
      if (client.readyState === client.OPEN) {
        client.send(chunk, { binary: true });
      }
    }
  });

  serial.on("error", (error) => {
    console.error(`[serial] ${error.message}`);
  });

  wss.on("listening", () => {
    const wsUrl = `ws://${wsHost}:${wsPort}${wsPath}`;
    console.log(`[ws] listening at ${wsUrl}`);
    console.log(`[dashboard] http://127.0.0.1:5173/?ws=${encodeURIComponent(wsUrl)}`);
  });

  await new Promise((resolve, reject) => {
    serial.open((error) => {
      if (error) {
        reject(error);
        return;
      }
      resolve();
    });
  });

  console.log(`[serial] opened ${serialPath} at ${baudRate} baud`);
  const statsTimer = setInterval(() => {
    console.log(`[bridge] clients=${wss.clients.size} chunks=${chunksForwarded} bytes=${bytesForwarded}`);
  }, 3000);

  const shutdown = () => {
    clearInterval(statsTimer);
    console.log("\n[bridge] shutting down");
    serial.close(() => {});
    wss.close(() => process.exit(0));
  };

  process.on("SIGINT", shutdown);
  process.on("SIGTERM", shutdown);
}

main().catch((error) => {
  console.error(error.message);
  process.exitCode = 1;
});
