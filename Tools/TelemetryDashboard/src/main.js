import { createIcons, Activity, Gauge, LineChart, Radio, RotateCcw, SlidersHorizontal, Usb, Waves, Wifi } from "lucide";
import "./styles.css";
import { VehicleScene } from "./scene.js";
import { StripChart } from "./charts.js";
import { TelemetryParser } from "./telemetryProtocol.js";

createIcons({
  icons: {
    Activity,
    Gauge,
    LineChart,
    Radio,
    RotateCcw,
    SlidersHorizontal,
    Usb,
    Waves,
    Wifi
  }
});

const parser = new TelemetryParser();
const scene = new VehicleScene(document.getElementById("vehicleCanvas"));
const attitudeChart = new StripChart(document.getElementById("attitudeChart"), [
  { key: "roll", color: "#eb6f55" },
  { key: "pitch", color: "#4fc3a1" },
  { key: "yaw", color: "#e2b84d" }
]);
const imuChart = new StripChart(document.getElementById("imuChart"), [
  { key: "ax", color: "#eb6f55" },
  { key: "ay", color: "#4fc3a1" },
  { key: "az", color: "#6fa8dc" }
]);

const ui = {
  connectButton: document.getElementById("connectSerialButton"),
  wifiButton: document.getElementById("connectWifiButton"),
  wifiUrlInput: document.getElementById("wifiUrlInput"),
  resetButton: document.getElementById("resetViewButton"),
  modePill: document.getElementById("modePill"),
  linkPill: document.getElementById("linkPill"),
  sourceBadge: document.getElementById("sourceBadge"),
  modeText: document.getElementById("modeText"),
  roll: document.getElementById("rollValue"),
  pitch: document.getElementById("pitchValue"),
  yaw: document.getElementById("yawValue"),
  tilt: document.getElementById("tiltValue"),
  frames: document.getElementById("frameCountValue"),
  packet: document.getElementById("packetTypeValue"),
  accelX: document.getElementById("accelXValue"),
  accelY: document.getElementById("accelYValue"),
  accelZ: document.getElementById("accelZValue"),
  gyroX: document.getElementById("gyroXValue"),
  gyroY: document.getElementById("gyroYValue"),
  gyroZ: document.getElementById("gyroZValue"),
  imuTemp: document.getElementById("imuTempValue"),
  baroTemp: document.getElementById("baroTempValue"),
  pressure: document.getElementById("pressureValue"),
  altitude: document.getElementById("altitudeValue"),
  servoA: document.getElementById("servoAValue"),
  servoB: document.getElementById("servoBValue"),
  servoC: document.getElementById("servoCValue"),
  servoD: document.getElementById("servoDValue"),
  servoAText: document.getElementById("servoAText"),
  servoBText: document.getElementById("servoBText"),
  servoCText: document.getElementById("servoCText"),
  servoDText: document.getElementById("servoDText")
};

const initialWsUrl = new URLSearchParams(window.location.search).get("ws");
if (initialWsUrl !== null && initialWsUrl.trim().length > 0) {
  ui.wifiUrlInput.value = initialWsUrl.trim();
}

let serialPort = null;
let serialReader = null;
let wifiSocket = null;
let isSerialRunning = false;
let isWifiRunning = false;
let frameCount = 0;
let lastPacketAt = 0;
let latestSourceLabel = "No telemetry";
let latestSnapshot = makeInitialSnapshot();

function makeInitialSnapshot() {
  return {
    packetFormat: "none",
    packetLabel: "--",
    timestampMs: 0,
    mode: "ACQUIRING",
    rollDeg: 0,
    pitchDeg: 0,
    yawDeg: 0,
    tiltDeg: 0,
    accel: { x: 0, y: 0, z: 0 },
    gyro: { x: 0, y: 0, z: 0 },
    imuTemperatureC: 0,
    barometerTemperatureC: 0,
    pressurePa: null,
    altitudeM: null,
    servosDeg: [0, 0, 0, 0]
  };
}

function fmt(value, digits = 1) {
  return Number.isFinite(value) ? value.toFixed(digits) : "--";
}

function displayModeText(mode) {
  const text = String(mode ?? "ACQUIRING");
  return text.length > 0 ? text.charAt(0).toUpperCase() + text.slice(1).toLowerCase() : "Acquiring";
}

function readServoAngles(source) {
  if (Array.isArray(source?.servosDeg) && source.servosDeg.length >= 4) {
    return source.servosDeg.slice(0, 4).map((value) => Number.isFinite(value) ? value : 0);
  }

  if (Array.isArray(source?.finsDeg) && source.finsDeg.length >= 4) {
    return source.finsDeg.slice(0, 4).map((value) => Number.isFinite(value) ? value : 0);
  }

  return null;
}

function setModeVisual(mode) {
  ui.modePill.dataset.mode = String(mode ?? "ACQUIRING").toLowerCase();
  ui.modePill.textContent = String(mode ?? "ACQUIRING").toUpperCase();
  ui.modeText.textContent = displayModeText(mode);
}

function setLinkVisual(isLive) {
  ui.linkPill.dataset.live = isLive ? "true" : "false";
  ui.linkPill.textContent = isLive ? "LIVE" : "NO LINK";
}

function updateUi(snapshot, sourceLabel) {
  const linkLive = (lastPacketAt > 0) && ((performance.now() - lastPacketAt) < 1600);
  const servosDeg = Array.isArray(snapshot.servosDeg) ? snapshot.servosDeg : [0, 0, 0, 0];

  setModeVisual(snapshot.mode);
  setLinkVisual(linkLive);
  ui.sourceBadge.textContent = sourceLabel;
  ui.roll.textContent = `${fmt(snapshot.rollDeg)} deg`;
  ui.pitch.textContent = `${fmt(snapshot.pitchDeg)} deg`;
  ui.yaw.textContent = `${fmt(snapshot.yawDeg)} deg`;
  ui.tilt.textContent = `${fmt(snapshot.tiltDeg)} deg`;
  ui.frames.textContent = String(frameCount);
  ui.packet.textContent = snapshot.packetLabel ?? "--";
  ui.accelX.textContent = `${fmt(snapshot.accel.x, 2)} m/s2`;
  ui.accelY.textContent = `${fmt(snapshot.accel.y, 2)} m/s2`;
  ui.accelZ.textContent = `${fmt(snapshot.accel.z, 2)} m/s2`;
  ui.gyroX.textContent = `${fmt(snapshot.gyro.x, 2)} rad/s`;
  ui.gyroY.textContent = `${fmt(snapshot.gyro.y, 2)} rad/s`;
  ui.gyroZ.textContent = `${fmt(snapshot.gyro.z, 2)} rad/s`;
  ui.imuTemp.textContent = `${fmt(snapshot.imuTemperatureC, 2)} C`;
  ui.baroTemp.textContent = `${fmt(snapshot.barometerTemperatureC, 2)} C`;
  ui.pressure.textContent = snapshot.pressurePa === null ? "-- Pa" : `${fmt(snapshot.pressurePa, 0)} Pa`;
  ui.altitude.textContent = snapshot.altitudeM === null ? "-- m" : `${fmt(snapshot.altitudeM, 2)} m`;
  ui.servoA.value = servosDeg[0];
  ui.servoB.value = servosDeg[1];
  ui.servoC.value = servosDeg[2];
  ui.servoD.value = servosDeg[3];
  ui.servoAText.textContent = `${fmt(servosDeg[0])} deg`;
  ui.servoBText.textContent = `${fmt(servosDeg[1])} deg`;
  ui.servoCText.textContent = `${fmt(servosDeg[2])} deg`;
  ui.servoDText.textContent = `${fmt(servosDeg[3])} deg`;
}

function ingestFrame(frame, sourceLabel) {
  latestSnapshot.packetFormat = frame.packetFormat ?? latestSnapshot.packetFormat;
  latestSnapshot.packetLabel = frame.packetLabel ?? latestSnapshot.packetLabel;
  if (Number.isFinite(frame.timestampMs)) {
    latestSnapshot.timestampMs = frame.timestampMs;
  }
  if (frame.accel) {
    latestSnapshot.accel = { ...latestSnapshot.accel, ...frame.accel };
  }
  if (frame.gyro) {
    latestSnapshot.gyro = { ...latestSnapshot.gyro, ...frame.gyro };
  }
  if (Number.isFinite(frame.temperatureC)) {
    latestSnapshot.imuTemperatureC = frame.temperatureC;
  }
  if (Number.isFinite(frame.barometerTemperatureC)) {
    latestSnapshot.barometerTemperatureC = frame.barometerTemperatureC;
  }
  if (Number.isFinite(frame.pressurePa)) {
    latestSnapshot.pressurePa = frame.pressurePa;
  }
  if (Number.isFinite(frame.altitudeM)) {
    latestSnapshot.altitudeM = frame.altitudeM;
  }
  if (frame.attitudeDeg) {
    latestSnapshot.rollDeg = Number.isFinite(frame.attitudeDeg.rollDeg) ? frame.attitudeDeg.rollDeg : latestSnapshot.rollDeg;
    latestSnapshot.pitchDeg = Number.isFinite(frame.attitudeDeg.pitchDeg) ? frame.attitudeDeg.pitchDeg : latestSnapshot.pitchDeg;
    latestSnapshot.yawDeg = Number.isFinite(frame.attitudeDeg.yawDeg) ? frame.attitudeDeg.yawDeg : latestSnapshot.yawDeg;
  }
  if (Number.isFinite(frame.tiltDeg)) {
    latestSnapshot.tiltDeg = frame.tiltDeg;
  }
  if (frame.mode !== null && frame.mode !== undefined && String(frame.mode).trim().length > 0) {
    latestSnapshot.mode = String(frame.mode).trim();
  }

  const servosDeg = readServoAngles(frame.servo);
  if (servosDeg !== null) {
    latestSnapshot.servosDeg = servosDeg;
  }

  latestSourceLabel = sourceLabel;
  frameCount += 1;
  lastPacketAt = performance.now();
  scene.update(latestSnapshot);

  if (frame.attitudeDeg) {
    attitudeChart.push(latestSnapshot.timestampMs, {
      roll: latestSnapshot.rollDeg,
      pitch: latestSnapshot.pitchDeg,
      yaw: latestSnapshot.yawDeg
    });
  }

  if (frame.accel) {
    imuChart.push(latestSnapshot.timestampMs, {
      ax: latestSnapshot.accel.x,
      ay: latestSnapshot.accel.y,
      az: latestSnapshot.accel.z
    });
  }

  updateUi(latestSnapshot, sourceLabel);
}

async function connectSerial() {
  if (!("serial" in navigator)) {
    ui.sourceBadge.textContent = "Chrome Web Serial required";
    return;
  }

  if (isSerialRunning) {
    await stopSerial();
    return;
  }

  serialPort = await navigator.serial.requestPort();
  await serialPort.open({ baudRate: 115200 });
  serialReader = serialPort.readable.getReader();
  isSerialRunning = true;
  ui.connectButton.classList.add("armed");
  ui.connectButton.querySelector("span").textContent = "Stop";
  readSerialLoop();
}

async function stopSerial() {
  isSerialRunning = false;
  ui.connectButton.classList.remove("armed");
  ui.connectButton.querySelector("span").textContent = "Serial";
  if (serialReader) {
    await serialReader.cancel().catch(() => {});
    serialReader.releaseLock();
    serialReader = null;
  }
  if (serialPort) {
    await serialPort.close().catch(() => {});
    serialPort = null;
  }
}

function connectWifi() {
  if (isWifiRunning) {
    stopWifi();
    return;
  }

  const url = ui.wifiUrlInput.value.trim();
  if (url.length === 0) {
    ui.sourceBadge.textContent = "WiFi URL missing";
    return;
  }

  wifiSocket = new WebSocket(url);
  wifiSocket.binaryType = "arraybuffer";
  ui.sourceBadge.textContent = "WiFi connecting";

  wifiSocket.addEventListener("open", () => {
    isWifiRunning = true;
    ui.wifiButton.classList.add("armed");
    ui.wifiButton.querySelector("span").textContent = "Stop";
    ui.sourceBadge.textContent = "WiFi telemetry";
  });

  wifiSocket.addEventListener("message", (event) => {
    if (event.data instanceof ArrayBuffer) {
      parser.push(new Uint8Array(event.data)).forEach((frame) => ingestFrame(frame, "WiFi telemetry"));
    } else if (event.data instanceof Blob) {
      event.data.arrayBuffer().then((buffer) => {
        parser.push(new Uint8Array(buffer)).forEach((frame) => ingestFrame(frame, "WiFi telemetry"));
      });
    }
  });

  wifiSocket.addEventListener("close", () => {
    stopWifi(false);
    ui.sourceBadge.textContent = "WiFi disconnected";
  });

  wifiSocket.addEventListener("error", () => {
    ui.sourceBadge.textContent = "WiFi connection failed";
  });
}

function stopWifi(shouldClose = true) {
  const socket = wifiSocket;
  wifiSocket = null;
  isWifiRunning = false;
  ui.wifiButton.classList.remove("armed");
  ui.wifiButton.querySelector("span").textContent = "WiFi";
  if (shouldClose && socket !== null) {
    socket.close();
  }
}

async function readSerialLoop() {
  try {
    while (isSerialRunning && serialReader) {
      const { value, done } = await serialReader.read();
      if (done) break;
      if (value) {
        parser.push(value).forEach((frame) => ingestFrame(frame, "Serial telemetry"));
      }
    }
  } catch (error) {
    ui.sourceBadge.textContent = "Serial read stopped";
  } finally {
    isSerialRunning = false;
    ui.connectButton.classList.remove("armed");
    ui.connectButton.querySelector("span").textContent = "Serial";
  }
}

function resetDashboard() {
  parser.reset();
  frameCount = 0;
  lastPacketAt = 0;
  latestSourceLabel = "No telemetry";
  latestSnapshot = makeInitialSnapshot();
  attitudeChart.clear();
  imuChart.clear();
  scene.update(latestSnapshot);
  updateUi(latestSnapshot, latestSourceLabel);
}

ui.connectButton.addEventListener("click", () => {
  connectSerial().catch(() => {
    ui.sourceBadge.textContent = "Serial connection failed";
  });
});
ui.wifiButton.addEventListener("click", connectWifi);
ui.resetButton.addEventListener("click", resetDashboard);

const initialConnectionMode = new URLSearchParams(window.location.search).get("connect");
if (initialConnectionMode === "wifi") {
  window.setTimeout(connectWifi, 0);
}

function animate(timeMs) {
  scene.render(timeMs);
  attitudeChart.draw();
  imuChart.draw();
  updateUi(latestSnapshot, latestSourceLabel);
  requestAnimationFrame(animate);
}

resetDashboard();
requestAnimationFrame(animate);
