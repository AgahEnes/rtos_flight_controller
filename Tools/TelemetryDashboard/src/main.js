import { createIcons, Activity, Gauge, LineChart, Play, Radio, RotateCcw, SlidersHorizontal, Usb, Waves, Wifi } from "lucide";
import "./styles.css";
import { VehicleScene } from "./scene.js";
import { StripChart } from "./charts.js";
import { TelemetryParser, packImuFrame } from "./telemetryProtocol.js";
import {
  attitudeSnapshot,
  createEstimator,
  makeSimulatedImuSample,
  resetEstimator,
  updateEstimatorFromImu
} from "./attitudeEstimator.js";

createIcons({
  icons: {
    Activity,
    Gauge,
    LineChart,
    Play,
    Radio,
    RotateCcw,
    SlidersHorizontal,
    Usb,
    Waves,
    Wifi
  }
});

const parser = new TelemetryParser();
const estimator = createEstimator();
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
  simButton: document.getElementById("toggleSimButton"),
  resetButton: document.getElementById("resetViewButton"),
  modePill: document.getElementById("modePill"),
  linkPill: document.getElementById("linkPill"),
  estimatorBadge: document.getElementById("estimatorBadge"),
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
  temp: document.getElementById("tempValue"),
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
let isSimulationRunning = false;
let simTimer = null;
let frameCount = 0;
let lastPacketAt = 0;
let latestSourceLabel = "IMU estimate";
let latestSnapshot = attitudeSnapshot(estimator, {
  timestampMs: 0,
  accel: { x: 0, y: 0, z: 9.80665 },
  gyro: { x: 0, y: 0, z: 0 },
  temperatureC: 0
});

function fmt(value, digits = 1) {
  return Number.isFinite(value) ? value.toFixed(digits) : "--";
}

function normalizeServoAngles(source) {
  if (Array.isArray(source?.servosDeg) && source.servosDeg.length >= 4) {
    return source.servosDeg.slice(0, 4).map((value) => Number.isFinite(value) ? value : 0);
  }

  if (Array.isArray(source?.finsDeg) && source.finsDeg.length >= 4) {
    return source.finsDeg.slice(0, 4).map((value) => Number.isFinite(value) ? value : 0);
  }

  const servoA = Number.isFinite(source?.servoA) ? source.servoA : Number.isFinite(source?.aDeg) ? source.aDeg : 0;
  const servoB = Number.isFinite(source?.servoB) ? source.servoB : Number.isFinite(source?.bDeg) ? source.bDeg : 0;
  const servoC = Number.isFinite(source?.servoC) ? source.servoC : Number.isFinite(source?.cDeg) ? source.cDeg : -servoA;
  const servoD = Number.isFinite(source?.servoD) ? source.servoD : Number.isFinite(source?.dDeg) ? source.dDeg : -servoB;

  return [servoA, servoB, servoC, servoD];
}

function setModeVisual(mode) {
  ui.modePill.dataset.mode = mode.toLowerCase();
  ui.modePill.textContent = mode;
  ui.modeText.textContent = mode.charAt(0) + mode.slice(1).toLowerCase();
}

function setLinkVisual(isLive) {
  ui.linkPill.dataset.live = isLive ? "true" : "false";
  ui.linkPill.textContent = isLive ? "LIVE" : "NO LINK";
}

function updateUi(snapshot, sourceLabel) {
  const linkLive = ((lastPacketAt > 0) && ((performance.now() - lastPacketAt) < 1600)) || isSimulationRunning;
  const mode = linkLive ? snapshot.mode : "ACQUIRING";

  setModeVisual(mode);
  setLinkVisual(linkLive);
  ui.estimatorBadge.textContent = sourceLabel;
  ui.roll.textContent = `${fmt(snapshot.rollDeg)} deg`;
  ui.pitch.textContent = `${fmt(snapshot.pitchDeg)} deg`;
  ui.yaw.textContent = `${fmt(snapshot.yawDeg)} deg`;
  ui.tilt.textContent = `${fmt(snapshot.tiltDeg)} deg`;
  ui.frames.textContent = String(frameCount);
  ui.packet.textContent = latestSnapshot.packetFormat === "extended-imu-sensors"
    ? "EXT 0x10"
    : latestSnapshot.packetFormat === "rtosfus-ascii"
      ? "RTOSFUS"
      : "IMU 0x10";
  ui.accelX.textContent = `${fmt(snapshot.accel.x, 2)} m/s2`;
  ui.accelY.textContent = `${fmt(snapshot.accel.y, 2)} m/s2`;
  ui.accelZ.textContent = `${fmt(snapshot.accel.z, 2)} m/s2`;
  ui.gyroX.textContent = `${fmt(snapshot.gyro.x, 2)} rad/s`;
  ui.gyroY.textContent = `${fmt(snapshot.gyro.y, 2)} rad/s`;
  ui.gyroZ.textContent = `${fmt(snapshot.gyro.z, 2)} rad/s`;
  ui.temp.textContent = `${fmt(snapshot.temperatureC, 2)} C`;
  ui.pressure.textContent = snapshot.pressurePa === null ? "-- Pa" : `${fmt(snapshot.pressurePa, 0)} Pa`;
  ui.altitude.textContent = snapshot.altitudeM === null ? "-- m" : `${fmt(snapshot.altitudeM, 2)} m`;
  const servosDeg = normalizeServoAngles(snapshot);
  ui.servoA.value = servosDeg[0];
  ui.servoB.value = servosDeg[1];
  ui.servoC.value = servosDeg[2];
  ui.servoD.value = servosDeg[3];
  ui.servoAText.textContent = `${fmt(servosDeg[0])} deg`;
  ui.servoBText.textContent = `${fmt(servosDeg[1])} deg`;
  ui.servoCText.textContent = `${fmt(servosDeg[2])} deg`;
  ui.servoDText.textContent = `${fmt(servosDeg[3])} deg`;
}

function ingestSample(sample, sourceLabel) {
  latestSnapshot = updateEstimatorFromImu(estimator, sample);
  latestSnapshot.packetFormat = sample.packetFormat;
  if (sample.attitudeDeg) {
    latestSnapshot.rollDeg = sample.attitudeDeg.rollDeg;
    latestSnapshot.pitchDeg = sample.attitudeDeg.pitchDeg;
    latestSnapshot.yawDeg = sample.attitudeDeg.yawDeg;
    latestSnapshot.tiltDeg = Math.sqrt(
      (latestSnapshot.rollDeg * latestSnapshot.rollDeg) +
      (latestSnapshot.pitchDeg * latestSnapshot.pitchDeg)
    );
  }
  if (sample.servo) {
    latestSnapshot.servosDeg = normalizeServoAngles(sample.servo);
    latestSnapshot.servoA = latestSnapshot.servosDeg[0];
    latestSnapshot.servoB = latestSnapshot.servosDeg[1];
    latestSnapshot.servoC = latestSnapshot.servosDeg[2];
    latestSnapshot.servoD = latestSnapshot.servosDeg[3];
  }
  if (sample.mode) {
    latestSnapshot.mode = sample.mode;
  }
  latestSourceLabel = sourceLabel;
  frameCount += 1;
  lastPacketAt = performance.now();
  scene.update(latestSnapshot);
  attitudeChart.push(sample.timestampMs, {
    roll: latestSnapshot.rollDeg,
    pitch: latestSnapshot.pitchDeg,
    yaw: latestSnapshot.yawDeg
  });
  imuChart.push(sample.timestampMs, {
    ax: sample.accel.x,
    ay: sample.accel.y,
    az: sample.accel.z
  });
  updateUi(latestSnapshot, sourceLabel);
}

async function connectSerial() {
  if (!("serial" in navigator)) {
    ui.estimatorBadge.textContent = "Chrome Web Serial required";
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
    ui.estimatorBadge.textContent = "WiFi URL missing";
    return;
  }

  wifiSocket = new WebSocket(url);
  wifiSocket.binaryType = "arraybuffer";
  ui.estimatorBadge.textContent = "WiFi connecting";

  wifiSocket.addEventListener("open", () => {
    isWifiRunning = true;
    ui.wifiButton.classList.add("armed");
    ui.wifiButton.querySelector("span").textContent = "Stop";
    ui.estimatorBadge.textContent = "WiFi telemetry";
  });

  wifiSocket.addEventListener("message", (event) => {
    if (event.data instanceof ArrayBuffer) {
      parser.push(new Uint8Array(event.data)).forEach((frame) => ingestSample(frame, "WiFi telemetry"));
    } else if (event.data instanceof Blob) {
      event.data.arrayBuffer().then((buffer) => {
        parser.push(new Uint8Array(buffer)).forEach((frame) => ingestSample(frame, "WiFi telemetry"));
      });
    }
  });

  wifiSocket.addEventListener("close", () => {
    stopWifi(false);
    ui.estimatorBadge.textContent = "WiFi disconnected";
  });

  wifiSocket.addEventListener("error", () => {
    ui.estimatorBadge.textContent = "WiFi connection failed";
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
        parser.push(value).forEach((frame) => ingestSample(frame, "Live IMU estimate"));
      }
    }
  } catch (error) {
    ui.estimatorBadge.textContent = "Serial read stopped";
  } finally {
    isSerialRunning = false;
    ui.connectButton.classList.remove("armed");
    ui.connectButton.querySelector("span").textContent = "Serial";
  }
}

function toggleSimulation() {
  if (isSimulationRunning) {
    clearInterval(simTimer);
    simTimer = null;
    isSimulationRunning = false;
    ui.simButton.classList.remove("armed");
    ui.simButton.querySelector("span").textContent = "Sim";
    return;
  }

  const startedAt = performance.now();
  let sequence = 0;
  isSimulationRunning = true;
  ui.simButton.classList.add("armed");
  ui.simButton.querySelector("span").textContent = "Stop";
  simTimer = setInterval(() => {
    const sample = makeSimulatedImuSample(performance.now() - startedAt);
    const frame = packImuFrame(sample, sequence);
    sequence = (sequence + 1) & 0xff;
    parser.push(frame).forEach((packet) => {
      packet.pressurePa = sample.pressurePa;
      packet.altitudeM = sample.altitudeM;
      ingestSample(packet, "Simulation");
    });
  }, 50);
}

function resetDashboard() {
  parser.reset();
  resetEstimator(estimator);
  frameCount = 0;
  lastPacketAt = 0;
  latestSourceLabel = "IMU estimate";
  latestSnapshot = attitudeSnapshot(estimator, {
    timestampMs: 0,
    accel: { x: 0, y: 0, z: 9.80665 },
    gyro: { x: 0, y: 0, z: 0 },
    temperatureC: 0
  });
  scene.update(latestSnapshot);
  updateUi(latestSnapshot, "IMU estimate");
}

ui.connectButton.addEventListener("click", () => {
  connectSerial().catch(() => {
    ui.estimatorBadge.textContent = "Serial connection failed";
  });
});
ui.wifiButton.addEventListener("click", connectWifi);
ui.simButton.addEventListener("click", toggleSimulation);
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
