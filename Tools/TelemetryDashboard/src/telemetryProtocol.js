export const LEGACY_SYNC_0 = 0xa5;
export const LEGACY_SYNC_1 = 0x5a;
export const EXTENDED_SYNC_0 = 0xaa;
export const EXTENDED_SYNC_1 = 0x55;
export const MSG_ID_LEGACY_IMU = 0x10;
export const MSG_ID_IMU_VEHICLE_STATE = 0x12;
export const MSG_ID_IMU_CALIBRATION = 0x81;
export const LEGACY_IMU_FRAME_LENGTH = 38;
export const VEHICLE_STATE_FRAME_LENGTH = 64;
export const CALIBRATION_FRAME_LENGTH = 43;
export const EXTENDED_FRAME_LENGTH = 71;
export const EXTENDED_PAYLOAD_LENGTH = 65;
export const ASCII_PACKET_START = 0x24;
export const ASCII_LINE_FEED = 0x0a;
export const FLIGHT_MODE_PREFLIGHT = 0;
export const FLIGHT_MODE_READY_FOR_IGNITION = 1;
export const FLIGHT_MODE_BOOST = 2;
export const FLIGHT_MODE_STABILIZE = 3;
export const FLIGHT_MODE_FAILSAFE = 4;

const RAD_TO_DEG = 180 / Math.PI;

export function crc16CcittFalse(bytes) {
  let crc = 0xffff;
  for (let i = 0; i < bytes.length; i += 1) {
    crc ^= bytes[i] << 8;
    for (let bit = 0; bit < 8; bit += 1) {
      crc = (crc & 0x8000) !== 0 ? ((crc << 1) ^ 0x1021) : (crc << 1);
      crc &= 0xffff;
    }
  }
  return crc;
}

function readFloat32Le(view, offset) {
  return view.getFloat32(offset, true);
}

function readFloat32FromDecimalBytes(byte0, byte1, byte2, byte3) {
  const buffer = new ArrayBuffer(4);
  const bytes = new Uint8Array(buffer);
  bytes[0] = byte0;
  bytes[1] = byte1;
  bytes[2] = byte2;
  bytes[3] = byte3;
  return new DataView(buffer).getFloat32(0, true);
}

function parseFiniteFloat(value, fallback = 0) {
  const parsed = Number.parseFloat(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function parseFiniteInt(value, fallback = 0) {
  const parsed = Number.parseInt(value, 10);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function parseOptionalFloat(value) {
  const parsed = Number.parseFloat(value);
  return Number.isFinite(parsed) ? parsed : null;
}

function isDecimalByteField(value) {
  const text = String(value ?? "").trim();
  if (!/^\d{1,3}$/.test(text)) {
    return false;
  }

  const parsed = Number.parseInt(text, 10);
  return parsed >= 0 && parsed <= 255;
}

function normalizeServoLabel(label) {
  const normalized = String(label ?? "").trim().toUpperCase().replace(/[^A-Z0-9]/g, "");
  if (normalized === "SA" || normalized === "SERVOA" || normalized === "FINA" || normalized === "CANARDA" || normalized === "S1") return 0;
  if (normalized === "SB" || normalized === "SERVOB" || normalized === "FINB" || normalized === "CANARDB" || normalized === "S2") return 1;
  if (normalized === "SC" || normalized === "SERVOC" || normalized === "FINC" || normalized === "CANARDC" || normalized === "S3") return 2;
  if (normalized === "SD" || normalized === "SERVOD" || normalized === "FIND" || normalized === "CANARDD" || normalized === "S4") return 3;
  return -1;
}

function parseServoFromNamedFields(fields) {
  const finsDeg = [null, null, null, null];

  for (const field of fields) {
    const match = String(field).match(/^\s*([A-Za-z0-9_ -]+)\s*[:=]\s*(-?\d+(?:\.\d+)?)\s*$/);
    if (match === null) {
      continue;
    }

    const index = normalizeServoLabel(match[1]);
    const value = parseOptionalFloat(match[2]);
    if (index >= 0 && value !== null) {
      finsDeg[index] = value;
    }
  }

  return finsDeg.some((value) => value !== null) ? { finsDeg } : null;
}

function parseServoFromNumericTail(fields) {
  if (fields.length < 29) {
    return null;
  }

  const values = fields.slice(25, 29).map((field) => parseOptionalFloat(field));
  if (values.some((value) => value === null)) {
    return null;
  }

  return { finsDeg: values };
}

function parseServoFromAsciiFields(fields) {
  return parseServoFromNamedFields(fields) ?? parseServoFromNumericTail(fields);
}

function parseFirmwareMode(modeText) {
  const mode = String(modeText ?? "").trim();
  return mode.length > 0 ? mode : null;
}

function flightModeToUi(modeByte) {
  switch (modeByte) {
    case FLIGHT_MODE_PREFLIGHT:
      return { mode: "PREFLIGHT", modePill: "preflight" };
    case FLIGHT_MODE_READY_FOR_IGNITION:
      return { mode: "READY_FOR_IGNITION", modePill: "ready_for_ignition" };
    case FLIGHT_MODE_BOOST:
      return { mode: "BOOST", modePill: "boost" };
    case FLIGHT_MODE_STABILIZE:
      return { mode: "STABILIZE", modePill: "stabilize" };
    case FLIGHT_MODE_FAILSAFE:
      return { mode: "FAILSAFE", modePill: "failsafe" };
    default:
      return { mode: "UNKNOWN", modePill: null };
  }
}

function parseAsciiChecksum(lineBytes, starIndex) {
  let checksum = 0;
  for (let i = 1; i < starIndex; i += 1) {
    checksum ^= lineBytes[i];
  }
  return checksum & 0xff;
}

function parseRtosFusionLine(lineBytes) {
  const cleanBytes = lineBytes[lineBytes.length - 1] === ASCII_LINE_FEED
    ? lineBytes.slice(0, lineBytes.length - 1)
    : lineBytes;
  const trimmedBytes = cleanBytes[cleanBytes.length - 1] === 0x0d
    ? cleanBytes.slice(0, cleanBytes.length - 1)
    : cleanBytes;
  const starIndex = trimmedBytes.indexOf(0x2a);

  if (trimmedBytes[0] !== ASCII_PACKET_START || starIndex < 0 || starIndex + 2 >= trimmedBytes.length) {
    return null;
  }

  const receivedChecksum = Number.parseInt(String.fromCharCode(trimmedBytes[starIndex + 1], trimmedBytes[starIndex + 2]), 16);
  if (!Number.isFinite(receivedChecksum) || parseAsciiChecksum(trimmedBytes, starIndex) !== receivedChecksum) {
    return null;
  }

  const body = String.fromCharCode(...trimmedBytes.slice(1, starIndex));
  const fields = body.split(",");
  if (fields[0] !== "RTOSFUS" || fields.length < 17) {
    return null;
  }

  const mode = parseFirmwareMode(fields[16]);
  const servo = parseServoFromAsciiFields(fields);
  const hasPressureBytes = fields.length >= 24 && fields.slice(20, 24).every(isDecimalByteField);
  const pressureHpa = hasPressureBytes
    ? readFloat32FromDecimalBytes(
      parseFiniteInt(fields[20], 0),
      parseFiniteInt(fields[21], 0),
      parseFiniteInt(fields[22], 0),
      parseFiniteInt(fields[23], 0)
    )
    : null;

  return {
    type: "fusion",
    packetFormat: "rtosfus-ascii",
    packetLabel: "RTOSFUS",
    messageId: "RTOSFUS",
    sequence: null,
    timestampMs: parseFiniteInt(fields[1], 0),
    accel: {
      x: parseFiniteFloat(fields[10], 0),
      y: parseFiniteFloat(fields[11], 0),
      z: parseFiniteFloat(fields[12], 0)
    },
    gyro: {
      x: parseFiniteFloat(fields[7], 0),
      y: parseFiniteFloat(fields[8], 0),
      z: parseFiniteFloat(fields[9], 0)
    },
    temperatureC: parseFiniteFloat(fields[15], 0),
    pressureHpa,
    pressurePa: pressureHpa === null ? null : pressureHpa * 100.0,
    altitudeM: parseOptionalFloat(fields[14]),
    fixType: fields.length > 24 ? parseFiniteInt(fields[24], 0) : 0,
    attitudeDeg: {
      rollDeg: parseFiniteFloat(fields[4], 0),
      pitchDeg: parseFiniteFloat(fields[5], 0),
      yawDeg: parseFiniteFloat(fields[6], 0)
    },
    servo,
    mode,
    rawFields: fields
  };
}

function parseLegacyImuFrame(frame) {
  const view = new DataView(frame.buffer, frame.byteOffset, frame.byteLength);
  return {
    type: "imu",
    packetFormat: "legacy-imu",
    packetLabel: "IMU 0x10",
    messageId: MSG_ID_LEGACY_IMU,
    sequence: frame[3],
    timestampMs: view.getUint32(4, true),
    accel: {
      x: readFloat32Le(view, 8),
      y: readFloat32Le(view, 12),
      z: readFloat32Le(view, 16)
    },
    gyro: {
      x: readFloat32Le(view, 20),
      y: readFloat32Le(view, 24),
      z: readFloat32Le(view, 28)
    },
    temperatureC: readFloat32Le(view, 32)
  };
}

function parseVehicleStateFrame(frame) {
  const view = new DataView(frame.buffer, frame.byteOffset, frame.byteLength);
  const parsedMode = flightModeToUi(frame[61]);
  return {
    type: "imu-vehicle-state",
    packetFormat: "agah-imu-vehicle-state",
    packetLabel: "STATE 0x12",
    messageId: MSG_ID_IMU_VEHICLE_STATE,
    sequence: frame[3],
    timestampMs: view.getUint32(4, true),
    accel: {
      x: readFloat32Le(view, 8),
      y: readFloat32Le(view, 12),
      z: readFloat32Le(view, 16)
    },
    gyro: {
      x: readFloat32Le(view, 20),
      y: readFloat32Le(view, 24),
      z: readFloat32Le(view, 28)
    },
    temperatureC: readFloat32Le(view, 32),
    attitudeDeg: {
      rollDeg: readFloat32Le(view, 36) * RAD_TO_DEG,
      pitchDeg: readFloat32Le(view, 40) * RAD_TO_DEG,
      yawDeg: readFloat32Le(view, 44) * RAD_TO_DEG
    },
    attitudeRateRadS: {
      roll: readFloat32Le(view, 48),
      pitch: readFloat32Le(view, 52),
      yaw: readFloat32Le(view, 56)
    },
    isEstimated: frame[60] !== 0,
    mode: parsedMode.mode,
    modePill: parsedMode.modePill
  };
}

function parseCalibrationFrame(frame) {
  const view = new DataView(frame.buffer, frame.byteOffset, frame.byteLength);
  return {
    type: "imu-calibration",
    packetFormat: "imu-calibration",
    packetLabel: "CAL 0x81",
    messageId: MSG_ID_IMU_CALIBRATION,
    sequence: frame[3],
    timestampMs: view.getUint32(4, true),
    calibration: {
      accelBias: {
        x: readFloat32Le(view, 8),
        y: readFloat32Le(view, 12),
        z: readFloat32Le(view, 16)
      },
      gyroBias: {
        x: readFloat32Le(view, 20),
        y: readFloat32Le(view, 24),
        z: readFloat32Le(view, 28)
      },
      timestampMs: view.getUint32(32, true),
      updateCounter: view.getUint32(36, true),
      isValid: frame[40] !== 0
    }
  };
}

function parseExtendedImuFrame(frame) {
  const view = new DataView(frame.buffer, frame.byteOffset, frame.byteLength);
  return {
    type: "imu",
    packetFormat: "extended-imu-sensors",
    packetLabel: "EXT 0x10",
    messageId: frame[2],
    payloadLength: frame[3],
    sequence: null,
    timestampMs: view.getUint32(4, true),
    accel: {
      x: readFloat32Le(view, 8),
      y: readFloat32Le(view, 12),
      z: readFloat32Le(view, 16)
    },
    gyro: {
      x: readFloat32Le(view, 20),
      y: readFloat32Le(view, 24),
      z: readFloat32Le(view, 28)
    },
    temperatureC: readFloat32Le(view, 32),
    magnetometerMgauss: {
      x: readFloat32Le(view, 36),
      y: readFloat32Le(view, 40),
      z: readFloat32Le(view, 44)
    },
    pressureHpa: readFloat32Le(view, 48),
    pressurePa: readFloat32Le(view, 48) * 100.0,
    latitudeDeg: readFloat32Le(view, 52),
    longitudeDeg: readFloat32Le(view, 56),
    gnssAltitudeM: readFloat32Le(view, 60),
    altitudeM: readFloat32Le(view, 60),
    fixType: frame[64],
    reserved: {
      byte65: frame[65],
      byte66: frame[66],
      byte67: frame[67],
      byte68: frame[68]
    }
  };
}

export class TelemetryParser {
  constructor() {
    this.buffer = new Uint8Array(0);
    this.droppedBytes = 0;
    this.crcErrors = 0;
  }

  reset() {
    this.buffer = new Uint8Array(0);
    this.droppedBytes = 0;
    this.crcErrors = 0;
  }

  push(chunk) {
    const merged = new Uint8Array(this.buffer.length + chunk.length);
    merged.set(this.buffer, 0);
    merged.set(chunk, this.buffer.length);
    this.buffer = merged;
    return this.extractFrames();
  }

  extractFrames() {
    const frames = [];

    while (this.buffer.length >= 2) {
      const syncIndex = this.findSync();
      if (syncIndex < 0) {
        this.droppedBytes += this.buffer.length > 0 && this.isPossiblePartialSync(this.buffer[this.buffer.length - 1])
          ? this.buffer.length - 1
          : this.buffer.length;
        this.buffer = this.buffer.length > 0 && this.isPossiblePartialSync(this.buffer[this.buffer.length - 1])
          ? this.buffer.slice(this.buffer.length - 1)
          : new Uint8Array(0);
        break;
      }

      if (syncIndex > 0) {
        this.droppedBytes += syncIndex;
        this.buffer = this.buffer.slice(syncIndex);
      }

      if (this.buffer.length < 3) {
        break;
      }

      if (this.buffer[0] === ASCII_PACKET_START) {
        const lineFeedIndex = this.buffer.indexOf(ASCII_LINE_FEED);
        if (lineFeedIndex < 0) {
          break;
        }
        const lineBytes = this.buffer.slice(0, lineFeedIndex + 1);
        this.buffer = this.buffer.slice(lineFeedIndex + 1);
        const packet = parseRtosFusionLine(lineBytes);
        if (packet !== null) {
          frames.push(packet);
        } else {
          this.crcErrors += 1;
        }
        continue;
      }

      const frameLength = this.frameLengthForCurrentHeader();
      if (frameLength === 0) {
        this.droppedBytes += 1;
        this.buffer = this.buffer.slice(1);
        continue;
      }

      if (this.buffer.length < frameLength) {
        break;
      }

      const frame = this.buffer.slice(0, frameLength);
      this.buffer = this.buffer.slice(frameLength);

      const receivedCrc = frame[frameLength - 2] | (frame[frameLength - 1] << 8);
      const calculatedCrc = crc16CcittFalse(frame.slice(0, frameLength - 2));
      if (receivedCrc !== calculatedCrc) {
        this.crcErrors += 1;
        continue;
      }

      if (this.isExtendedFrame(frame)) {
        frames.push(parseExtendedImuFrame(frame));
      } else if (frame[2] === MSG_ID_IMU_VEHICLE_STATE) {
        frames.push(parseVehicleStateFrame(frame));
      } else if (frame[2] === MSG_ID_IMU_CALIBRATION) {
        frames.push(parseCalibrationFrame(frame));
      } else if (frame[2] === MSG_ID_LEGACY_IMU) {
        frames.push(parseLegacyImuFrame(frame));
      }
    }

    return frames;
  }

  findSync() {
    for (let i = 0; i < this.buffer.length - 1; i += 1) {
      if (this.isKnownSyncPair(this.buffer[i], this.buffer[i + 1])) {
        return i;
      }
      if (this.buffer[i] === ASCII_PACKET_START) {
        return i;
      }
    }
    return -1;
  }

  isKnownSyncPair(firstByte, secondByte) {
    return (
      (firstByte === LEGACY_SYNC_0 && secondByte === LEGACY_SYNC_1) ||
      (firstByte === EXTENDED_SYNC_0 && secondByte === EXTENDED_SYNC_1)
    );
  }

  isPossiblePartialSync(byte) {
    return byte === LEGACY_SYNC_0 || byte === EXTENDED_SYNC_0 || byte === ASCII_PACKET_START;
  }

  isExtendedFrame(frame) {
    return (
      frame.length === EXTENDED_FRAME_LENGTH &&
      frame[0] === EXTENDED_SYNC_0 &&
      frame[1] === EXTENDED_SYNC_1 &&
      frame[2] === MSG_ID_LEGACY_IMU &&
      frame[3] === EXTENDED_PAYLOAD_LENGTH
    );
  }

  frameLengthForCurrentHeader() {
    if (this.buffer[0] === LEGACY_SYNC_0 && this.buffer[1] === LEGACY_SYNC_1) {
      if (this.buffer[2] === MSG_ID_IMU_VEHICLE_STATE) {
        return VEHICLE_STATE_FRAME_LENGTH;
      }
      if (this.buffer[2] === MSG_ID_IMU_CALIBRATION) {
        return CALIBRATION_FRAME_LENGTH;
      }
      if (this.buffer[2] === MSG_ID_LEGACY_IMU) {
        return LEGACY_IMU_FRAME_LENGTH;
      }
    }
    if (this.buffer.length >= 4 &&
        this.buffer[0] === EXTENDED_SYNC_0 &&
        this.buffer[1] === EXTENDED_SYNC_1 &&
        this.buffer[2] === MSG_ID_LEGACY_IMU &&
        this.buffer[3] === EXTENDED_PAYLOAD_LENGTH) {
      return EXTENDED_FRAME_LENGTH;
    }
    return 0;
  }
}
