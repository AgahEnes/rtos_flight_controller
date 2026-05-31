export const LEGACY_SYNC_0 = 0xa5;
export const LEGACY_SYNC_1 = 0x5a;
export const EXTENDED_SYNC_0 = 0xaa;
export const EXTENDED_SYNC_1 = 0x55;
export const TELEMETRY_SYNC_0 = LEGACY_SYNC_0;
export const TELEMETRY_SYNC_1 = LEGACY_SYNC_1;
export const MSG_ID_IMU = 0x10;
export const IMU_FRAME_LENGTH = 38;
export const EXTENDED_FRAME_LENGTH = 71;
export const EXTENDED_PAYLOAD_LENGTH = 65;
export const ASCII_PACKET_START = 0x24;
export const ASCII_LINE_FEED = 0x0a;

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

function normalizeFirmwareMode(modeText) {
  const mode = String(modeText ?? "").trim().toUpperCase();
  if (mode === "BALANCED" || mode === "DENGEDE") return "BALANCED";
  if (mode === "RECOVERING" || mode === "ACQUIRING" || mode === "DEGRADED") return mode;
  if (mode === "FAILURE" || mode === "ERROR" || mode === "FAULT") return "FAILURE";
  return mode.length > 0 ? mode : "ACQUIRING";
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

  const pressureHpa = fields.length >= 24
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
    altitudeM: parseFiniteFloat(fields[14], 0),
    fixType: fields.length > 24 ? parseFiniteInt(fields[24], 0) : 0,
    attitudeDeg: {
      rollDeg: parseFiniteFloat(fields[4], 0),
      pitchDeg: parseFiniteFloat(fields[5], 0),
      yawDeg: parseFiniteFloat(fields[6], 0)
    },
    servo: {
      aDeg: parseFiniteFloat(fields[7], 0),
      bDeg: parseFiniteFloat(fields[8], 0)
    },
    mode: normalizeFirmwareMode(fields[16]),
    firmwareMode: String(fields[16] ?? ""),
    rawFields: fields
  };
}

function parseImuFrame(frame) {
  const view = new DataView(frame.buffer, frame.byteOffset, frame.byteLength);
  return {
    type: "imu",
    packetFormat: "legacy-imu",
    messageId: MSG_ID_IMU,
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

function parseExtendedImuFrame(frame) {
  const view = new DataView(frame.buffer, frame.byteOffset, frame.byteLength);
  return {
    type: "imu",
    packetFormat: "extended-imu-sensors",
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
      } else if (frame[2] === MSG_ID_IMU) {
        frames.push(parseImuFrame(frame));
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
      frame[2] === MSG_ID_IMU &&
      frame[3] === EXTENDED_PAYLOAD_LENGTH
    );
  }

  frameLengthForCurrentHeader() {
    if (this.buffer[0] === LEGACY_SYNC_0 && this.buffer[1] === LEGACY_SYNC_1 && this.buffer[2] === MSG_ID_IMU) {
      return IMU_FRAME_LENGTH;
    }
    if (this.buffer.length >= 4 &&
        this.buffer[0] === EXTENDED_SYNC_0 &&
        this.buffer[1] === EXTENDED_SYNC_1 &&
        this.buffer[2] === MSG_ID_IMU &&
        this.buffer[3] === EXTENDED_PAYLOAD_LENGTH) {
      return EXTENDED_FRAME_LENGTH;
    }
    return 0;
  }
}

export function packImuFrame(sample, sequence) {
  const frame = new Uint8Array(IMU_FRAME_LENGTH);
  const view = new DataView(frame.buffer);
  frame[0] = TELEMETRY_SYNC_0;
  frame[1] = TELEMETRY_SYNC_1;
  frame[2] = MSG_ID_IMU;
  frame[3] = sequence & 0xff;
  view.setUint32(4, sample.timestampMs >>> 0, true);
  view.setFloat32(8, sample.accel.x, true);
  view.setFloat32(12, sample.accel.y, true);
  view.setFloat32(16, sample.accel.z, true);
  view.setFloat32(20, sample.gyro.x, true);
  view.setFloat32(24, sample.gyro.y, true);
  view.setFloat32(28, sample.gyro.z, true);
  view.setFloat32(32, sample.temperatureC, true);
  const crc = crc16CcittFalse(frame.slice(0, IMU_FRAME_LENGTH - 2));
  frame[36] = crc & 0xff;
  frame[37] = (crc >> 8) & 0xff;
  return frame;
}
