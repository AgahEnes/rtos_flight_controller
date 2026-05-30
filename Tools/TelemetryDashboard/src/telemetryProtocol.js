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
    return byte === LEGACY_SYNC_0 || byte === EXTENDED_SYNC_0;
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
