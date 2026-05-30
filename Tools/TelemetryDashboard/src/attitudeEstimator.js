const RAD_TO_DEG = 180 / Math.PI;
const DEG_TO_RAD = Math.PI / 180;

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function wrapPi(value) {
  let wrapped = value;
  while (wrapped > Math.PI) wrapped -= 2 * Math.PI;
  while (wrapped < -Math.PI) wrapped += 2 * Math.PI;
  return wrapped;
}

export function createEstimator() {
  return {
    rollRad: 0,
    pitchRad: 0,
    yawRad: 0,
    lastTimestampMs: null,
    isReady: false
  };
}

export function resetEstimator(estimator) {
  estimator.rollRad = 0;
  estimator.pitchRad = 0;
  estimator.yawRad = 0;
  estimator.lastTimestampMs = null;
  estimator.isReady = false;
}

export function updateEstimatorFromImu(estimator, sample) {
  const ax = sample.accel.x;
  const ay = sample.accel.y;
  const az = sample.accel.z;
  const accelRoll = Math.atan2(ay, az);
  const accelPitch = Math.atan2(-ax, Math.sqrt((ay * ay) + (az * az)));
  const dt = estimator.lastTimestampMs === null
    ? 0
    : clamp((sample.timestampMs - estimator.lastTimestampMs) / 1000, 0, 0.2);

  estimator.lastTimestampMs = sample.timestampMs;

  if (!estimator.isReady || dt === 0) {
    estimator.rollRad = accelRoll;
    estimator.pitchRad = accelPitch;
    estimator.yawRad = 0;
    estimator.isReady = true;
  } else {
    const alpha = 0.98;
    estimator.rollRad = (alpha * (estimator.rollRad + (sample.gyro.x * dt))) + ((1 - alpha) * accelRoll);
    estimator.pitchRad = (alpha * (estimator.pitchRad + (sample.gyro.y * dt))) + ((1 - alpha) * accelPitch);
    estimator.yawRad = wrapPi(estimator.yawRad + (sample.gyro.z * dt));
  }

  return attitudeSnapshot(estimator, sample);
}

export function attitudeSnapshot(estimator, sample) {
  const rollDeg = estimator.rollRad * RAD_TO_DEG;
  const pitchDeg = estimator.pitchRad * RAD_TO_DEG;
  const yawDeg = estimator.yawRad * RAD_TO_DEG;
  const tiltDeg = Math.sqrt((rollDeg * rollDeg) + (pitchDeg * pitchDeg));
  const servoA = clamp(-pitchDeg * 1.15, -35, 35);
  const servoB = clamp(rollDeg * 1.15, -35, 35);
  const mode = deriveMode(tiltDeg, sample);

  return {
    rollDeg,
    pitchDeg,
    yawDeg,
    tiltDeg,
    servoA,
    servoB,
    mode,
    timestampMs: sample.timestampMs,
    accel: sample.accel,
    gyro: sample.gyro,
    temperatureC: sample.temperatureC,
    pressurePa: sample.pressurePa ?? null,
    altitudeM: sample.altitudeM ?? null
  };
}

export function deriveMode(tiltDeg, sample) {
  const gyroMagnitude = Math.sqrt(
    (sample.gyro.x * sample.gyro.x) +
    (sample.gyro.y * sample.gyro.y) +
    (sample.gyro.z * sample.gyro.z)
  );

  if (tiltDeg > 38 || gyroMagnitude > 6) {
    return "FAILURE";
  }
  if (tiltDeg < 3 && gyroMagnitude < 0.4) {
    return "BALANCED";
  }
  return "RECOVERING";
}

export function makeSimulatedImuSample(elapsedMs) {
  const t = elapsedMs / 1000;
  const roll = (14 * Math.sin(t * 0.75) + 4 * Math.sin(t * 2.1)) * DEG_TO_RAD;
  const pitch = (11 * Math.cos(t * 0.63) + 3 * Math.sin(t * 1.7)) * DEG_TO_RAD;
  const yawRate = 0.12 * Math.sin(t * 0.45);
  const g = 9.80665;

  return {
    type: "imu",
    sequence: Math.floor(elapsedMs / 50) & 0xff,
    timestampMs: Math.floor(elapsedMs),
    accel: {
      x: -Math.sin(pitch) * g,
      y: Math.sin(roll) * Math.cos(pitch) * g,
      z: Math.cos(roll) * Math.cos(pitch) * g
    },
    gyro: {
      x: 0.19 * Math.cos(t * 0.75),
      y: -0.12 * Math.sin(t * 0.63),
      z: yawRate
    },
    temperatureC: 24.5 + (0.6 * Math.sin(t * 0.1)),
    pressurePa: 101325 + (22 * Math.sin(t * 0.2)),
    altitudeM: 0.15 + (0.05 * Math.sin(t * 0.35))
  };
}
