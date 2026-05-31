function sFrame = decodeCalibrationFrame(u8Frame, c)
%DECODECALIBRATIONFRAME Decode one IMU calibration telemetry frame.
%
% sFrame fields:
%   sequence, timestampMs, accelBiasMps2[1x3], gyroBiasRadS[1x3],
%   calibrationTimestampMs, updateCounter, isValid, crcRx, crcCalc, crcOk.

if nargin < 2
    c = telem.FrameConstants();
end

if ~isa(u8Frame, 'uint8')
    error('decodeCalibrationFrame:Type', 'u8Frame must be uint8.');
end
if numel(u8Frame) ~= double(c.calibrationFrameLength)
    error('decodeCalibrationFrame:Length', ...
          'u8Frame length must be %u bytes.', c.calibrationFrameLength);
end

u8Frame = u8Frame(:).';

if u8Frame(1) ~= c.sync0 || u8Frame(2) ~= c.sync1 || u8Frame(3) ~= c.msgIdImuCalibration
    error('decodeCalibrationFrame:Header', 'Frame header is invalid.');
end

u16RxCrc = bitor(uint16(u8Frame(c.idxCalCrcStart)), ...
                 bitshift(uint16(u8Frame(c.idxCalCrcStart + 1)), 8));
u16CalcCrc = telem.Crc16CcittFalse(u8Frame(1:c.calibrationCrcDataLength));

u32TelemetryTimestampMs = typecast(u8Frame(c.idxTimestampStart:c.idxTimestampStart + 3), 'uint32');
f32AccelBias = typecast(u8Frame(9:20), 'single');
f32GyroBias = typecast(u8Frame(21:32), 'single');
u32CalibrationTimestampMs = typecast(u8Frame(c.idxCalTimestampStart:c.idxCalTimestampStart + 3), 'uint32');
u32UpdateCounter = typecast(u8Frame(c.idxCalUpdateCounterStart:c.idxCalUpdateCounterStart + 3), 'uint32');

sFrame = struct();
sFrame.sequence = u8Frame(c.idxSeq);
sFrame.timestampMs = u32TelemetryTimestampMs;
sFrame.accelBiasMps2 = double(f32AccelBias(1:3));
sFrame.gyroBiasRadS = double(f32GyroBias(1:3));
sFrame.calibrationTimestampMs = u32CalibrationTimestampMs;
sFrame.updateCounter = u32UpdateCounter;
sFrame.isValid = (u8Frame(c.idxCalIsValid) ~= 0);
sFrame.crcRx = u16RxCrc;
sFrame.crcCalc = u16CalcCrc;
sFrame.crcOk = (u16RxCrc == u16CalcCrc);
end
