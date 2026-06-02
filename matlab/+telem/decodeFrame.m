function sFrame = decodeFrame(u8Frame, c)
%DECODEFRAME Decode one telemetry frame into typed fields.
%
% sFrame fields:
%   sequence, timestampMs, accelMps2[1x3], gyroRadS[1x3], tempC,
%   rollRad, pitchRad, yawRad, rollRateRadS, pitchRateRadS, yawRateRadS,
%   isEstimated, crcRx, crcCalc, crcOk.

if nargin < 2
    c = telem.FrameConstants();
end

if ~isa(u8Frame, 'uint8')
    error('decodeFrame:Type', 'u8Frame must be uint8.');
end
if numel(u8Frame) ~= double(c.frameLength)
    error('decodeFrame:Length', 'u8Frame length must be %u bytes.', c.frameLength);
end

u8Frame = u8Frame(:).';

if u8Frame(1) ~= c.sync0 || u8Frame(2) ~= c.sync1 || u8Frame(3) ~= c.msgIdImuVehicleState
    error('decodeFrame:Header', 'Frame header is invalid.');
end

u16RxCrc = bitor(uint16(u8Frame(c.idxCrcStart)), ...
                 bitshift(uint16(u8Frame(c.idxCrcStart + 1)), 8));
u16CalcCrc = telem.Crc16CcittFalse(u8Frame(1:c.crcDataLength));

u32TimestampMs = typecast(u8Frame(c.idxTimestampStart:c.idxTimestampStart + 3), 'uint32');
f32Imu = typecast(u8Frame(c.idxImuStart:c.idxImuStart + double(c.imuPayloadLength) - 1), 'single');
f32Vehicle = typecast(u8Frame(c.idxVehicleStart:c.idxVehicleStart + 23), 'single');

sFrame = struct();
sFrame.sequence = u8Frame(c.idxSeq);
sFrame.timestampMs = u32TimestampMs;
sFrame.accelMps2 = double(f32Imu(1:3));
sFrame.gyroRadS = double(f32Imu(4:6));
sFrame.tempC = double(f32Imu(7));
sFrame.rollRad = double(f32Vehicle(1));
sFrame.pitchRad = double(f32Vehicle(2));
sFrame.yawRad = double(f32Vehicle(3));
sFrame.rollRateRadS = double(f32Vehicle(4));
sFrame.pitchRateRadS = double(f32Vehicle(5));
sFrame.yawRateRadS = double(f32Vehicle(6));
sFrame.isEstimated = (u8Frame(c.idxIsEstimated) ~= 0);
sFrame.crcRx = u16RxCrc;
sFrame.crcCalc = u16CalcCrc;
sFrame.crcOk = (u16RxCrc == u16CalcCrc);
end
