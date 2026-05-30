function test_decode_frame()
%TEST_DECODE_FRAME Validate binary frame decoding.

addpath(fileparts(fileparts(mfilename('fullpath'))));
c = telem.FrameConstants();
u8Frame = localBuildValidFrame(c);

s = telem.decodeFrame(u8Frame, c);

assert(s.sequence == uint8(42), 'Sequence mismatch.');
assert(s.timestampMs == uint32(987654321), 'Timestamp mismatch.');
assert(all(abs(s.accelMps2 - [0.5 -1.5 9.81]) < 1e-6), 'Accel mismatch.');
assert(all(abs(s.gyroRadS - [0.1 0.2 0.3]) < 1e-6), 'Gyro mismatch.');
assert(abs(s.tempC - 32.25) < 1e-6, 'Temp mismatch.');
assert(abs(s.rollRad - 0.01) < 1e-6, 'Roll mismatch.');
assert(abs(s.pitchRad + 0.02) < 1e-6, 'Pitch mismatch.');
assert(abs(s.yawRad - 0.03) < 1e-6, 'Yaw mismatch.');
assert(abs(s.rollRateRadS - 0.4) < 1e-6, 'RollRate mismatch.');
assert(abs(s.pitchRateRadS + 0.5) < 1e-6, 'PitchRate mismatch.');
assert(abs(s.yawRateRadS - 0.6) < 1e-6, 'YawRate mismatch.');
assert(s.isEstimated == true, 'isEstimated mismatch.');
assert(s.crcOk == true, 'CRC expected true.');

disp('test_decode_frame: PASS');
end

function u8Frame = localBuildValidFrame(c)
u8Frame = zeros(1, double(c.frameLength), 'uint8');
u8Frame(1:4) = uint8([c.sync0 c.sync1 c.msgIdImuVehicleState 42]);
u8Frame(5:8) = typecast(uint32(987654321), 'uint8');
u8Frame(9:36) = typecast(single([0.5 -1.5 9.81 0.1 0.2 0.3 32.25]), 'uint8');
u8Frame(37:60) = typecast(single([0.01 -0.02 0.03 0.4 -0.5 0.6]), 'uint8');
u8Frame(61) = uint8(1);
u16Crc = telem.Crc16CcittFalse(u8Frame(1:c.crcDataLength));
u8Frame(62:63) = typecast(u16Crc, 'uint8');
end
