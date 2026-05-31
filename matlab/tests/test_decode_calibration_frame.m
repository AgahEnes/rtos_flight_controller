function test_decode_calibration_frame()
%TEST_DECODE_CALIBRATION_FRAME Validate calibration frame decoding.

addpath(fileparts(fileparts(mfilename('fullpath'))));
c = telem.FrameConstants();
u8Frame = localBuildValidCalibrationFrame(c);

s = telem.decodeCalibrationFrame(u8Frame, c);

assert(s.sequence == uint8(21), 'Sequence mismatch.');
assert(s.timestampMs == uint32(123456), 'Telemetry timestamp mismatch.');
assert(all(abs(s.accelBiasMps2 - [0.1 -0.2 0.3]) < 1e-6), 'Accel bias mismatch.');
assert(all(abs(s.gyroBiasRadS - [-0.4 0.5 -0.6]) < 1e-6), 'Gyro bias mismatch.');
assert(s.calibrationTimestampMs == uint32(2222), 'Calibration timestamp mismatch.');
assert(s.updateCounter == uint32(9), 'Update counter mismatch.');
assert(s.isValid == true, 'isValid mismatch.');
assert(s.crcOk == true, 'CRC expected true.');

disp('test_decode_calibration_frame: PASS');
end

function u8Frame = localBuildValidCalibrationFrame(c)
u8Frame = zeros(1, double(c.calibrationFrameLength), 'uint8');
u8Frame(1:4) = uint8([c.sync0 c.sync1 c.msgIdImuCalibration 21]);
u8Frame(5:8) = typecast(uint32(123456), 'uint8');
u8Frame(9:20) = typecast(single([0.1 -0.2 0.3]), 'uint8');
u8Frame(21:32) = typecast(single([-0.4 0.5 -0.6]), 'uint8');
u8Frame(33:36) = typecast(uint32(2222), 'uint8');
u8Frame(37:40) = typecast(uint32(9), 'uint8');
u8Frame(41) = uint8(1);
u16Crc = telem.Crc16CcittFalse(u8Frame(1:c.calibrationCrcDataLength));
u8Frame(42:43) = typecast(u16Crc, 'uint8');
end
