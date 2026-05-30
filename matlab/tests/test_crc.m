function test_crc()
%TEST_CRC Validate CRC16/CCITT-FALSE implementation.

addpath(fileparts(fileparts(mfilename('fullpath'))));
c = telem.FrameConstants();

u8Data = uint8([1 2 3 4 5 6 7 8 9]);
u16Crc = telem.Crc16CcittFalse(u8Data);
assert(u16Crc == uint16(hex2dec('3B0A')), 'CRC mismatch for known vector.');

u8Frame = localBuildValidFrame(c);
u16RxCrc = bitor(uint16(u8Frame(c.idxCrcStart)), ...
                 bitshift(uint16(u8Frame(c.idxCrcStart + 1)), 8));
u16Calc = telem.Crc16CcittFalse(u8Frame(1:c.crcDataLength));
assert(u16RxCrc == u16Calc, 'CRC mismatch for synthesized frame.');

disp('test_crc: PASS');
end

function u8Frame = localBuildValidFrame(c)
u8Frame = zeros(1, double(c.frameLength), 'uint8');
u8Frame(1:4) = uint8([c.sync0 c.sync1 c.msgIdImuVehicleState 25]);
u8Frame(5:8) = typecast(uint32(123456), 'uint8');

f32Imu = single([1.25 -2.50 3.75 0.10 -0.20 0.30 27.5]);
u8Frame(9:36) = typecast(f32Imu, 'uint8');

f32Vehicle = single([0.11 -0.22 0.33 0.44 -0.55 0.66]);
u8Frame(37:60) = typecast(f32Vehicle, 'uint8');
u8Frame(61) = uint8(1);

u16Crc = telem.Crc16CcittFalse(u8Frame(1:c.crcDataLength));
u8Frame(62:63) = typecast(u16Crc, 'uint8');
end
