function test_parser_resync()
%TEST_PARSER_RESYNC Validate parser with noise and CRC corruption.

addpath(fileparts(fileparts(mfilename('fullpath'))));
c = telem.FrameConstants();
p = telem.FrameParser(c);

u8FrameA = localBuildValidFrame(c, uint8(10), uint32(1000));
u8FrameB = localBuildValidFrame(c, uint8(11), uint32(1100));
u8FrameCorrupt = u8FrameA;
u8FrameCorrupt(15) = bitxor(u8FrameCorrupt(15), uint8(1));

u8Noise = uint8([99 88 77 66 55 44 33]);
u8Stream = [u8Noise u8FrameCorrupt u8Noise u8FrameA u8Noise u8FrameB u8Noise];

chunkSizes = [3 1 9 5 17 2 11 4 23 7 13];
idx = 1;
while idx <= numel(u8Stream)
    n = chunkSizes(mod(idx - 1, numel(chunkSizes)) + 1);
    j = min(idx + n - 1, numel(u8Stream));
    p.push(u8Stream(idx:j));
    idx = j + 1;
end

frames = {};
while p.hasFrame()
    frames{end + 1} = p.pop(); %#ok<AGROW>
end

assert(numel(frames) == 2, 'Expected 2 valid frames after resync.');
s1 = telem.decodeFrame(frames{1}, c);
s2 = telem.decodeFrame(frames{2}, c);
assert(s1.sequence == uint8(10), 'First sequence mismatch.');
assert(s2.sequence == uint8(11), 'Second sequence mismatch.');
assert(p.crcDropCount >= 1, 'CRC drop count should increase for corrupt frame.');

disp('test_parser_resync: PASS');
end

function u8Frame = localBuildValidFrame(c, seq, tsMs)
u8Frame = zeros(1, double(c.frameLength), 'uint8');
u8Frame(1:4) = uint8([c.sync0 c.sync1 c.msgIdImuVehicleState seq]);
u8Frame(5:8) = typecast(uint32(tsMs), 'uint8');
u8Frame(9:36) = typecast(single([1 2 3 0.1 0.2 0.3 25]), 'uint8');
u8Frame(37:60) = typecast(single([0.1 0.2 0.3 0.4 0.5 0.6]), 'uint8');
u8Frame(61) = uint8(1);
u16Crc = telem.Crc16CcittFalse(u8Frame(1:c.crcDataLength));
u8Frame(62:63) = typecast(u16Crc, 'uint8');
end
