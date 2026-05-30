function u16Crc = Crc16CcittFalse(u8Data)
%CRC16CCITTFALSE CRC16/CCITT-FALSE (poly=0x1021, init=0xFFFF).
%
% Input:
%   u8Data : uint8 row/column vector
%
% Output:
%   u16Crc : uint16 CRC value

if ~isa(u8Data, 'uint8')
    error('Crc16CcittFalse:Type', 'u8Data must be uint8.');
end

u16Crc = uint16(hex2dec('FFFF'));
u16Poly = uint16(hex2dec('1021'));

for i = 1:numel(u8Data)
    u16Crc = bitxor(u16Crc, bitshift(uint16(u8Data(i)), 8));
    for b = 1:8
        if bitand(u16Crc, uint16(hex2dec('8000'))) ~= 0
            u16Crc = bitxor(bitshift(u16Crc, 1), u16Poly);
        else
            u16Crc = bitshift(u16Crc, 1);
        end
        u16Crc = uint16(u16Crc);
    end
end
end
