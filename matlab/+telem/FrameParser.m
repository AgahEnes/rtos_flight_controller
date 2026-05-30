classdef FrameParser < handle
    %FRAMEPARSER Incremental telemetry frame parser with CRC-based resync.

    properties (Access = private)
        c
        state uint8 = 0
        frameBuf uint8
        framePos uint16 = 0
        outQueue cell
    end

    properties (SetAccess = private)
        validCount uint32 = 0
        crcDropCount uint32 = 0
        headerDropCount uint32 = 0
    end

    methods
        function obj = FrameParser(constants)
            if nargin < 1
                constants = telem.FrameConstants();
            end
            obj.c = constants;
            obj.frameBuf = zeros(1, double(obj.c.frameLength), 'uint8');
            obj.outQueue = {};
        end

        function push(obj, u8Chunk)
            if isempty(u8Chunk)
                return;
            end

            % Some MATLAB versions/adapters may return serial bytes as double.
            % Accept numeric byte vectors and normalize to uint8.
            if ~isa(u8Chunk, 'uint8')
                if ~isnumeric(u8Chunk)
                    error('FrameParser:Type', 'push input must be numeric byte data.');
                end

                if any(~isfinite(u8Chunk(:))) || any(u8Chunk(:) < 0) || any(u8Chunk(:) > 255) || ...
                        any(u8Chunk(:) ~= floor(u8Chunk(:)))
                    error('FrameParser:Range', ...
                          'push input must contain integer byte values in [0, 255].');
                end

                u8Chunk = uint8(u8Chunk);
            end

            u8Chunk = u8Chunk(:).';
            for k = 1:numel(u8Chunk)
                obj.consumeByte(u8Chunk(k));
            end
        end

        function tf = hasFrame(obj)
            tf = ~isempty(obj.outQueue);
        end

        function u8Frame = pop(obj)
            if isempty(obj.outQueue)
                u8Frame = uint8([]);
                return;
            end

            u8Frame = obj.outQueue{1};
            obj.outQueue(1) = [];
        end

        function reset(obj)
            obj.state = uint8(0);
            obj.framePos = uint16(0);
            obj.outQueue = {};
            obj.validCount = uint32(0);
            obj.crcDropCount = uint32(0);
            obj.headerDropCount = uint32(0);
        end
    end

    methods (Access = private)
        function consumeByte(obj, b)
            switch obj.state
                case 0 % hunt sync0
                    if b == obj.c.sync0
                        obj.framePos = uint16(1);
                        obj.frameBuf(1) = b;
                        obj.state = uint8(1);
                    end

                case 1 % expect sync1
                    if b == obj.c.sync1
                        obj.framePos = uint16(2);
                        obj.frameBuf(2) = b;
                        obj.state = uint8(2);
                    elseif b == obj.c.sync0
                        obj.framePos = uint16(1);
                        obj.frameBuf(1) = b;
                        obj.state = uint8(1);
                    else
                        obj.state = uint8(0);
                        obj.framePos = uint16(0);
                    end

                case 2 % expect message id
                    if b == obj.c.msgIdImuVehicleState
                        obj.framePos = uint16(3);
                        obj.frameBuf(3) = b;
                        obj.state = uint8(3);
                    elseif b == obj.c.sync0
                        obj.framePos = uint16(1);
                        obj.frameBuf(1) = b;
                        obj.state = uint8(1);
                        obj.headerDropCount = obj.headerDropCount + 1;
                    else
                        obj.state = uint8(0);
                        obj.framePos = uint16(0);
                        obj.headerDropCount = obj.headerDropCount + 1;
                    end

                otherwise % collect until frame complete
                    obj.framePos = obj.framePos + 1;
                    obj.frameBuf(obj.framePos) = b;
                    if obj.framePos == obj.c.frameLength
                        obj.finalizeCandidateFrame();
                    end
            end
        end

        function finalizeCandidateFrame(obj)
            u16RxCrc = bitor(uint16(obj.frameBuf(obj.c.idxCrcStart)), ...
                             bitshift(uint16(obj.frameBuf(obj.c.idxCrcStart + 1)), 8));
            u16Calc = telem.Crc16CcittFalse(obj.frameBuf(1:obj.c.crcDataLength));
            if u16RxCrc == u16Calc
                obj.outQueue{end + 1} = obj.frameBuf; %#ok<AGROW>
                obj.validCount = obj.validCount + 1;
                obj.state = uint8(0);
                obj.framePos = uint16(0);
                return;
            end

            obj.crcDropCount = obj.crcDropCount + 1;
            obj.resyncAfterCrcFail();
        end

        function resyncAfterCrcFail(obj)
            % Keep only a possible leading sync pattern from the tail.
            tail = obj.frameBuf(end-1:end);
            obj.state = uint8(0);
            obj.framePos = uint16(0);

            if tail(1) == obj.c.sync0 && tail(2) == obj.c.sync1
                obj.frameBuf(1:2) = tail;
                obj.state = uint8(2);
                obj.framePos = uint16(2);
            elseif tail(2) == obj.c.sync0
                obj.frameBuf(1) = tail(2);
                obj.state = uint8(1);
                obj.framePos = uint16(1);
            end
        end
    end
end
