% GNC IMU telemetry live parser (binary frame reader)
% Frame format (38 bytes total):
%   [0]   Sync0 = 0xA5
%   [1]   Sync1 = 0x5A
%   [2]   MsgId = 0x10
%   [3]   Sequence (uint8)
%   [4:7] TimestampMs (uint32, little-endian)
%   [8:35] 7x float32 little-endian
%   [36:37] CRC16/CCITT-FALSE over bytes [0:35], little-endian

clear; clc;

% ---- User config ----
u32BaudRate = 115200;
bEnablePlot = true;
u32PrintDecimation = 10;          % Print one line every N valid frames.
f64PlotUpdatePeriodSec = 0.05;    % Plot refresh period (20 Hz).
f64PlotWindowSec = 10.0;          % Sliding window size in seconds.
u32PlotCapacity = 2000;           % Ring buffer capacity for plotting.
u32MaxRxBufferBytes = 4096;       % Bound RX buffer to avoid growth on desync.
% ---------------------

u8Sync0 = uint8(hex2dec('A5'));
u8Sync1 = uint8(hex2dec('5A'));
u8MsgIdImu = uint8(hex2dec('10'));
u16FrameLen = uint16(38);

asPorts = serialportlist("available");
if isempty(asPorts)
    error('No available serial port found.');
end

sSelectedPort = prvSelectPort(asPorts);
fprintf('Selected serial port: %s\n', sSelectedPort);

sObj = serialport(sSelectedPort, u32BaudRate, "Timeout", 0.2);
cleanupObj = onCleanup(@() prvCleanupSerial(sObj)); %#ok<NASGU>
flush(sObj);

if bEnablePlot
    tFig = figure('Name', 'GNC IMU Telemetry', 'NumberTitle', 'off');
    tAx1 = subplot(2,1,1, 'Parent', tFig); grid(tAx1, 'on'); hold(tAx1, 'on');
    title(tAx1, 'Acceleration (m/s^2)');
    xlabel(tAx1, 'Time (s)'); ylabel(tAx1, 'm/s^2');
    hAx = plot(tAx1, nan, nan, 'Color', [0.85 0.20 0.20], 'DisplayName', 'ax');
    hAy = plot(tAx1, nan, nan, 'Color', [0.20 0.60 0.20], 'DisplayName', 'ay');
    hAz = plot(tAx1, nan, nan, 'Color', [0.20 0.30 0.85], 'DisplayName', 'az');
    legend(tAx1, 'show');

    tAx2 = subplot(2,1,2, 'Parent', tFig); grid(tAx2, 'on'); hold(tAx2, 'on');
    title(tAx2, 'Gyro (rad/s)');
    xlabel(tAx2, 'Time (s)'); ylabel(tAx2, 'rad/s');
    hGx = plot(tAx2, nan, nan, 'Color', [0.85 0.20 0.20], 'DisplayName', 'gx');
    hGy = plot(tAx2, nan, nan, 'Color', [0.20 0.60 0.20], 'DisplayName', 'gy');
    hGz = plot(tAx2, nan, nan, 'Color', [0.20 0.30 0.85], 'DisplayName', 'gz');
    legend(tAx2, 'show');
else
    tFig = [];
end

fprintf('Listening... Press Ctrl+C to stop.\n');
u8Buffer = zeros(0,1,'uint8');
f64T0 = [];
u32ValidFrameCount = uint32(0);
tLastPlotUpdate = tic;

% Plot ring buffers.
vf64T = nan(u32PlotCapacity, 1);
vf64Ax = nan(u32PlotCapacity, 1);
vf64Ay = nan(u32PlotCapacity, 1);
vf64Az = nan(u32PlotCapacity, 1);
vf64Gx = nan(u32PlotCapacity, 1);
vf64Gy = nan(u32PlotCapacity, 1);
vf64Gz = nan(u32PlotCapacity, 1);
u32RingHead = 0;
u32RingCount = 0;

while true
    if ~isempty(tFig) && ~isvalid(tFig)
        fprintf('Plot closed. Stopping parser.\n');
        break;
    end

    u32Avail = sObj.NumBytesAvailable;
    if u32Avail > 0
        u8Chunk = read(sObj, u32Avail, "uint8");
        u8Buffer = [u8Buffer; u8Chunk(:)]; %#ok<AGROW>
        if numel(u8Buffer) > u32MaxRxBufferBytes
            u32Keep = max(double(u16FrameLen) * 4, 2);
            u8Buffer = u8Buffer(end-u32Keep+1:end);
        end
    else
        pause(0.005);
    end

    while true
        if numel(u8Buffer) < 2
            break;
        end

        u32SyncIdx = find(u8Buffer(1:end-1) == u8Sync0 & u8Buffer(2:end) == u8Sync1, 1, 'first');
        if isempty(u32SyncIdx)
            if ~isempty(u8Buffer) && u8Buffer(end) == u8Sync0
                u8Buffer = u8Buffer(end); % Keep partial sync candidate.
            else
                u8Buffer = zeros(0,1,'uint8');
            end
            break;
        end

        if u32SyncIdx > 1
            u8Buffer(1:u32SyncIdx-1) = [];
        end

        if numel(u8Buffer) < u16FrameLen
            break;
        end

        u8Frame = u8Buffer(1:u16FrameLen);
        u8Buffer(1:u16FrameLen) = [];

        if u8Frame(3) ~= u8MsgIdImu
            fprintf('[drop] Unknown msg id: 0x%02X\n', u8Frame(3));
            continue;
        end

        u16RxCrc = uint16(u8Frame(37)) + bitshift(uint16(u8Frame(38)), 8);
        u16CalcCrc = prvCrc16CcittFalse(u8Frame(1:36));
        if u16RxCrc ~= u16CalcCrc
            fprintf('[drop] CRC mismatch rx=0x%04X calc=0x%04X\n', u16RxCrc, u16CalcCrc);
            continue;
        end

        u8Seq = u8Frame(4);
        u32TsMs = prvReadU32Le(u8Frame(5:8));
        f32Vals = zeros(1,7, 'single');
        for u8i = 1:7
            u32Start = 9 + (u8i - 1) * 4;
            f32Vals(u8i) = prvReadF32Le(u8Frame(u32Start:u32Start+3));
        end

        f32Ax = f32Vals(1); f32Ay = f32Vals(2); f32Az = f32Vals(3);
        f32Gx = f32Vals(4); f32Gy = f32Vals(5); f32Gz = f32Vals(6);
        f32Temp = f32Vals(7);

        u32ValidFrameCount = u32ValidFrameCount + 1;
        if mod(u32ValidFrameCount, u32PrintDecimation) == 0
            fprintf(['seq=%3u t=%8u ms | acc=(%8.3f,%8.3f,%8.3f) m/s^2 | ' ...
                     'gyro=(%8.3f,%8.3f,%8.3f) rad/s | T=%6.2f C\n'], ...
                     u8Seq, u32TsMs, f32Ax, f32Ay, f32Az, f32Gx, f32Gy, f32Gz, f32Temp);
        end

        if bEnablePlot && ~isempty(tFig) && isvalid(tFig)
            if isempty(f64T0)
                f64T0 = double(u32TsMs);
            end
            f64T = (double(u32TsMs) - f64T0) / 1000.0;

            u32RingHead = mod(u32RingHead, u32PlotCapacity) + 1;
            if u32RingCount < u32PlotCapacity
                u32RingCount = u32RingCount + 1;
            end

            vf64T(u32RingHead) = f64T;
            vf64Ax(u32RingHead) = double(f32Ax);
            vf64Ay(u32RingHead) = double(f32Ay);
            vf64Az(u32RingHead) = double(f32Az);
            vf64Gx(u32RingHead) = double(f32Gx);
            vf64Gy(u32RingHead) = double(f32Gy);
            vf64Gz(u32RingHead) = double(f32Gz);

            if toc(tLastPlotUpdate) >= f64PlotUpdatePeriodSec
                vu32Idx = prvRingIndices(u32RingHead, u32RingCount, u32PlotCapacity);
                vf64X = vf64T(vu32Idx);
                vf64YAx = vf64Ax(vu32Idx);
                vf64YAy = vf64Ay(vu32Idx);
                vf64YAz = vf64Az(vu32Idx);
                vf64YGx = vf64Gx(vu32Idx);
                vf64YGy = vf64Gy(vu32Idx);
                vf64YGz = vf64Gz(vu32Idx);

                if ~isempty(vf64X)
                    f64XEnd = vf64X(end);
                    vbWindow = vf64X >= (f64XEnd - f64PlotWindowSec);
                    vf64X = vf64X(vbWindow);
                    vf64YAx = vf64YAx(vbWindow);
                    vf64YAy = vf64YAy(vbWindow);
                    vf64YAz = vf64YAz(vbWindow);
                    vf64YGx = vf64YGx(vbWindow);
                    vf64YGy = vf64YGy(vbWindow);
                    vf64YGz = vf64YGz(vbWindow);

                    set(hAx, 'XData', vf64X, 'YData', vf64YAx);
                    set(hAy, 'XData', vf64X, 'YData', vf64YAy);
                    set(hAz, 'XData', vf64X, 'YData', vf64YAz);
                    set(hGx, 'XData', vf64X, 'YData', vf64YGx);
                    set(hGy, 'XData', vf64X, 'YData', vf64YGy);
                    set(hGz, 'XData', vf64X, 'YData', vf64YGz);

                    if numel(vf64X) >= 2
                        xlim(tAx1, [vf64X(1), vf64X(end)]);
                        xlim(tAx2, [vf64X(1), vf64X(end)]);
                    end
                end

                drawnow limitrate nocallbacks;
                tLastPlotUpdate = tic;
            end
        end
    end
end

function sPort = prvSelectPort(asPorts)
    asLower = lower(asPorts);

    % Favor direct USB UART adapters, then generic tty/COM candidates.
    bPreferred = contains(asLower, "usb") | contains(asLower, "acm") | ...
                 contains(asLower, "serial") | contains(asLower, "wch") | ...
                 contains(asLower, "ch340") | contains(asLower, "silab");
    bReject = contains(asLower, "bluetooth") | contains(asLower, "irda");

    idx = find(bPreferred & ~bReject, 1, "first");
    if isempty(idx)
        idx = find(~bReject, 1, "first");
    end
    if isempty(idx)
        idx = 1;
    end

    sPort = asPorts(idx);

    fprintf('Available serial ports:\n');
    for i = 1:numel(asPorts)
        marker = ' ';
        if i == idx
            marker = '*';
        end
        fprintf('  [%s] %s\n', marker, asPorts(i));
    end
end

function u16Crc = prvCrc16CcittFalse(u8Data)
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
        end
    end
end

function u32Value = prvReadU32Le(u8Bytes4)
    u32Value = uint32(u8Bytes4(1)) + ...
               bitshift(uint32(u8Bytes4(2)), 8) + ...
               bitshift(uint32(u8Bytes4(3)), 16) + ...
               bitshift(uint32(u8Bytes4(4)), 24);
end

function f32Value = prvReadF32Le(u8Bytes4)
    u32Raw = prvReadU32Le(u8Bytes4);
    f32Value = typecast(u32Raw, 'single');
end

function prvCleanupSerial(sObj)
    if ~isempty(sObj) && isvalid(sObj)
        flush(sObj);
        clear sObj;
    end
end

function vu32Idx = prvRingIndices(u32Head, u32Count, u32Capacity)
    if u32Count <= 0
        vu32Idx = zeros(0,1);
        return;
    end

    if u32Count < u32Capacity
        vu32Idx = (1:u32Count).';
        return;
    end

    vu32Idx = [((u32Head+1):u32Capacity), (1:u32Head)].';
end
