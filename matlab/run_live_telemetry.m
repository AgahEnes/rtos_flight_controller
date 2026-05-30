function run_live_telemetry(varargin)
%RUN_LIVE_TELEMETRY Live decoder + visualization for STM32 telemetry frames.
%
% Example:
%   run_live_telemetry();
%   run_live_telemetry("Port", "/dev/cu.usbserial-1410", "PlotHz", 10);


p = inputParser;
p.addParameter("Port", "", @(x)ischar(x) || isstring(x));
p.addParameter("BaudRate", 115200, @(x)isnumeric(x) && isscalar(x));
p.addParameter("TimeoutSec", 0.05, @(x)isnumeric(x) && isscalar(x));
p.addParameter("EnablePlot", true, @(x)islogical(x) || isnumeric(x));
p.addParameter("PlotHz", 10, @(x)isnumeric(x) && isscalar(x) && x > 0);
p.addParameter("WindowSec", 10, @(x)isnumeric(x) && isscalar(x) && x > 0);
p.addParameter("RingCapacity", 1000, @(x)isnumeric(x) && isscalar(x) && x >= 100);
p.addParameter("MaxPlotPoints", 300, @(x)isnumeric(x) && isscalar(x) && x >= 30);
p.addParameter("IdlePauseSec", 0.001, @(x)isnumeric(x) && isscalar(x) && x >= 0);
p.addParameter("EnableConsole", true, @(x)islogical(x) || isnumeric(x));
p.addParameter("PrintHz", 1, @(x)isnumeric(x) && isscalar(x) && x > 0);
p.addParameter("ExpectedSeqHz", 10, @(x)isnumeric(x) && isscalar(x) && x > 0);
p.addParameter("SeqCheckToleranceFrames", 0, @(x)isnumeric(x) && isscalar(x) && x >= 0);
p.parse(varargin{:});
cfg = p.Results;

thisDir = fileparts(mfilename('fullpath'));
addpath(thisDir);

c = telem.FrameConstants();
sPort = telem.selectPort(cfg.Port);
fprintf('Selected serial port: %s\n', sPort);

[sObj, cleanupObj] = telem.openSerial(sPort, cfg.BaudRate, cfg.TimeoutSec); %#ok<ASGLU>
parser = telem.FrameParser(c);
ring = telem.RingBuffer(cfg.RingCapacity);
plotter = telem.LivePlot(cfg);

fprintf('Listening: frame=%u bytes, baud=%u\n', c.frameLength, cfg.BaudRate);

validCount = uint32(0);
lastPrintTic = tic;

hasPrevFrame = false;
u8PrevSeq = uint8(0);
u32PrevTs = uint32(0);
f64AccumSeqDelta = 0.0;
f64AccumTsDeltaSec = 0.0;

while true
    if plotter.isClosed
        fprintf('Plot closed; stopping telemetry loop.\n');
        break;
    end

    nAvail = sObj.NumBytesAvailable;
    if nAvail > 0
        u8Chunk = uint8(read(sObj, nAvail, "uint8"));
        parser.push(u8Chunk);
    else
        pause(cfg.IdlePauseSec);
    end

    while parser.hasFrame()
        u8Frame = parser.pop();
        sFrame = telem.decodeFrame(u8Frame, c);
        if sFrame.crcOk
            ring.push(sFrame);
            validCount = validCount + 1;

            if hasPrevFrame
                u8SeqDelta = uint8(mod(double(sFrame.sequence) - double(u8PrevSeq), 256));
                f64TsDeltaMs = double(sFrame.timestampMs) - double(u32PrevTs);
                if f64TsDeltaMs < 0
                    f64TsDeltaMs = f64TsDeltaMs + 2^32;
                end

                f64AccumSeqDelta = f64AccumSeqDelta + double(u8SeqDelta);
                f64AccumTsDeltaSec = f64AccumTsDeltaSec + (f64TsDeltaMs / 1000.0);
            end

            u8PrevSeq = sFrame.sequence;
            u32PrevTs = sFrame.timestampMs;
            hasPrevFrame = true;
        end
    end

    plotter.maybeUpdate(ring);

    if cfg.EnableConsole && toc(lastPrintTic) >= (1 / cfg.PrintHz)
        if f64AccumTsDeltaSec > 0
            f64ExpectedSeq = f64AccumTsDeltaSec * cfg.ExpectedSeqHz;
            f64SeqErr = f64AccumSeqDelta - f64ExpectedSeq;
            f64SeqRateHz = f64AccumSeqDelta / f64AccumTsDeltaSec;
            if abs(f64SeqErr) <= cfg.SeqCheckToleranceFrames
                sSeqCheck = "OK";
            else
                sSeqCheck = "WARN";
            end
        else
            f64ExpectedSeq = 0;
            f64SeqErr = 0;
            f64SeqRateHz = 0;
            sSeqCheck = "N/A";
        end

        fprintf(['frames=%u crcDrops=%u headerDrops=%u buffered=%u | ' ...
                 'seqCheck=%s seqDelta=%.1f expSeq=%.1f err=%.2f seqHz=%.2f\n'], ...
            validCount, parser.crcDropCount, parser.headerDropCount, ring.count, ...
            sSeqCheck, f64AccumSeqDelta, f64ExpectedSeq, f64SeqErr, f64SeqRateHz);

        f64AccumSeqDelta = 0.0;
        f64AccumTsDeltaSec = 0.0;
        lastPrintTic = tic;
    end
end
end
