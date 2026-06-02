classdef RingBuffer < handle
    %RINGBUFFER Circular storage for decoded telemetry samples.

    properties (SetAccess = private)
        capacity uint32
        head uint32 = 0
        count uint32 = 0
        droppedCount uint32 = 0
    end

    properties (Access = private)
        tSec double
        seq uint8
        accelX double
        accelY double
        accelZ double
        tempC double
        gyroXDegS double
        gyroYDegS double
        gyroZDegS double
        rollDeg double
        pitchDeg double
        yawDeg double
        rollRateDegS double
        pitchRateDegS double
        yawRateDegS double
        errXDegS double
        errYDegS double
        errZDegS double
        isEstimated double

        firstTimestamp uint32 = 0
        lastTimestamp uint32 = 0
        timeAccumulatorSec double = 0
        hasTimestamp logical = false
    end

    methods
        function obj = RingBuffer(capacity)
            if nargin < 1
                capacity = 1000;
            end
            validateattributes(capacity, {'numeric'}, {'scalar', 'integer', '>=', 10});
            obj.capacity = uint32(capacity);
            obj.allocate();
        end

        function push(obj, sFrame)
            tSec = obj.nextTimeSec(uint32(sFrame.timestampMs));

            gx = rad2deg(sFrame.gyroRadS(1));
            gy = rad2deg(sFrame.gyroRadS(2));
            gz = rad2deg(sFrame.gyroRadS(3));
            rr = rad2deg(sFrame.rollRateRadS);
            pr = rad2deg(sFrame.pitchRateRadS);
            yr = rad2deg(sFrame.yawRateRadS);

            i = mod(double(obj.head), double(obj.capacity)) + 1;
            obj.head = uint32(i);
            if obj.count < obj.capacity
                obj.count = obj.count + 1;
            else
                obj.droppedCount = obj.droppedCount + 1;
            end

            obj.tSec(i) = tSec;
            obj.seq(i) = sFrame.sequence;
            obj.accelX(i) = sFrame.accelMps2(1);
            obj.accelY(i) = sFrame.accelMps2(2);
            obj.accelZ(i) = sFrame.accelMps2(3);
            obj.tempC(i) = sFrame.tempC;
            obj.gyroXDegS(i) = gx;
            obj.gyroYDegS(i) = gy;
            obj.gyroZDegS(i) = gz;
            obj.rollDeg(i) = rad2deg(sFrame.rollRad);
            obj.pitchDeg(i) = rad2deg(sFrame.pitchRad);
            obj.yawDeg(i) = rad2deg(sFrame.yawRad);
            obj.rollRateDegS(i) = rr;
            obj.pitchRateDegS(i) = pr;
            obj.yawRateDegS(i) = yr;
            obj.errXDegS(i) = abs(rr - gx);
            obj.errYDegS(i) = abs(pr - gy);
            obj.errZDegS(i) = abs(yr - gz);
            obj.isEstimated(i) = double(sFrame.isEstimated);
        end

        function s = getWindow(obj, windowSec, maxPoints)
            if nargin < 2 || isempty(windowSec)
                windowSec = 10;
            end
            if nargin < 3 || isempty(maxPoints)
                maxPoints = 300;
            end

            if obj.count == 0
                s = obj.emptyWindow();
                return;
            end

            idx = obj.linearIndices();
            t = obj.tSec(idx);
            tEnd = t(end);
            keep = (t >= (tEnd - windowSec));
            idx = idx(keep);

            if isempty(idx)
                s = obj.emptyWindow();
                return;
            end

            n = numel(idx);
            if n > maxPoints
                step = ceil(n / maxPoints);
                idx = idx(1:step:end);
            end

            s.tSec = obj.tSec(idx);
            s.seq = obj.seq(idx);
            s.rollDeg = obj.rollDeg(idx);
            s.pitchDeg = obj.pitchDeg(idx);
            s.yawDeg = obj.yawDeg(idx);
            s.gyroXDegS = obj.gyroXDegS(idx);
            s.gyroYDegS = obj.gyroYDegS(idx);
            s.gyroZDegS = obj.gyroZDegS(idx);
            s.rollRateDegS = obj.rollRateDegS(idx);
            s.pitchRateDegS = obj.pitchRateDegS(idx);
            s.yawRateDegS = obj.yawRateDegS(idx);
            s.errXDegS = obj.errXDegS(idx);
            s.errYDegS = obj.errYDegS(idx);
            s.errZDegS = obj.errZDegS(idx);
            s.isEstimated = obj.isEstimated(idx);
            s.accelX = obj.accelX(idx);
            s.accelY = obj.accelY(idx);
            s.accelZ = obj.accelZ(idx);
            s.tempC = obj.tempC(idx);
        end
    end

    methods (Access = private)
        function allocate(obj)
            n = double(obj.capacity);
            obj.tSec = nan(1, n);
            obj.seq = zeros(1, n, 'uint8');
            obj.accelX = nan(1, n);
            obj.accelY = nan(1, n);
            obj.accelZ = nan(1, n);
            obj.tempC = nan(1, n);
            obj.gyroXDegS = nan(1, n);
            obj.gyroYDegS = nan(1, n);
            obj.gyroZDegS = nan(1, n);
            obj.rollDeg = nan(1, n);
            obj.pitchDeg = nan(1, n);
            obj.yawDeg = nan(1, n);
            obj.rollRateDegS = nan(1, n);
            obj.pitchRateDegS = nan(1, n);
            obj.yawRateDegS = nan(1, n);
            obj.errXDegS = nan(1, n);
            obj.errYDegS = nan(1, n);
            obj.errZDegS = nan(1, n);
            obj.isEstimated = nan(1, n);
        end

        function idx = linearIndices(obj)
            if obj.count == 0
                idx = zeros(1, 0);
                return;
            end
            if obj.count < obj.capacity
                idx = 1:double(obj.count);
                return;
            end

            h = double(obj.head);
            n = double(obj.capacity);
            idx = [h+1:n, 1:h];
        end

        function tSec = nextTimeSec(obj, ts)
            if ~obj.hasTimestamp
                obj.firstTimestamp = ts;
                obj.lastTimestamp = ts;
                obj.timeAccumulatorSec = 0;
                obj.hasTimestamp = true;
                tSec = 0;
                return;
            end

            d = double(ts) - double(obj.lastTimestamp);
            if d < 0
                d = d + 2^32;
            end
            obj.timeAccumulatorSec = obj.timeAccumulatorSec + (d / 1000.0);
            obj.lastTimestamp = ts;
            tSec = obj.timeAccumulatorSec;
        end

        function s = emptyWindow(~)
            s = struct( ...
                'tSec', zeros(1, 0), ...
                'seq', zeros(1, 0, 'uint8'), ...
                'rollDeg', zeros(1, 0), ...
                'pitchDeg', zeros(1, 0), ...
                'yawDeg', zeros(1, 0), ...
                'gyroXDegS', zeros(1, 0), ...
                'gyroYDegS', zeros(1, 0), ...
                'gyroZDegS', zeros(1, 0), ...
                'rollRateDegS', zeros(1, 0), ...
                'pitchRateDegS', zeros(1, 0), ...
                'yawRateDegS', zeros(1, 0), ...
                'errXDegS', zeros(1, 0), ...
                'errYDegS', zeros(1, 0), ...
                'errZDegS', zeros(1, 0), ...
                'isEstimated', zeros(1, 0), ...
                'accelX', zeros(1, 0), ...
                'accelY', zeros(1, 0), ...
                'accelZ', zeros(1, 0), ...
                'tempC', zeros(1, 0));
        end
    end
end
