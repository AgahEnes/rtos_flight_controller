classdef LivePlot < handle
    %LIVEPLOT Throttled telemetry visualization (4 panels).

    properties
        enabled logical = true
        windowSec double = 10
        plotHz double = 10
        maxPoints double = 300
    end

    properties (SetAccess = private)
        fig
        isClosed logical = false
    end

    properties (Access = private)
        tLastDraw uint64 = uint64(0)
        ax1
        ax2
        ax3
        ax4
        hRoll
        hPitch
        hYaw
        hGx
        hGy
        hGz
        hRR
        hPR
        hYR
        hEx
        hEy
        hEz
        hEst
        hAx
        hAy
        hAz
        hTemp
    end

    methods
        function obj = LivePlot(cfg)
            if nargin >= 1 && isstruct(cfg)
                obj.applyCfg(cfg);
            end

            if ~obj.enabled
                return;
            end

            obj.buildFigure();
            obj.tLastDraw = tic;
        end

        function maybeUpdate(obj, ring)
            if ~obj.enabled || obj.isClosed
                return;
            end
            if isempty(obj.fig) || ~isvalid(obj.fig)
                obj.isClosed = true;
                return;
            end
            if toc(obj.tLastDraw) < (1 / obj.plotHz)
                return;
            end

            s = ring.getWindow(obj.windowSec, obj.maxPoints);
            if isempty(s.tSec)
                obj.tLastDraw = tic;
                return;
            end

            set(obj.hRoll, 'XData', s.tSec, 'YData', s.rollDeg);
            set(obj.hPitch, 'XData', s.tSec, 'YData', s.pitchDeg);
            set(obj.hYaw, 'XData', s.tSec, 'YData', s.yawDeg);

            set(obj.hGx, 'XData', s.tSec, 'YData', s.gyroXDegS);
            set(obj.hGy, 'XData', s.tSec, 'YData', s.gyroYDegS);
            set(obj.hGz, 'XData', s.tSec, 'YData', s.gyroZDegS);
            set(obj.hRR, 'XData', s.tSec, 'YData', s.rollRateDegS);
            set(obj.hPR, 'XData', s.tSec, 'YData', s.pitchRateDegS);
            set(obj.hYR, 'XData', s.tSec, 'YData', s.yawRateDegS);

            set(obj.hEx, 'XData', s.tSec, 'YData', s.errXDegS);
            set(obj.hEy, 'XData', s.tSec, 'YData', s.errYDegS);
            set(obj.hEz, 'XData', s.tSec, 'YData', s.errZDegS);
            errMax = max([0.2, s.errXDegS, s.errYDegS, s.errZDegS]);
            set(obj.hEst, 'XData', s.tSec, 'YData', s.isEstimated .* errMax);

            set(obj.hAx, 'XData', s.tSec, 'YData', s.accelX);
            set(obj.hAy, 'XData', s.tSec, 'YData', s.accelY);
            set(obj.hAz, 'XData', s.tSec, 'YData', s.accelZ);
            set(obj.hTemp, 'XData', s.tSec, 'YData', s.tempC);

            if numel(s.tSec) >= 2
                xLimits = [s.tSec(1), s.tSec(end)];
                xlim(obj.ax1, xLimits);
                xlim(obj.ax2, xLimits);
                xlim(obj.ax3, xLimits);
                xlim(obj.ax4, xLimits);
            end

            drawnow limitrate nocallbacks;
            obj.tLastDraw = tic;
        end
    end

    methods (Access = private)
        function applyCfg(obj, cfg)
            if isfield(cfg, 'EnablePlot')
                obj.enabled = logical(cfg.EnablePlot);
            end
            if isfield(cfg, 'WindowSec')
                obj.windowSec = double(cfg.WindowSec);
            end
            if isfield(cfg, 'PlotHz')
                obj.plotHz = double(cfg.PlotHz);
            end
            if isfield(cfg, 'MaxPlotPoints')
                obj.maxPoints = double(cfg.MaxPlotPoints);
            end
        end

        function buildFigure(obj)
            obj.fig = figure('Name', 'Telemetry Live (Sifirdan)', 'NumberTitle', 'off');
            obj.fig.CloseRequestFcn = @(src, ~)obj.onFigureClose(src);
            tl = tiledlayout(obj.fig, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

            obj.ax1 = nexttile(tl, 1); hold(obj.ax1, 'on'); grid(obj.ax1, 'on');
            title(obj.ax1, 'Attitude (deg)');
            ylabel(obj.ax1, 'deg');
            obj.hRoll = plot(obj.ax1, nan, nan, 'DisplayName', 'roll');
            obj.hPitch = plot(obj.ax1, nan, nan, 'DisplayName', 'pitch');
            obj.hYaw = plot(obj.ax1, nan, nan, 'DisplayName', 'yaw');
            legend(obj.ax1, 'show', 'Location', 'northwest');

            obj.ax2 = nexttile(tl, 2); hold(obj.ax2, 'on'); grid(obj.ax2, 'on');
            title(obj.ax2, 'Rates (raw solid / estimated dashed)');
            ylabel(obj.ax2, 'deg/s');
            obj.hGx = plot(obj.ax2, nan, nan, '-', 'DisplayName', 'gyroX raw');
            obj.hGy = plot(obj.ax2, nan, nan, '-', 'DisplayName', 'gyroY raw');
            obj.hGz = plot(obj.ax2, nan, nan, '-', 'DisplayName', 'gyroZ raw');
            obj.hRR = plot(obj.ax2, nan, nan, '--', 'DisplayName', 'rollRate est');
            obj.hPR = plot(obj.ax2, nan, nan, '--', 'DisplayName', 'pitchRate est');
            obj.hYR = plot(obj.ax2, nan, nan, '--', 'DisplayName', 'yawRate est');
            legend(obj.ax2, 'show', 'Location', 'northwest');

            obj.ax3 = nexttile(tl, 3); hold(obj.ax3, 'on'); grid(obj.ax3, 'on');
            title(obj.ax3, 'Residual |estimated - raw|');
            ylabel(obj.ax3, 'deg/s');
            obj.hEx = plot(obj.ax3, nan, nan, 'DisplayName', '|dX|');
            obj.hEy = plot(obj.ax3, nan, nan, 'DisplayName', '|dY|');
            obj.hEz = plot(obj.ax3, nan, nan, 'DisplayName', '|dZ|');
            obj.hEst = plot(obj.ax3, nan, nan, ':k', 'DisplayName', 'isEstimated');
            legend(obj.ax3, 'show', 'Location', 'northwest');

            obj.ax4 = nexttile(tl, 4); hold(obj.ax4, 'on'); grid(obj.ax4, 'on');
            title(obj.ax4, 'IMU (accel + temp)');
            xlabel(obj.ax4, 'time (s)');
            yyaxis(obj.ax4, 'left');
            ylabel(obj.ax4, 'm/s^2');
            obj.hAx = plot(obj.ax4, nan, nan, 'DisplayName', 'accelX');
            obj.hAy = plot(obj.ax4, nan, nan, 'DisplayName', 'accelY');
            obj.hAz = plot(obj.ax4, nan, nan, 'DisplayName', 'accelZ');
            yyaxis(obj.ax4, 'right');
            ylabel(obj.ax4, 'degC');
            obj.hTemp = plot(obj.ax4, nan, nan, 'k--', 'DisplayName', 'temp');
            legend(obj.ax4, 'show', 'Location', 'northwest');
        end

        function onFigureClose(obj, src)
            obj.isClosed = true;
            delete(src);
        end
    end
end
