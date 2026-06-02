function [sObj, cleanupObj] = openSerial(portName, baudRate, timeoutSec)
%OPENSERIAL Open serialport object with cleanup handler.

if nargin < 2 || isempty(baudRate)
    baudRate = 115200;
end
if nargin < 3 || isempty(timeoutSec)
    timeoutSec = 0.05;
end

sObj = serialport(portName, baudRate, "Timeout", timeoutSec);
cleanupObj = onCleanup(@()localCleanup(sObj)); %#ok<NASGU>
end

function localCleanup(sObj)
if ~isempty(sObj) && isvalid(sObj)
    flush(sObj);
    clear sObj;
end
end
