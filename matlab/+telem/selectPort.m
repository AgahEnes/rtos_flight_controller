function sPort = selectPort(preferredPort)
%SELECTPORT Select a serial port for telemetry stream.

asPorts = serialportlist("available");
if isempty(asPorts)
    error('selectPort:NoPort', 'No serial ports are available.');
end

if nargin >= 1 && strlength(string(preferredPort)) > 0
    sPreferred = string(preferredPort);
    if any(asPorts == sPreferred)
        sPort = sPreferred;
        return;
    end
    error('selectPort:MissingPreferred', 'Requested port not found: %s', sPreferred);
end

asLower = lower(asPorts);
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
