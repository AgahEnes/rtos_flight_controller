function c = FrameConstants()
%FRAMECONSTANTS Telemetry frame constants from firmware telemetry_task.h.

c.sync0 = uint8(hex2dec('A5'));
c.sync1 = uint8(hex2dec('5A'));
c.msgIdImuVehicleState = uint8(hex2dec('12'));
c.msgIdImuCalibration = uint8(hex2dec('81'));

c.frameHeaderLength = uint16(4);
c.frameCrcLength = uint16(2);
c.timestampLength = uint16(4);
c.imuPayloadLength = uint16(28);
c.vehiclePayloadLength = uint16(25);
c.framePayloadLength = uint16(c.timestampLength + c.imuPayloadLength + c.vehiclePayloadLength);
c.frameLength = uint16(c.frameHeaderLength + c.framePayloadLength + c.frameCrcLength);
c.crcDataLength = uint16(c.frameLength - c.frameCrcLength);
c.stateFrameLength = c.frameLength;
c.stateCrcDataLength = c.crcDataLength;

c.calibrationPayloadLength = uint16(37);
c.calibrationFrameLength = uint16(c.frameHeaderLength + c.calibrationPayloadLength + c.frameCrcLength);
c.calibrationCrcDataLength = uint16(c.calibrationFrameLength - c.frameCrcLength);
c.maxFrameLength = uint16(max(double(c.stateFrameLength), double(c.calibrationFrameLength)));

% 1-based MATLAB indices mapped from firmware offsets.
c.idxSeq = uint16(4);
c.idxTimestampStart = uint16(5);
c.idxImuStart = uint16(9);
c.idxVehicleStart = uint16(37);
c.idxIsEstimated = uint16(61);
c.idxCrcStart = uint16(62);

c.idxCalTimestampStart = uint16(33);
c.idxCalUpdateCounterStart = uint16(37);
c.idxCalIsValid = uint16(41);
c.idxCalCrcStart = uint16(42);

% Payload field counts.
c.imuFloatCount = uint16(7);      % accel xyz, gyro xyz, temp
c.vehicleFloatCount = uint16(6);  % roll/pitch/yaw + rates
end
