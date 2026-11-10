function loggerReleaseTransport(transport)
%loggerReleaseTransport - Release a logger transport on every cleanup path
%   loggerReleaseTransport(TRANSPORT) disables acquisition callbacks and
%   deletes a valid serialport/udpport object. Empty or already deleted
%   objects are accepted. Cleanup errors are reported independently so a
%   failure to disable callbacks does not prevent attempting deletion.
%   This function never sends a command to the remote device.
%
%   See also configureCallback, delete

if isempty(transport) || ~isvalid(transport)
    return
end
try
    configureCallback(transport, 'off');
catch err
    warning('cobs:CallbackCleanupFailed', '%s', err.message);
end
try
    delete(transport);
catch err
    warning('cobs:TransportCleanupFailed', '%s', err.message);
end
end
