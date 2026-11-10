function packetData = serialPacketReceive(sp, packetSpec, cobsEncoded, nullTerminated)
%serialPacketReceive - Receive and unpack one binary serial packet
%   packetData = serialPacketReceive(SP,packetSpec) reads a zero-delimited
%   COBS frame from an open serialport object, validates it and decodes the
%   little-endian fields specified by packetSpec. Each output cell is typed.
%
%   packetData = serialPacketReceive(...,cobsEncoded) optionally disables
%   COBS decoding.
%
%   packetData = serialPacketReceive(...,cobsEncoded,nullTerminated) also
%   selects fixed-length reception when nullTerminated is false. Fixed
%   COBS payloads must be shorter than 254 bytes. Raw null framing is invalid.
%   A total receive deadline uses SP.Timeout. Malformed frames raise errors;
%   an oversized frame is drained to its delimiter before reporting failure.
%
%   See also serialPacketSend, dataUnpack

%%
% Riccardo Antonello (riccardo.antonello@unipd.it)
%
% February 19, 2026
%
% Dept. of Information Engineering, University of Padova
%


%%  Validate input arguments

arguments
    sp  (1,1)   internal.Serialport
    packetSpec (1,:) cell   %   content will be validated by getPacketInfo
    cobsEncoded (1,1) logical = true
    nullTerminated (1,1) logical = true
end


%%  Receive packet

%   get packet info
packetInfo = getPacketInfo(packetSpec);

if nullTerminated
    if ~cobsEncoded
        error("cobs:InvalidFraming", "Null framing requires COBS.");
    end
    frame = uint8([]);
    bound = packetInfo.byteSize + floor(packetInfo.byteSize/254) + 1;
    originalTimeout = sp.Timeout;
    restoreTimeout = onCleanup(@() restoreReceiveTimeout(sp, originalTimeout));
    start = tic;
    oversized = false;
    while true
        remaining = originalTimeout - toc(start);
        if remaining <= 0
            error("cobs:Timeout", "Timed out waiting for a complete frame.");
        end
        sp.Timeout = remaining;
        b = uint8(read(sp, 1, 'uint8'));
        if isempty(b)
            error("cobs:Timeout", "Timed out waiting for a complete frame.");
        end
        if b == 0
            if oversized
                error("cobs:FrameTooLong", "Received frame exceeds packet bound.");
            end
            break
        end
        % Keep memory bounded while draining an invalid frame to its delimiter.
        if ~oversized
            frame(end+1) = b; %#ok<AGROW>
            oversized = numel(frame) > bound;
        end
    end
else
    if cobsEncoded && packetInfo.byteSize >= 254
        error("cobs:InvalidFraming", "COBS payloads >=254 bytes require null framing.");
    end
    frame = uint8(read(sp, packetInfo.byteSize + double(cobsEncoded), 'uint8'));
end
frame = reshape(frame, 1, []);

%   decode
if cobsEncoded
    frame = cobsDecode(frame);
end

%   unpack data
packetData = dataUnpack(frame, packetInfo);

end


function restoreReceiveTimeout(sp, timeout)
%restoreReceiveTimeout - Preserve the caller's connection configuration
if isvalid(sp)
    sp.Timeout = timeout;
end
end
