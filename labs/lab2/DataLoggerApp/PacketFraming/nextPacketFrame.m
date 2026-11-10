function [frame, buffer, ready] = nextPacketFrame(buffer, payloadSize, encoded, terminated)
%nextPacketFrame - Extract one complete frame from a buffered byte stream
%   FRAME = nextPacketFrame(BUFFER,payloadSize,ENCODED,TERMINATED) extracts
%   the next frame without decoding its payload. BUFFER is a uint8 row,
%   payloadSize is the schema byte count, and both flags select framing.
%
%   [FRAME,BUFFER] = nextPacketFrame(...) also returns the unconsumed bytes.
%
%   [FRAME,BUFFER,READY] = nextPacketFrame(...) also indicates that one frame
%   was consumed. Append new bytes to returned BUFFER and call repeatedly.
%   An incomplete frame returns READY=false. An oversized terminated frame
%   returns uint8(0), which the COBS decoder rejects. Oversized fragments are
%   replaced with a bounded nonzero marker until their delimiter arrives.
%
%   See also cobsDecode, dataUnpack

% Normalize shape because serial and UDP readers can return different shapes.
frame = uint8([]);
ready = false;
buffer = reshape(uint8(buffer), 1, []);
% Null delimiters are safe only after COBS has removed payload zeros.
if terminated
    if ~encoded
        error("cobs:InvalidFraming", "Null framing requires COBS.");
    end
    z = find(buffer == 0, 1);
    if isempty(z)
        if numel(buffer) > payloadSize + floor(payloadSize/254) + 1
            % Preserve an invalid prefix, so a suffix cannot become a valid frame.
            buffer = ones(1, payloadSize + floor(payloadSize/254) + 2, "uint8");
        end
        return
    end
    frame = buffer(1:z-1);
    if numel(frame) > payloadSize + floor(payloadSize/254) + 1
        frame = uint8(0);
    end
    % Keep later frames intact, including multiple packets from one read.
    buffer = buffer(z+1:end);
else
    % Below 254 bytes, encoded length is payloadSize+1 for every payload.
    if encoded && payloadSize >= 254
        error("cobs:InvalidFraming", "COBS payloads >=254 bytes require null framing.");
    end
    n = payloadSize + double(encoded);
    if n < 1
        error("cobs:InvalidFraming", "Fixed frames must be nonempty.");
    end
    if numel(buffer) < n
        return
    end
    frame = buffer(1:n);
    buffer = buffer(n+1:end);
end
ready = true;
end
