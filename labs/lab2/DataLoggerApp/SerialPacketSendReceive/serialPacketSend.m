function serialPacketSend(sp, packetData, cobsEncoded, nullTerminated)
%serialPacketSend - Send a typed binary packet through a serial port
%   serialPacketSend(SP,packetData) serializes numeric cells in little-endian
%   order, COBS encodes the payload and appends a zero delimiter. SP is an
%   open serialport object; field order and classes define the wire schema.
%
%   serialPacketSend(...,cobsEncoded) optionally disables COBS encoding.
%
%   serialPacketSend(...,cobsEncoded,nullTerminated) optionally disables
%   the external delimiter. Raw zero-delimited data is rejected because
%   payload zeros would be ambiguous. Fixed COBS frames require agreement
%   on encoded length; use delimiters for payloads of 254 bytes or more.
%
%   See also serialPacketReceive, dataPack

%%
% Riccardo Antonello (riccardo.antonello@unipd.it)
%
% February 18, 2026
%
% Dept. of Information Engineering, University of Padova
%


%%  Validate input arguments

arguments
    sp  (1,1)   internal.Serialport
    packetData (1,:) cell {mustBeValidPacketData}
    cobsEncoded (1,1) logical = true
    nullTerminated (1,1) logical = true
end


if nullTerminated && ~cobsEncoded
    error("cobs:InvalidFraming", "Null framing requires COBS.");
end

%%  Compute packet size in bytes

%   supported data types names
typeName = {
    'double',   ...
    'single',   ...
    'int8',    ...
    'uint8',    ...
    'int16',    ...
    'uint16',    ...
    'int32',    ...
    'uint32'  };

%   supported data types byte sizes
typeByteSize = [
    8, ...
    4, ...
    1, ...
    1, ...
    2, ...
    2, ...
    4, ...
    4 ];

%   compute packet byte size
packetByteSize = 0;
packetCellsNum = numel(packetData);
for n = 1:packetCellsNum
    [~, k] = ismember(class(packetData{n}), typeName);
    packetByteSize = packetByteSize + numel(packetData{n}) * typeByteSize(k);
end


% Fixed-length COBS cannot delimit arbitrary long payloads unambiguously.
if cobsEncoded && ~nullTerminated && packetByteSize >= 254
    error("cobs:InvalidFraming", "COBS payloads >=254 bytes require null framing.");
end

%%  Serialize data 

%   preallocate buffer
serializedData = zeros(1, packetByteSize, 'uint8');

%   serialize data 
idx = 1;
for n = 1:packetCellsNum
    value = reshape(packetData{n}, 1, []);
    [~, ~, endian] = computer;
    if endian == 'B', value = swapbytes(value); end
    bytes = typecast(value, 'uint8');
    nextIdx = idx + numel(bytes);

    serializedData(idx:nextIdx-1) = bytes;
    idx = nextIdx;
end


%%  Encode data

if cobsEncoded
    serializedData = cobsEncode(serializedData);
end


%%  Terminate packet

if nullTerminated
    serializedData = [serializedData, uint8(0)];
end


%%  Send packet

%   write the serialized data to the serial port
write(sp, serializedData, 'uint8');


end


%%  Input arguments validation functions

function mustBeValidPacketData(c)

    validDTypeNames = [ ...
        "double", ...
        "single", ...
        "int8", ...
        "uint8", ...
        "int16", ...
        "uint16", ...
        "int32", ...
        "uint32" ];

    if ~iscell(c)
        error("packetData must be a cell array.");
    end

    for k = 1:numel(c)
        x = c{k};

        if isempty(x)
            error("packetData{%d} must not be empty.", k);
        end
        
        if ~isnumeric(x)
            error("packetData{%d} must be numeric (scalar or array).", k);
        end

        if ~ismember(string(class(x)), validDTypeNames)
            error("packetData{%d} has type '%s'. Allowed types: %s.", ...
                k, class(x), strjoin(validDTypeNames, ", "));
        end

        if ~isreal(x)
            error("packetData{%d} must be real-valued.", k);
        end

        if issparse(x)
            error("packetData{%d} must not be sparse.", k);
        end

        if (isfloat(x) && any(~isfinite(x), "all"))
            error("packetData{%d} must not contain NaN or Inf.", k);
        end

    end
end



