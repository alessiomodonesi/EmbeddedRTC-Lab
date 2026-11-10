function validatePacketInfo(info)
%validatePacketInfo - Check the consistency of a numeric packet layout
%   validatePacketInfo(INFO) rejects missing fields, unsupported types,
%   invalid widths and inconsistent byte counts. This protects dataPack and
%   dataUnpack when a caller supplies manually constructed or edited metadata.
%
%   See also getPacketInfo, dataPack, dataUnpack

required = {'byteSize','cellsNum','cellDType','cellDTypeSize','cellWidth'};
if ~isscalar(info) || ~isstruct(info) || ~all(isfield(info, required))
    error('cobs:InvalidPacketInfo', 'Packet metadata is incomplete.');
end
names = {'double','single','int8','uint8','int16','uint16','int32','uint32'};
sizes = [8 4 1 1 2 2 4 4];
n = info.cellsNum;
if ~isnumeric(n) || ~isscalar(n) || ~isreal(n) || ~isfinite(n) || n < 0 || fix(n) ~= n
    error('cobs:InvalidPacketInfo', 'Invalid cell count.');
end
if ~iscell(info.cellDType) || numel(info.cellDType) ~= n || ...
        ~isnumeric(info.cellWidth) || ~isnumeric(info.cellDTypeSize) || ...
        numel(info.cellWidth) ~= n || numel(info.cellDTypeSize) ~= n
    error('cobs:InvalidPacketInfo', 'Metadata lengths must match cellsNum.');
end
for k = 1:n
    type = info.cellDType{k};
    if ~(ischar(type) && isrow(type)) && ~(isstring(type) && isscalar(type))
        error('cobs:InvalidPacketInfo', 'Invalid field type.');
    end
    [supported, idx] = ismember(char(type), names);
    width = info.cellWidth(k);
    if ~supported || ~isreal(width) || ~isfinite(width) || width < 1 || ...
            fix(width) ~= width || info.cellDTypeSize(k) ~= sizes(idx)
        error('cobs:InvalidPacketInfo', 'Invalid field type, width or size.');
    end
end
bytes = sum(double(info.cellWidth(:)) .* double(info.cellDTypeSize(:)));
if ~isnumeric(info.byteSize) || ~isscalar(info.byteSize) || ...
        ~isreal(info.byteSize) || ~isfinite(info.byteSize) || ...
        info.byteSize ~= bytes || bytes > flintmax
    error('cobs:InvalidPacketInfo', 'Inconsistent packet byte size.');
end
end
