function packedData = dataPack(unpackedData, packetInfo)
%dataPack - Serialize typed numeric cells into little-endian payload bytes
%   packedData = dataPack(unpackedData,packetInfo) concatenates real, full
%   numeric cells according to a schema produced by getPacketInfo. Cell
%   count, numeric classes and widths must match. No padding, field names,
%   delimiter or COBS bytes are added. IEEE floating-point bits are retained.
%
%   See also dataUnpack, getPacketInfo

%%
% Riccardo Antonello (riccardo.antonello@unipd.it)
%
% February 18, 2026
%
% Dept. of Information Engineering, University of Padova
%


%%  Validate input arguments

arguments
    unpackedData (1,:) cell
    packetInfo (1,1) struct {mustBeValidPacketInfo}
end


if numel(unpackedData) ~= packetInfo.cellsNum
    error("dataPack:invalidCellCount", "Cell count must match packetInfo.");
end

%%  Pack data  

%   init packed data array
packedDataLen = packetInfo.byteSize;
packedData = zeros(1, packedDataLen, 'uint8');

%   start index to packed data of current cell element
inIdx = 1;

for n = 1:packetInfo.cellsNum
    
    %   cell must be numeric and not empty
    if ~isnumeric(unpackedData{n}) || isempty(unpackedData{n})
        error("dataPack:invalidCell", ...
            "unpackedData{%d} must be non-empty numeric.", n);
    end

    if ~isreal(unpackedData{n}) || issparse(unpackedData{n})
        error("dataPack:invalidCell", "Cells must be real and full.");
    end

    %   cell must match the specified type
    if ~isa(unpackedData{n}, packetInfo.cellDType{n})
        error("dataPack:unmatchedDType", ...
            "unpackedData{%d} has type '%s' but expected '%s'.", ...
            n, class(unpackedData{n}), packetInfo.cellDType{n});
    end
    
    %   cell must match the specified width
    if numel(unpackedData{n}) ~= packetInfo.cellWidth(n)
        error("dataPack:unmatchedWidth", ...
            "unpackedData{%d} has %d element(s) but expected %d.", ...
            n, numel(unpackedData{n}), packetInfo.cellWidth(n));
    end
    
    %   pack data
    for m = 1:packetInfo.cellWidth(n)
        
        %   start index to packed data of next cell element
        nextInIdx = inIdx + packetInfo.cellDTypeSize(n);
        
        %   pack single cell element
        value = unpackedData{n}(m);
        [~, ~, endian] = computer;
        if endian == 'B', value = swapbytes(value); end
        packedData(inIdx:nextInIdx-1) = typecast(value, 'uint8');
        
        %   update start index
        inIdx = nextInIdx;
        
    end
    
end

end


%%  Input arguments validation functions

function mustBeValidPacketInfo(s)

validatePacketInfo(s);

end
