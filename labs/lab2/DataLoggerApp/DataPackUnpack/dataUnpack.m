function unpackedData = dataUnpack(packedData, packetInfo)
%dataUnpack - Recover typed numeric cells from little-endian payload bytes
%   unpackedData = dataUnpack(packedData,packetInfo) decodes a uint8 row
%   using a schema produced by getPacketInfo. The byte count must match
%   exactly. Each output cell is a row with the declared class and width.
%   The input must already have its delimiter removed and COBS decoded.
%
%   See also dataPack, getPacketInfo

%%
% Riccardo Antonello (riccardo.antonello@unipd.it)
%
% February 18, 2026
%
% Dept. of Information Engineering, University of Padova
%


%%  Validate input arguments

arguments
    packedData (1,:) uint8
    packetInfo (1,1) struct {mustBeValidPacketInfo}
end


%%  Data unpack

%   init unpacked data cell array
unpackedDataLen = packetInfo.cellsNum;
unpackedData = cell(1, unpackedDataLen);

%   init packed data length 
packedDataLen = length(packedData);

%   packedData length must be equal to the byte size specified in packetInfo
if packedDataLen ~= packetInfo.byteSize
    error("dataUnpack:invalidPackedDataSize", ...
        "The length of packed data array must be equal to the byte size specified in packetInfo.");    
end

%   start index to packed data of current cell
inIdx = 1;

%   unpack data
for n = 1:unpackedDataLen

    %   start index to packed data of next cell    
    nextInIdx = inIdx + packetInfo.cellDTypeSize(n) * packetInfo.cellWidth(n);
    
    %   unpack data
    inData = packedData( inIdx:(nextInIdx-1) );
    outData = typecast( uint8(inData), packetInfo.cellDType{n});
    
    [~, ~, endian] = computer;
    if endian == 'B', outData = swapbytes(outData); end

    %   update start index
    inIdx = nextInIdx;

    %   store unpacked data to output cell array
    unpackedData{n} = outData;
    
end

end


%%  Input arguments validation functions

function mustBeValidPacketInfo(s)

validatePacketInfo(s);

end
