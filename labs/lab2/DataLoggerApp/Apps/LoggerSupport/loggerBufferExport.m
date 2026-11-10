function data = loggerBufferExport(buffer)
%loggerBufferExport - Produce a compact workspace snapshot of logger data
%   DATA = loggerBufferExport(BUFFER) removes only unwritten sample columns
%   and the internal tailPos field. NaN signal values are preserved. Call
%   loggerBufferMerge with the pending batch before exporting so samples
%   waiting for the next plot refresh are included without modifying buffers.
%
%   See also loggerBufferMerge

keep = ~isnan(buffer.sampleIndex);
data = rmfield(buffer, 'tailPos');
data.sampleIndex = buffer.sampleIndex(keep);
data.time = buffer.time(keep);
for k = 1:numel(buffer.signals)
    data.signals{k} = buffer.signals{k}(:,keep);
end
data.size = nnz(keep);
end
