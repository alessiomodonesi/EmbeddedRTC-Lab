function buffer = loggerBufferMerge(buffer, capacity, pending)
%loggerBufferMerge - Resize or append samples to a bounded logger buffer
%   BUFFER = loggerBufferMerge(BUFFER, CAPACITY) retains the newest samples
%   and allocates exactly CAPACITY columns in every signal matrix.
%   BUFFER = loggerBufferMerge(BUFFER, CAPACITY, PENDING) appends all written
%   samples from PENDING first. This also includes a partially filled batch.
%   sampleIndex NaNs mark unused columns; signal NaNs remain valid data.
%   tailPos is always the next free column, or CAPACITY+1 for a full buffer.
%
%   See also loggerBufferExport

validateattributes(capacity, {'numeric'}, {'scalar','real','finite','integer','positive'});
if nargin < 3
    pending = buffer;
    pending.sampleIndex = [];
    pending.time = [];
    pending.signals = cell(size(buffer.signals));
    for k = 1:numel(buffer.signals)
        pending.signals{k} = zeros(size(buffer.signals{k},1),0);
    end
end
old = find(~isnan(buffer.sampleIndex));
new = find(~isnan(pending.sampleIndex));
indices = [buffer.sampleIndex(old), pending.sampleIndex(new)];
times = [buffer.time(old), pending.time(new)];
% Selecting after concatenation handles partial fills, oversized incoming
% batches and rollover at capacities that are not refresh-ratio multiples.
keep = max(1, numel(indices)-capacity+1):numel(indices);
count = numel(keep);
for k = 1:numel(buffer.signals)
    samples = [buffer.signals{k}(:,old), pending.signals{k}(:,new)];
    buffer.signals{k} = nan(size(samples,1),capacity);
    buffer.signals{k}(:,1:count) = samples(:,keep);
end
buffer.sampleIndex = nan(1,capacity);
buffer.time = nan(1,capacity);
buffer.sampleIndex(1:count) = indices(keep);
buffer.time(1:count) = times(keep);
buffer.size = capacity;
buffer.tailPos = count+1;
end
