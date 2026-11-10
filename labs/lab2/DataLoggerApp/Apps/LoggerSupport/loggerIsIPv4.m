function valid = loggerIsIPv4(address)
%loggerIsIPv4 - Validate four decimal IPv4 octets without opening a socket
%   VALID = loggerIsIPv4(ADDRESS) accepts text with four integers in [0,255].
%   Empty octets, signs, whitespace, leading zeros and trailing text fail.
%   This shared predicate keeps constructor and UI validation consistent.
%
%   See also split, str2double

address = string(address);
valid = isscalar(address) && ~ismissing(address);
if ~valid
    return
end
parts = split(address, '.');
valid = numel(parts) == 4 && all(matches(parts,regexpPattern('0|[1-9][0-9]{0,2}'))) ...
    && all(str2double(parts) <= 255);
end
