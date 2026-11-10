function setup()
%SETUP - Add the maintained MATLAB routines and apps to the search path
%   SETUP resolves folders relative to this file and adds only maintained
%   routine and app directories. It does not change the working directory,
%   build MEX files, connect hardware, or save the MATLAB path permanently.
%
%   See also addpath, fileparts

% Explicit paths prevent legacy copies and example scripts from shadowing
% the canonical implementations. Add this directory, then call setup once.
base = fileparts(mfilename('fullpath'));
folders = { ...
    'CobsEncodeDecode', ...
    'DataPackUnpack', ...
    'PacketFraming', ...
    'SerialPacketSendReceive', ...
    fullfile('Apps','LoggerSupport'), ...
    fullfile('Apps','SerialDataLogger'), ...
    fullfile('Apps','UdpDataLogger') };

for k = 1:numel(folders)
    addpath(fullfile(base, folders{k}));
end

end
