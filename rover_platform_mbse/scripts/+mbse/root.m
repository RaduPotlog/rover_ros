function r = root()
%ROOT Absolute path of the rover_platform_mbse folder.
r = fileparts(fileparts(fileparts(mfilename('fullpath'))));
end
