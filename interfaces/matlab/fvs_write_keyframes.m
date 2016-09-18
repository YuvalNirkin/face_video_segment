function fvs_write_keyframes(varargin)
%FVS_WRITE_KEYFRAMES(fvsFile, outDir, 'max_scale', max_scale,
%   'upscale', upscale, 'verbose', verbose):
%   fvsFile - Path to the face video segmentation file (.fvs). The paths
%   for video, segmentation and landmarks will be taken from this file.
%   outDir - Path to output directory
%   max_scale (=0) - Maximum width or height of the keyframes [pixels].
%   upscale (=0) - If true all keyframes will be upscaled up to max_scale.
%   debug (=0) - output debug information level (0 means no debug)
%
%FVS_WRITE_KEYFRAMES(fvsFile, outDir, videoFile, segmentationFile,
%   landmarksFile):
%   videoFile - Path to input video file
%   segmentationFile - Path to the segmentation protobuffer file (.pb)
%   landmarksFile - Path to the landmarks model file or landmarks cache
%   file (.pb)

%% Parse input arguments
p = inputParser;
addRequired(p, 'fvsFile', @ischar);
addRequired(p, 'outDir', @ischar);
if(mod(nargin, 2) == 1)
    addRequired(p, 'videoFile', @ischar);
    addRequired(p, 'segmentationFile', @ischar);
    addRequired(p, 'landmarksFile', @ischar);
end
addParameter(p, 'max_scale', 0, @isscalar);
addParameter(p, 'upscale', 0, @isscalar);
addParameter(p, 'debug', 0, @isscalar);
parse(p,varargin{:});

%% Execute face video segmentation write keyframes
exeName = 'fvs_write_keyframes';
cmd = [exeName ' "' p.Results.fvsFile '"'...
    ' -o "' p.Results.outDir '"'...
    ' -d ' num2str(p.Results.debug)];
if(mod(nargin, 2) == 1)
    cmd = [cmd  ' -v "' p.Results.videoFile '"'...
    ' -s "' p.Results.segmentationFile '"'...
    ' -l "' p.Results.landmarksFile '"'];
end
if(p.Results.max_scale > 0)
     cmd = [cmd  ' -m ' num2str(p.Results.max_scale)];
end
if(p.Results.upscale > 0)
     cmd = [cmd  ' -u'];
end
[status, cmdout] = system(cmd);
if(status ~= 0)
    error(cmdout);
end
end
