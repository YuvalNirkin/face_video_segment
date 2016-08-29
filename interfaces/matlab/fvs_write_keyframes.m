function fvs_write_keyframes(varargin)
%FVS_WRITE_KEYFRAMES(videoFile, outDir, landmarksFile, segmentationFile,
%   fvsFile, 'verbose', verbose):
%   videoFile - Path to input video file
%   outDir - Path to output directory
%   landmarksFile - Path to the landmarks model file or landmarks cache
%   file (.pb)
%   segmentationFile - Path to the segmentation protobuffer file (.pb)
%   fvsFile - Path to the face video segmentation file (.fvs)
%   verbose (=0) - output debug information level (0 means no debug)

%% Parse input arguments
p = inputParser;
addRequired(p, 'fvsFile', @ischar);
addRequired(p, 'outDir', @ischar);
addRequired(p, 'videoFile', @ischar);
addRequired(p, 'landmarksFile', @ischar);
addRequired(p, 'segmentationFile', @ischar);

addParameter(p, 'verbose', 0, @isscalar);
parse(p,varargin{:});

%% Execute local face video segmentation
exeName = 'fvs_write_keyframes';
[status, cmdout] = system([exeName ' "' p.Results.videoFile...
    '" -o "' p.Results.outDir '" -l "' p.Results.landmarksFile...
    '" -s "' p.Results.segmentationFile...
    '" -f "' p.Results.fvsFile...
    '" -v ' num2str(p.Results.verbose)]);
if(status ~= 0)
    error(cmdout);
end

end

