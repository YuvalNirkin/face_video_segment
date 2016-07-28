function face_video_seg(varargin)
%FACE_VIDEO_SEG Summary of this function goes here
%   FACE_VIDEO_SEG(videoFile, outDir, landmarksFile, segmentationFile,
%   'verbose', verbose):
%   videoFile - Path to input video file
%   outDir - Path to output directory
%   landmarksFile - Path to the landmarks model file or landmarks cache
%   file (.pb)
%   segmentationFile - Path to the segmentation protobuffer file (.pb)
%   verbose (=0) - output debug information level (0 means no debug)

%% Parse input arguments
p = inputParser;
addRequired(p, 'videoFile', @ischar);
addRequired(p, 'outDir', @ischar);
addRequired(p, 'landmarksFile', @ischar);
addRequired(p, 'segmentationFile', @ischar);
addParameter(p, 'verbose', 0, @isscalar);
parse(p,varargin{:});

%% Execute face video segmentation
exeName = 'face_video_segment';
[status, cmdout] = system([exeName ' "' p.Results.videoFile...
    '" -o "' p.Results.outDir '" -l "' p.Results.landmarksFile...
    '" -s "' p.Results.segmentationFile...
    '" -v ' num2str(p.Results.verbose)]);
if(status ~= 0)
    error(cmdout);
end

end

