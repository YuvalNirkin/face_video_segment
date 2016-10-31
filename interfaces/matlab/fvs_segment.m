function fvs_segment(varargin)
%FVS_SEGMENT Create video segmentation tree
%   FVS_SEGMENT(videoFile, 'outFile', outFile, 'preview', preview):
%   videoFile - Path to input video file
%   outFile - Path to output directory
%   preview (=-1) - If >= 0, display segmentation at the specified level
%   [0, 1]

%% Parse input arguments
p = inputParser;
addRequired(p, 'videoFile', @ischar);
addParameter(p, 'outFile', '', @ischar);
addParameter(p, 'preview', -1, @isscalar);
parse(p,varargin{:});

%% Execute segmentation
cmd = [mfilename ' "' p.Results.videoFile '"'...
    ' -p ' num2str(p.Results.preview)];
if(~isempty(p.Results.outFile))
    cmd = [cmd  ' -o "' p.Results.outFile '"'];
end
[status, cmdout] = system(cmd);
if(status ~= 0)
    error(sprintf(['error in seg_tree_sample executable:\n' cmdout]));
end
end
