function seg_tree(varargin)
%SEG_TREE Create segmentation tree
%   SEG_TREE(videoFile, outDir, 'verbose', verbose):
%   videoFile - Path to input video file
%   outDir - Path to output directory
%   verbose (=-1) - If >= 0, display segmentation at the specified level
%   [0, 1]

%% Parse input arguments
p = inputParser;
addRequired(p, 'videoFile', @ischar);
addRequired(p, 'outDir', @ischar);
addParameter(p, 'verbose', -1, @isscalar);
parse(p,varargin{:});

%% Execute face motion segmentation
exeName = 'seg_tree_sample';
[status, cmdout] = system([exeName ' --input_file="' p.Results.videoFile...
    '" --display="' num2str(p.Results.verbose)...
    '" --write_to_file --use_pipeline']);
if(status ~= 0)
    %error(cmdout);
    error(sprintf(['error in seg_tree_sample executable:\n' cmdout]));
end

seg_path = [p.Results.videoFile '.pb'];
if(~exist(seg_path, 'file') == 2)
    error(['Failed to create segmentation tree for "' p.Results.videoFile '"']);
end

%% Copy segmentation to output directory
movefile(seg_path, p.Results.outDir);

end
