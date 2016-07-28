function seg_viewer(segFile)
%SEG_VIEWER View segmentation files
%   SEG_VIEWER(segFile) will view the segmentation file, segFile.

%% Execute segmentation viewer
exeName = 'segment_viewer';
[status, cmdout] = system([exeName ' --input="' segFile...
    '" --window_name=seg_viewer']);
if(status ~= 0)
    error(cmdout);
end

end

