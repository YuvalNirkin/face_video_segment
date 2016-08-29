function fvs_init_trainval(inDir, outFile)
%FVS_INIT_EMPTY_TRAINVAL Summary of this function goes here
%   Detailed explanation goes here


%% Parse input directory
fileDescs = dir(inDir);
dirNames = {fileDescs([fileDescs.isdir]).name};
dirNames = dirNames(3:end);

%% Initialize sheet cell array
M = cell(length(dirNames), 3);
M(:,1) = dirNames;

%% Write to file
T = cell2table(M, 'VariableNames',{'Name', 'Target', 'Notes'});
writetable(T, outFile);