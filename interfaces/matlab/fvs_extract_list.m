function fvs_extract_list(inDir, outDir, listFile)
%FVS_EXTRACT_LIST Summary of this function goes here
%   Detailed explanation goes here

%% Read list
include = readList(listFile);

%% Extract trainval file
trainvalFile = fullfile(inDir, 'trainval.csv');
if(exist(trainvalFile) == 2)
    T = readtable(trainvalFile,'Delimiter',';','Format', '%s%s%s%s');
    names = table2cell(T(:,1));
    [~,table_ind,~] = intersect(names,include);
    
    % Write intersection table
    outTrainvalFile = fullfile(outDir, 'trainval.csv');
    writetable(T(table_ind, :), outTrainvalFile, 'Delimiter', ';');
end

%% Parse input directory
fileDescs = dir(inDir);
dirNames = {fileDescs([fileDescs.isdir]).name};
dirNames = dirNames(3:end);

%% Copy intersection directories
[~,dir_ind,~] = intersect(dirNames,include);
for i = dir_ind'
    disp(['Copying ' dirNames{i}]);
    inDirPath = fullfile(inDir, dirNames{i});
    outDirPath = fullfile(outDir, dirNames{i});
    copyfile(inDirPath, outDirPath);
end

%% Parse video files
filt = '^.*\.(avi|mp4|mkv)$';
vidNames = {fileDescs(~cellfun(@isempty,regexpi({fileDescs.name},filt))).name};
vidNamesNoExt = cellfun(@(x) x(1:end-4), vidNames, 'un', 0);

%% Copy intersection videos
[~,vid_ind,~] = intersect(vidNamesNoExt,include);
for i = vid_ind'
    disp(['Copying ' vidNames{i}]);
    inDirPath = fullfile(inDir, vidNames{i});
    outDirPath = fullfile(outDir, vidNames{i});
    copyfile(inDirPath, outDirPath);
end

end

function names = readList(filename)
    fileID = fopen(filename);
    names = textscan(fileID,'%s');
    names = names{1};
    fclose(fileID);
end
