function add_pascal_voc_db(varargin)
%ADD_PASCAL_VOC_DB Summary of this function goes here
%   Detailed explanation goes here

%% Parse input arguments
p = inputParser;
addRequired(p, 'imgDir', @ischar);
addRequired(p, 'segDir', @ischar);
addRequired(p, 'trainvalDir', @ischar);
addRequired(p, 'dbDir', @ischar);
parse(p,varargin{:});

%% Calculate paths
inTrainPath = fullfile(p.Results.trainvalDir, 'train.txt');
inValPath = fullfile(p.Results.trainvalDir, 'val.txt');
outImgPath = fullfile(p.Results.dbDir, 'JPEGImages');
outSegPath = fullfile(p.Results.dbDir, 'SegmentationClass');
outTrainPath = fullfile(p.Results.dbDir, 'ImageSets', 'Segmentation', 'train.txt');
outValPath = fullfile(p.Results.dbDir, 'ImageSets', 'Segmentation', 'val.txt');
outMainTrainPath = fullfile(p.Results.dbDir, 'ImageSets', 'Main', 'train.txt');
outMainValPath = fullfile(p.Results.dbDir, 'ImageSets', 'Main', 'val.txt');
outPersonTrainPath = fullfile(p.Results.dbDir, 'ImageSets', 'Main', 'person_train.txt');
outPersonValPath = fullfile(p.Results.dbDir, 'ImageSets', 'Main', 'person_val.txt');

%% Modify trainval files
train = readTextFile(inTrainPath);
val = readTextFile(inValPath);
writeTextFile(outTrainPath, train);
writeTextFile(outValPath, val);
copyfile(outTrainPath, outMainTrainPath, 'f');
copyfile(outValPath, outMainValPath, 'f');
copyfile(outTrainPath, outPersonTrainPath, 'f');
copyfile(outValPath, outPersonValPath, 'f');

%% Parse image directory
filt = '.*(png|jpg)';
imgDescs = dir(p.Results.imgDir);
%imgNames = {imgDescs.name};
imgNames = {imgDescs(~cellfun(@isempty,regexpi({imgDescs.name},filt))).name};

%% Parse segmentation directory
filt = '.*(png|jpg)';
segDescs = dir(p.Results.segDir);
%segNames = {segDescs.name};
segNames = {segDescs(~cellfun(@isempty,regexpi({segDescs.name},filt))).name};

%% Copy input images
for i = 1:numel(imgNames)
    inFilePath = fullfile(p.Results.imgDir, imgNames{i});
    outFilePath = fullfile(outImgPath, imgNames{i});
    copyfile(inFilePath, outFilePath);
end

%% Copy segmentation images
for i = 1:numel(segNames)
    inFilePath = fullfile(p.Results.segDir, segNames{i});
    outFilePath = fullfile(outSegPath, segNames{i});
    copyfile(inFilePath, outFilePath);
end

end

function names = readTextFile(textFile)
    fileID = fopen(textFile);
    names = textscan(fileID,'%s');
    names = names{1};
    fclose(fileID);
end

function writeTextFile(textFile, set)
    fid = fopen(textFile,'a');
    for i = 1:length(set)
         fprintf(fid, '%s\n', set{i});
    end
    fclose(fid);
end