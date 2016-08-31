function produce_pascal_voc_db(varargin)
%PRODUCE_PASCAL_VOC_DB Summary of this function goes here
%   Detailed explanation goes here

%% Parse input arguments
p = inputParser;
addRequired(p, 'inDir', @ischar);
addRequired(p, 'outDir', @ischar);
addParameter(p, 'indices', [], @isvector);
addParameter(p, 'max_scale', 0, @isscalar);
addParameter(p, 'upscale', 0, @isscalar);
parse(p,varargin{:});
indices = p.Results.indices;

%% Create output directory structure
if(~exist(p.Results.outDir, 'dir'))
    mkdir(p.Results.outDir)
end
framesPath = fullfile(p.Results.outDir, 'JPEGImages');
segmentationsPath = fullfile(p.Results.outDir, 'SegmentationClass');
imageSetsMainPath = fullfile(p.Results.outDir, 'ImageSets', 'Main');
imageSetsSegPath = fullfile(p.Results.outDir, 'ImageSets', 'Segmentation');
if(~exist(framesPath, 'dir'))
    mkdir(framesPath);
end
if(~exist(segmentationsPath, 'dir'))
    mkdir(segmentationsPath);
end
if(~exist(imageSetsMainPath, 'dir'))
    mkdir(imageSetsMainPath);
end
if(~exist(imageSetsSegPath, 'dir'))
    mkdir(imageSetsSegPath);
end

%% Parse input directory
fileDescs = dir(p.Results.inDir);
dirNames = {fileDescs([fileDescs.isdir]).name};
dirNames = dirNames(3:end);
if(isempty(indices))
    indices = 1:length(dirNames);
elseif(max(indices) > length(dirNames) || min(indices) < 1)
    error(['indices must be from 1 to ' num2str(length(dirNames))]);
end

%% Read trainval file
trainvalFile = fullfile(p.Results.inDir, 'trainval.csv');
T = readtable(trainvalFile,'Delimiter',';','Format', '%s%s%s');
names = table2cell(T(:,1));
targets = table2cell(T(:,2));
if(length(dirNames) ~= length(names))
    error(['There is a mismatch between the trainval file '...
        'and number of directories in "' p.Results.inDir '".']);
end

%% Initialize training and valuation set
train = [];
val = [];

%% For each directory
cmap = labelColors();
for i = indices   
    dirPath = fullfile(p.Results.inDir, dirNames{i});
    if(~strcmp(dirNames{i}, names{i}))
        error(['There is a mismatch between the trainval file '...
        'and the directories!']);
    end
    if(~any(strcmp(targets{i}, {'train','val'})))
        disp(['Skipping "' dirNames{i} '"']);
        continue;
    end
    disp(['Processing "' dirNames{i} '"']);
    
    %% Parse current directory
    fileDescs = dir(fullfile(dirPath, '*frame*'));
    srcFrames = {fileDescs.name};
    fileDescs = dir(fullfile(dirPath, '*seg*'));
    srcSegmentations = {fileDescs.name};
    
    %% Copy files
    dstNames = calc_dst_names(srcFrames);
    process_files(srcFrames, dstNames, dirPath, framesPath, 'bicubic');
    process_files(srcSegmentations, dstNames, dirPath,...
        segmentationsPath, 'nearest');
    
    %% Add images to training and valuation sets
    if(strcmp(targets{i}, 'train'))
        train = [train; dstNames];
    else    % val
        val = [val; dstNames];
    end
end

%% Write training and valuation sets to files
write_to_file(fullfile(imageSetsMainPath, 'train.txt'), train);
write_to_file(fullfile(imageSetsSegPath, 'train.txt'), train);
write_to_file(fullfile(imageSetsMainPath, 'val.txt'), val);
write_to_file(fullfile(imageSetsSegPath, 'val.txt'), val);

classes = {...
  'aeroplane', 'bicycle', 'bird', 'boat', 'bottle', 'bus', 'car', ...
  'cat', 'chair', 'cow', 'diningtable', 'dog', 'horse', 'motorbike', ...
  'person', 'pottedplant', 'sheep', 'sofa', 'train', 'tvmonitor'};
for i = 1:length(classes)
    trainPath = fullfile(imageSetsMainPath, [classes{i} '_train.txt']);
    valPath = fullfile(imageSetsMainPath, [classes{i} '_val.txt']);
    if(strcmp(classes{i}, 'person'))
        write_to_file(trainPath, train);
        write_to_file(valPath, val);
    else
        write_to_file(trainPath, []);
        write_to_file(valPath, []);
    end
end

%% Helper functions

    function process_files(srcFiles, dstNames, inDir, outDir,...
        inter_method)
        for j = 1:length(srcFiles)
            [~,~,ext] = fileparts(srcFiles{j});
            dstFile = [dstNames{j} ext];
            srcPath = fullfile(inDir, srcFiles{j});
            dstPath = fullfile(outDir, dstFile);
            I = imread(srcPath);
            if(p.Results.max_scale > 0 && ...
                (max(size(I)) > p.Results.max_scale || ...
                (max(size(I)) < p.Results.max_scale && p.Results.upscale)))
                scale = p.Results.max_scale / max(size(I));
                outputSize = round([size(I,1),size(I,2)]*scale);
                I = imresize(I, outputSize, inter_method);
            end
            imwrite(I,cmap,dstPath,'png');
        end
    end

    function write_to_file(fileName, stringArray)
        fid = fopen(fileName,'wt');
        for j = 1:length(stringArray)
             fprintf(fid, '%s\n', stringArray{j});
        end
        fclose(fid);
    end
end

function dstNames = calc_dst_names(srcFiles)
    dstNames = cell(length(srcFiles),1);
    for i = 1:length(srcFiles)
        C = strsplit(srcFiles{i}, {'_', '.'});
        dstNames{i} = [C{1} '_' C{3} '_' C{5}];
    end
end

function cmap = labelColors()
    N=21;
    cmap = zeros(N,3);
    for i=1:N
      id = i-1; r=0;g=0;b=0;
      for j=0:7
        r = bitor(r, bitshift(bitget(id,1),7 - j));
        g = bitor(g, bitshift(bitget(id,2),7 - j));
        b = bitor(b, bitshift(bitget(id,3),7 - j));
        id = bitshift(id,-3);
      end
      cmap(i,1)=r; cmap(i,2)=g; cmap(i,3)=b;
    end
    cmap = cmap / 255;
end

