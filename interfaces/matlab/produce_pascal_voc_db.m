function produce_pascal_voc_db(varargin)
%PRODUCE_PASCAL_VOC_DB Summary of this function goes here
%   Detailed explanation goes here

%% Parse input arguments
p = inputParser;
addRequired(p, 'inDir', @ischar);
addRequired(p, 'outDir', @ischar);
addParameter(p, 'indices', [], @isvector);
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

%% Initialize training and valuation set
train = [];
val = [];

%% For each directory
cmap = labelColors();
for i = indices   
    dirPath = fullfile(p.Results.inDir, dirNames{i});
    disp(['Processing "' dirNames{i} '"']);
    
    %% Parse current directory
    fileDescs = dir(fullfile(dirPath, '*frame*'));
    frames = {fileDescs.name};
    fileDescs = dir(fullfile(dirPath, '*seg*'));
    segmentations = {fileDescs.name};
    
    %% Copy files
    dstNames = process_files(frames, dirPath, framesPath);
    process_files(segmentations, dirPath, segmentationsPath);
    
    %% Add images to training and valuation sets
    if(length(dstNames) > 2)
        val_ind = floor((length(dstNames) + 1)/2);
        train = [train; dstNames(1:val_ind-1); dstNames(val_ind+1:end)];
        val = [val dstNames(val_ind)];
    else
        train = [train; dstNames];
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

    function dstNames = process_files(fileNames, inDir, outDir)
        dstNames = cell(length(fileNames),1);
        for j = 1:length(fileNames)
            dstNames{j} = [dirNames{i} fileNames{j}(end-8:end-4)];
            dstFile = [dirNames{i} fileNames{j}(end-8:end)];
            srcPath = fullfile(inDir, fileNames{j});
            dstPath = fullfile(outDir, dstFile);
            % copyfile(srcPath, dstPath);
            I = imread(srcPath);
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

