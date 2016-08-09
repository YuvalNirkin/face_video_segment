function imdb = face_pascal_voc_setup(varargin)
%FACE_PASCAL_VOC_SETUP Summary of this function goes here
%   Detailed explanation goes here
opts.edition = '07' ;
opts.dataDir = fullfile('data','voc07') ;
opts.archiveDir = fullfile('data','archives') ;
opts.includeDetection = false ;
opts.includeSegmentation = false ;
opts.includeTest = false ;
opts = vl_argparse(opts, varargin) ;

%% Source images and classes
imdb.paths.image = esc(fullfile(opts.dataDir, 'JPEGImages', '%s.jpg')) ;
imdb.sets.id = uint8([1 2 3]) ;
imdb.sets.name = {'train', 'val', 'test'} ;
imdb.classes.id = uint8(1:20) ;
imdb.classes.name = {...
  'aeroplane', 'bicycle', 'bird', 'boat', 'bottle', 'bus', 'car', ...
  'cat', 'chair', 'cow', 'diningtable', 'dog', 'horse', 'motorbike', ...
  'person', 'pottedplant', 'sheep', 'sofa', 'train', 'tvmonitor'} ;
imdb.classes.images = cell(1,20) ;
imdb.images.id = [] ;
imdb.images.name = {} ;
imdb.images.set = [] ;
index = containers.Map() ;
[imdb, index] = addImageSet(opts, imdb, index, 'train', 1) ;
[imdb, index] = addImageSet(opts, imdb, index, 'val', 2) ;
if opts.includeTest, [imdb, index] = addImageSet(opts, imdb, index, 'test', 3) ; end

%% Source segmentations
if opts.includeSegmentation
  n = numel(imdb.images.id) ;
  imdb.paths.objectSegmentation = esc(fullfile(opts.dataDir, 'SegmentationObject', '%s.png')) ;
  imdb.paths.classSegmentation = esc(fullfile(opts.dataDir, 'SegmentationClass', '%s.png')) ;
  imdb.images.segmentation = false(1, n) ;
  [imdb, index] = addSegmentationSet(opts, imdb, index, 'train', 1) ;
  [imdb, index] = addSegmentationSet(opts, imdb, index, 'val', 2) ;
  if opts.includeTest, [imdb, index] = addSegmentationSet(opts, imdb, index, 'test', 3) ; end
end

%% Compress data types
imdb.images.id = uint32(imdb.images.id) ;
imdb.images.set = uint8(imdb.images.set) ;
for i=1:20
  imdb.classes.images{i} = uint32(imdb.classes.images{i}) ;
end

%% Check images on disk and get their size
imdb = getImageSizes(imdb) ;

end

function [imdb, index] = addImageSet(opts, imdb, index, setName, setCode)
    j = length(imdb.images.id) ;
    for ci = 1:length(imdb.classes.name)
      className = imdb.classes.name{ci} ;
      annoPath = fullfile(opts.dataDir, 'ImageSets', 'Main', ...
        [className '_' setName '.txt']) ;
      fprintf('%s: reading %s\n', mfilename, annoPath) ;
      [names,labels] = textread(annoPath, '%s %f') ;
      for i=1:length(names)
        if ~index.isKey(names{i})
          j = j + 1 ;
          index(names{i}) = j ;
          imdb.images.id(j) = j ;
          imdb.images.set(j) = setCode ;
          imdb.images.name{j} = names{i} ;
          imdb.images.classification(j) = true ;
        else
          j = index(names{i}) ;
        end
        if labels(i) > 0, imdb.classes.images{ci}(end+1) = j ; end
      end
    end
end

function [imdb, index] = addSegmentationSet(opts, imdb, index, setName, setCode)
    segAnnoPath = fullfile(opts.dataDir, 'ImageSets', 'Segmentation', [setName '.txt']) ;
    fprintf('%s: reading %s\n', mfilename, segAnnoPath) ;
    segNames = textread(segAnnoPath, '%s') ;
    j = numel(imdb.images.id) ;
    for i=1:length(segNames)
      if index.isKey(segNames{i})
        k = index(segNames{i}) ;
        imdb.images.segmentation(k) = true ;
        imdb.images.set(k) = setCode ;
      else
        j = j + 1 ;
        index(segNames{i}) = j ;
        imdb.images.id(j) = j ;
        imdb.images.set(j) = setCode ;
        imdb.images.name{j} = segNames{i} ;
        imdb.images.classification(j) = false ;
        imdb.images.segmentation(j) = true ;
      end
    end
end

function imdb = getImageSizes(imdb)
    for j=1:numel(imdb.images.id)
      info = imfinfo(sprintf(imdb.paths.image, imdb.images.name{j})) ;
      imdb.images.size(:,j) = uint16([info.Width ; info.Height]) ;
      fprintf('%s: checked image %s [%d x %d]\n', mfilename, imdb.images.name{j}, info.Height, info.Width) ;
    end
end

function str=esc(str)
    str = strrep(str, '\', '/') ;
end
