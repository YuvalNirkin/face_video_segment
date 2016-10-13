function vi = images_per_video(imgListFile)
%GET_IMAGES_PER_VIDEO Summary of this function goes here
%   Detailed explanation goes here
D = dlmread(imgListFile,'_');
[imageCount videos] = hist(D(:,1), unique(D(:,1)));
[imageCount, indices] = sort(imageCount, 2, 'descend');
videos = videos(indices);
vi = [videos imageCount'];

