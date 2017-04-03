function vi = images_per_video(imgListFile)
%GET_IMAGES_PER_VIDEO(imgListFile) Count the number of images per video
%from a specified image list file.
D = dlmread(imgListFile,'_');
[imageCount videos] = hist(D(:,1), unique(D(:,1)));
[imageCount, indices] = sort(imageCount, 2, 'descend');
videos = videos(indices);
vi = [videos imageCount'];

