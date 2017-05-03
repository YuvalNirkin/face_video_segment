# Face Video Segmentation - Face segmentation ground truth from videos
![alt text](https://yuvalnirkin.github.io/face_video_segment/images/snapshot.jpg "Snapshot")  
Snapshot from the Face Video Segmentation Editor.

[Yuval Nirkin](http://www.nirkin.com/), [Iacopo Masi](http://www-bcf.usc.edu/~iacopoma/), [Anh Tuan Tran](https://sites.google.com/site/anhttranusc/), [Tal Hassner](http://www.openu.ac.il/home/hassner/), and [Gerard Medioni](http://iris.usc.edu/people/medioni/index.html).

## Overview
This project contains a collection of tools for semi-supervised gathering of ground truth face segmentation data from videos. A stable hierarchy of regions with temporal coherence is computed from dense optical flow using the method of [2]. Facial landmarks, extended to also include the forehead, are then used to extract the face contour. Regions are classified as belonging to the face segment according to their overlap with the face contour. The regions can then be further processed using a simple interface which allows browsing the entire video and manually classifying the regions using simple mouse clicks.


If you find this code useful, please make sure to cite our paper in your work [1]:

Yuval Nirkin, Iacopo Masi, Anh Tuan Tran, Tal Hassner, Gerard Medioni, "[On Face Segmentation, Face Swapping, and Face Perception](https://arxiv.org/abs/1704.06729)", arXiv preprint arXiv:1704.06729, 22 Apr 2017

Please see [project page](http://www.openu.ac.il/home/hassner/projects/faceswap/) for more details, more resources and updates on this project.

## Dependencies
| Library                                                            | Minimum Version | Notes                                    |
|--------------------------------------------------------------------|-----------------|------------------------------------------|
| [Boost](http://www.boost.org/)                                     | 1.47            |                                          |
| [OpenCV](http://opencv.org/)                                       | 3.0             |                                          |
| [find_face_landmarks](https://github.com/YuvalNirkin/find_face_landmarks) | 1.1      |                                          |
| [video_segment](https://github.com/YuvalNirkin/video_segment)      | 1.0             |                                          |
| [protobuf](https://github.com/google/protobuf)                     | 3.0.0           |                                          |
| [Qt](https://www.qt.io/)                                           | 5.4.0           |                                          |

## Installation
- Use CMake and your favorite compiler to build and install the library.
- Add face_video_segment/bin to path.
- Add face_video_segment/interfaces/matlab to Matlab's path.
- Download the [landmarks model file](http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2).

## Usage
For using the library's C++ interface, please take a look at the [Doxygen generated documentation](https://yuvalnirkin.github.io/face_video_segment/).

Before running the pipeline please prepare an input directory with all the face videos for processing, and an output directory that will contain all the experiment data.
To execute the entire automatic pipeline use the following Matlab code:
```Matlab
inDir = 'videos'; % Input videos directory
outDir = 'fvs_expr'; % Output directory
landmarksFile = 'shape_predictor_68_face_landmarks.dat';  % Landmarks model file
face_video_seg_batch(inDir, outDir, landmarksFile);
```

Now we have 4 directories in "fvs_expr":
- seg_trees - Containing the all the video segmentations hierarchies for each video.
- landmarks - Containing all the landmarks for each video.
- fvs_files - Containing all the classified regions for each video.
- output - Containing all the keyframe images and segmentations for each video in a separate directory.

For additional manual processing go to the "fvs_files" directory and edit the appropiate file with the fvs_editor.
```DOS .bat
fvs_editor video_name.fvs
```
On Windows you can right click the .fvs file, select open with... and point to the fvs_editor.

To regenerate the output images and segmentations you can either use fvs_write_keyframes.m:
```Matlab
fvsFile = 'fvs_expr/fvs_files/video_name.fvs'; % Edited fvs file
outDir = 'fvs_expr/output/video_name'; % Output directory for the specific video
face_video_seg_batch(fvsFile, outDir);
```
Or you can delete the corresponding video directories in "fvs_expr/output" and re-run the automatic pipeline (existing files will not be overwritten).

To convert the generated ground truth to a dataset in PASCAL VOC format, do the following:
Create an empty trainval.csv file:
```Matlab
fvs_init_trainval('fvs_expr/output', 'fvs_expr/output/trainval.csv');
```
Fill the "target" column in the trainval.csv file with "train" for training, "val" for valuation, or leave empty for ignoring the video.
Then to produce the dataset:
```Matlab
produce_pascal_voc_db('fvs_expr/output', 'fvs_expr/pascal_voc_db');
```
Use "add_pascal_voc_db.m" to add additional images and segmentations to the dataset.

## Bibliography
[1] Yuval Nirkin, Iacopo Masi, Anh Tuan Tran, Tal Hassner, Gerard Medioni, [On Face Segmentation, Face Swapping, and Face Perception](https://arxiv.org/pdf/1704.06729.pdf), arXiv preprint arXiv:1704.06729, 22 Apr 2017.  
[2] Grundmann, Matthias and Kwatra, Vivek and Han, Mei and Essa, Irfan, [Efficient hierarchical graph-based video segmentation](https://smartech.gatech.edu/bitstream/handle/1853/38305/cvpr2010_videosegmentation.pdf), In Computer Vision and Pattern Recognition (CVPR), 2010 IEEE Conference on, pp. 2141-2148. IEEE, 2010.

## Related projects
- [End-to-end, automatic face swapping pipeline](https://github.com/YuvalNirkin/face_swap), example application using out face segmentation method.
- [Deep face segmentation](https://github.com/YuvalNirkin/face_segmentation), used to segment face regions in the face swapping pipeline.
- [CNN3DMM](http://www.openu.ac.il/home/hassner/projects/CNN3DMM/), used to estimate 3D face shapes from single images.
- [ResFace101](http://www.openu.ac.il/home/hassner/projects/augmented_faces/), deep face recognition used in the paper to test face swapping capabilities. 

## Copyright
Copyright 2017, Yuval Nirkin, Iacopo Masi, Anh Tuan Tran, Tal Hassner, and Gerard Medioni 

The SOFTWARE provided in this page is provided "as is", without any guarantee made as to its suitability or fitness for any particular use. It may contain bugs, so use of this tool is at your own risk. We take no responsibility for any damage of any sort that may unintentionally be caused through its use.
