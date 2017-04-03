# Face Video Segmentation - Face segmentation ground truth from videos
Created by Yuval Nirkin.

[nirkin.com](http://www.nirkin.com/)

## Overview
This project contains a collection of tools for semi-supervised gathering of ground truth face segmentation data from videos. A stable hierarchy of regions with temporal coherence is computed using dense optical flow. Facial landmarks, extended to also include the forehead, are then used to extract the face contour. Regions are classified for participating in the segmentation according to their overlap with the face contour. The regions can then be further processes using a simple interface which allows browsing the entire video, and manually classify the regions using simple mouse clicks.

![alt text](https://yuvalnirkin.github.io/face_video_segment/images/snapshot.jpg "Snapshot")

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

## Bibliography
