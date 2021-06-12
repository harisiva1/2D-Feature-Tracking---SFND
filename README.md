# OVERVIEW

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project.
![OVERVIEW](https://user-images.githubusercontent.com/68550704/121785415-f4b7b300-cbb9-11eb-8038-2046a29ba05b.png)

# SFND 2D Feature Tracking - Midterm Project

<img src="images/keypoints.png" width="820" height="248" />

As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best.Here we will be Programming 1,5,6,7 tasks which is numbered in the above image.This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

# TASK 1

Your first task is to set up the loading procedure for the images, which is currently not optimal. In the student version of the code, we push all images into a vector inside a for-loop and with every new image, the data structure grows. Now imagine you want to process a large image sequence with several thousand images and Lidar point clouds over night - in the current implementation this would push the memory of your computer to its limit and eventually slow down the entire program. So in order to prevent this, we only want to hold a certain number of images in memory so that when a new one arrives, the oldest one is deleted from one end of the vector and the new one is added to the other end.

* Please check [solution](https://github.com/harisiva1/2D-Feature-Tracking---SFND/blob/9d30a61e16d9f8e5fc3382d9c7ce7608c890ffd5/src/MidTermProject_Camera_Student.cpp) from line 106-112

# TASK 2

Please implement a selection of alternative detectors, which are HARRIS,SHI TOMASI, FAST, BRISK, ORB, AKAZE, and SIFT

* please check [solution](https://github.com/harisiva1/2D-Feature-Tracking---SFND/blob/9d30a61e16d9f8e5fc3382d9c7ce7608c890ffd5/src/matching2D_Student.cpp) from line 114 to 289 and [solution2](https://github.com/harisiva1/2D-Feature-Tracking---SFND/blob/9d30a61e16d9f8e5fc3382d9c7ce7608c890ffd5/src/MidTermProject_Camera_Student.cpp) from line 131 to 145.

# TASK 3

In a later part of the mid-term project, you will be evaluating the various detectors and descriptors with regard to a set of performance metrics. As we are focussing on a collision detection system in this course, keypoints on the preceding vehicle are of special interest to us. Therefore, in order to enable a more targeted evaluation, we want to discard feature points that are not located on the preceding vehicle.

Your third task, therefore, is to remove all keypoints outside of a bounding box around the preceding vehicle. Box parameters you should use are : cx = 535, cy = 180, w = 180, h = 150

* Please check solution here [solution](https://github.com/harisiva1/2D-Feature-Tracking---SFND/blob/9d30a61e16d9f8e5fc3382d9c7ce7608c890ffd5/src/MidTermProject_Camera_Student.cpp) from line 155 to 169

# TASK 4

Your fourth task is to implement a variety of keypoint descriptors to the already implemented BRISK method and make them selectable using the string 'descriptorType'. The methods you must integrate are BRIEF, ORB, FREAK, AKAZE and SIFT.

* Please have a look here [solution](https://github.com/harisiva1/2D-Feature-Tracking---SFND/blob/9d30a61e16d9f8e5fc3382d9c7ce7608c890ffd5/src/matching2D_Student.cpp) from line 60 to 111 and [solution2](https://github.com/harisiva1/2D-Feature-Tracking---SFND/blob/9d30a61e16d9f8e5fc3382d9c7ce7608c890ffd5/src/MidTermProject_Camera_Student.cpp) line 202

# TASK 5 and 6

Your fifth task will focus on the matching part. The current implementation uses Brute Force matching combined with Nearest-Neighbor selection. You must now add FLANN as an alternative to brute-force as well as the K-Nearest-Neighbor approach.As your sixth task, you will then implement the descriptor distance ratio test as a filtering method to remove bad keypoint matches.

* Please have a look here [solution](https://github.com/harisiva1/2D-Feature-Tracking---SFND/blob/9d30a61e16d9f8e5fc3382d9c7ce7608c890ffd5/src/matching2D_Student.cpp) from line 8 to 57 and [solution2](https://github.com/harisiva1/2D-Feature-Tracking---SFND/blob/9d30a61e16d9f8e5fc3382d9c7ce7608c890ffd5/src/MidTermProject_Camera_Student.cpp) from line 212 to 237

# TASK 7,8 and 9

* count the number of keypoints on the preceding vehicle for all 10 images and take note of the distribution of their neighborhood size. Do this for all the detectors you have implemented.
* count the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. In the matching step, use the BF approach with the descriptor distance ratio set to 0.8.
* Your ninth task is to log the time it takes for keypoint detection and descriptor extraction. The results must be entered into a spreadsheet and based on this information you will then suggest the TOP3 detector / descriptor combinations as the best choice for our purpose of detecting keypoints on vehicles. 

please check solution for logging here [solution](https://github.com/harisiva1/2D-Feature-Tracking---SFND/blob/9d30a61e16d9f8e5fc3382d9c7ce7608c890ffd5/src/MidTermProject_Camera_Student.cpp) from line 29 to 82.

Logged Results are [here](https://github.com/harisiva1/2D-Feature-Tracking---SFND/blob/0e42b1491632ec9072c89e600561f13359198e25/Results.csv)



## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.
