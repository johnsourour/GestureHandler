# Gesture Handler

Integration between Realsense 2 SDK and the Darknet Arapaho YOLO Wrapper, using our own custom data set.

## Dependencies

### Darknet
Darknet is not included because of its massive size, but you can get it [https://github.com/pjreddie/darknet](here.)

### Arapaho
Arapaho is included as a git submodule

## Installation

 - Install [Realsense SDK 2.0](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)
 - Fetch the darknet library
 - Run `make darknet-cpp-shared` in deps/darknet
 - Copy libdarknet-cpp-shared.so to the main directory
 - Add the dataset configuration files (.cfg, .data, .names and .weights) + adjust names.
 - Call `make` from main directory
 - Run `./arapaho`

## Authors

 * John Sourour
 * Karim Abdel Hamid
