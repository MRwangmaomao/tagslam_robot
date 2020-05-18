# Test cases for TagSLAM

This repository has test cases for
[TagSLAM](https://github.com/berndpfrommer/tagslam), a package for
SLAM based on fiducial markers.

## Installation

This package is meant to be installed from the
[TagSLAM root repository](https://github.com/berndpfrommer/tagslam_root).
Please see instructions there.

## Running the tests

Once you have TagSLAM working, unzip all the reference bags in the
``tests`` directory, and run the tests as follows:

    rosrun tagslam_test run_tests.bash

You can also run an individual test like this:

    rosrun tagslam_test run_single_test.bash 18

