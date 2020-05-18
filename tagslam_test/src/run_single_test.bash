#!/bin/bash

i=$1
td=`rospack find tagslam_test`
echo -n 'running test ' $i ' ... '
roslaunch tagslam_test test.launch test:=$i > /dev/null
$td/src/compare_poses.py $td/tests/test_$i/reference.bag ~/.ros/out.bag
if [ $? -eq 0 ]; then
    echo 'test ' $i ' succeeded!'
else
    echo 'FAILED!'
    exit -1
fi

exit 0

