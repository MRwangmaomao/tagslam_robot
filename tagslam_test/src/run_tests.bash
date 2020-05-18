#!/bin/bash

num_tests=20

td=`rospack find tagslam_test`
for i in $(seq 19 $num_tests);
do
    echo -n 'running test ' $i ' ... '
    roslaunch tagslam_test test.launch test:=$i > /dev/null
    $td/src/compare_poses.py $td/tests/test_$i/reference.bag ~/.ros/out.bag
    if [ $? -eq 0 ]; then
       echo 'succeeded!'
    else
       echo 'FAILED!'
    fi
done

exit 0
