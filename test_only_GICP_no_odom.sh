#!/bin/bash

END=$1
PCD_PATH=$2
# END=26
# pcd_path=../test_pcd


cd ./bin

for save_id in $(seq 0 $END)
do
    # exe, mode, points, parameters, keypoints, correspondences, output
    ./submap_registration \
    1 \
    $2/points1_${save_id}.pcd $2/points2_${save_id}.pcd \
    ../config.yaml \
    $2/result1_gicp_no_odom_${save_id}.pcd $2/result2_gicp_no_odom${save_id}.pcd
done

