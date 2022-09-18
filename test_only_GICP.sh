#!/bin/bash

END=$1
PCD_PATH=$2
# END=26
# pcd_path=../test_pcd

# DR+GICP

cd ./bin

for save_id in $(seq 0 $END)
do
    # exe, mode, points, parameters, keypoints, correspondences, output
    ./submap_registration \
    1 \
    $2/original1_moved_${save_id}.pcd $2/original2_moved_${save_id}.pcd \
    ../config.yaml \
    $2/result1_gicp_${save_id}.pcd $2/result2_gicp_${save_id}.pcd
done

