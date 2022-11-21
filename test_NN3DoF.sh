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
    5 \
    $2/points1_${save_id}.pcd $2/points2_${save_id}.pcd \
    ../config.yaml \
    $2/keypoints1_${save_id}.pcd $2/keypoints2_${save_id}.pcd \
    $2/query_idx_${save_id}.txt $2/match_idx_${save_id}.txt \
    $2/result1_nn_3dof_${save_id}.pcd $2/result2_nn_3dof_${save_id}.pcd
done

# usage: ./test_NN.sh END PCD_PATH