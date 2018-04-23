#!/bin/bash

dataset_name="$@"

BASE_PATH="/home/kraft/Datasets/PoseEstimation/sixd/"
JOBNAME="constraints_in_3d_pose_estimation"
rm -rf "/data/msteenberg/log_dir/logs"
mkdir -p "/data/msteenberg/log_dir/logs"

for DATASET in $dataset_name
do
  #echo $DATASET
  OBJECT_IDS="$BASE_PATH/$DATASET/test/*/"
  SAVE_DIR="/data/msteenberg/log_dir/data_$DATASET"
  for OBJECT_ID in $OBJECT_IDS
  do
    OBJECT=${OBJECT_ID: -3}
    OBJECT=${OBJECT::-1}
    CREATE_DIR="$SAVE_DIR/$OBJECT"
    mkdir -p $CREATE_DIR
    sbatch -J $JOBNAME ./call_script.sh $BASE_PATH$DATASET $OBJECT $SAVE_DIR
  done
done
