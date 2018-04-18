#!/bin/bash

dataset_name=$1

BASE_PATH="/home/kraft/Datasets/PoseEstimation/sixd/"
JOBNAME="constraints_in_3d_pose_estimation"

for DATASET in $dataset_name do
  echo $DATASET

  OBJECT_IDS="$BASE_PATH/*/"

  for OBJECT_ID in $OBJECT_IDS do
    echo object: $OBJECT_ID
    SAVE_DIR="~/log_dir/$DATASET/$OBJECT_ID/"
    mkdir $SAVE_DIR
    echo $JOBNAME $DATASET $SAVE_DIR $OBJECT_ID
    #sbatch -J $JOBNAME ./call_script.sh $DATASET $SAVE_DIR $OBJECT_ID
  done
done
