#!/bin/bash

#!/bin/sh
#SBATCH --partition=mmmi-default
#SBATCH --time=10:00:00
#SBATCH --account=msteenberg
#SBATCH --mem-per-cpu=3584M
#SBATCH --output=../log_dir/dirk-%A_%a.out
#SBATCH --error=../log_dir/dirk-%A_%a.err
#SBATCH --cpus-per-task=8
#SBATCH --mail-type=FAIL,REQUEUE,TIME_LIMIT
#SBATCH --mail-user=mstee13@student.sdu.dk
#SBATCH --nice=1000

##SBATCH --job-name=constraints_in_3d_pose_estimation
##SBATCH --nodelist=sdur-cluster-2,sdur-cluster-9,sdur-cluster-3,sdur-cluster-0,sdur-cluster-1,sdur-cluster-4,sdur-cluster-5,sdur-cluster-6,sdur-cluster-8

echo "SLURM_CPUS_PER_TASK = "$SLURM_CPUS_PER_TASK
echo "SLURM_JOB_ACCOUNT   = "$SLURM_JOB_ACCOUNT
echo "SLURM_JOB_ID        = "$SLURM_JOB_ID
echo "SLURM_JOB_NAME      = "$SLURM_JOB_NAME
echo "SLURM_JOB_NUM_NODES = "$SLURM_JOB_NUM_NODES
echo "SLURM_JOB_QOS       = "$SLURM_JOB_QOS
echo "SLURM_MEM_PER_CPU   = "$SLURM_MEM_PER_CPU
echo "SLURM_NODEID        = "$SLURM_NODEID
echo "SLURM_NTASKS        = "$SLURM_NTASKS
echo "SLURM_PRIO_PROCESS  = "$SLURM_PRIO_PROCESS
echo "SLURM_RESTART_COUNT = "$SLURM_RESTART_COUNT
echo "SLURM_SUBMIT_DIR    = "$SLURM_SUBMIT_DIR
echo "SLURM_TASK_PID      = "$SLURM_TASK_PID
echo "SLURMD_NODENAME     = "$SLURMD_NODENAME

echo "********************************************************************************"

export OMP_NUM_THREADS=$SLURM_CPUS_PER_TASK

dataset_dir=$1
object_id=$2
save_dir=$3

echo "********************************************************************************"

echo "OMP_NUM_THREADS     = "$OMP_NUM_THREADS
echo "dataset_name        = "$dataset_name
echo "dataset_dir         = "$dataset_dir
echo "object              = "$object_id


echo "********************************************************************************"
echo "********************************************************************************"

command="srun ~/Constraints-in-3D-Pose-Estimation/build/aout $dataset_dir -z --pose_prior --benchmark-tejani --object-dir=models --yml-file=gt.yml --benchmark-file=test_set_v1.yml --scene-dir=test/$object_id/ply --save-dir=$save_dir --save

echo "COMMAND:"
echo $command
echo "********************************************************************************"
echo "********************************************************************************"
echo "********************************************************************************"

eval $command
