#!/bin/bash
# Run: bash run_registration_validation.sh <path_to_clouds>
# You must be in the folder of the bash
# The clouds must be at <path_to_clouds>
cwd=$(pwd)
mkdir -p $1/validation;
filenameA=cube_cloud_00
for i in `seq 1 100`;
do
  for cloudB in $( ls $1/cube_cloud_*.pcd | sort -n );
  do
    filenameB=$( basename "$cloudB" )
    filenameB="${filenameB%.*}"
    echo "--------------------------------------------------------------------------------"
    echo "[BASH] Running registration between cloud $filenameA.pcd and $filenameB.pcd"
    echo "--------------------------------------------------------------------------------"
    cd $1
    aicp-registration -a $1$filenameB.pcd -b $1$filenameA.pcd
    cd $cwd
  done
done
