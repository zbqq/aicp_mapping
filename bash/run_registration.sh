#!/bin/bash
# Run: bash run_registration.sh <path_to_clouds>
# You must be in the folder of the clouds
# You must be in the folder of the ground truth poses
# The clouds must be enumerated by order of appearance
cwd=$(pwd)
mkdir -p $1/corrected_poses;
for cloudA in $( ls $1/PointCloud*.pcd | sort -n );
do
  filenameA=$( basename "$cloudA" )
  filenameA="${filenameA%.*}"
  for cloudB in $( ls $1/PointCloud*.pcd | sort -n );
  do
    filenameB=$( basename "$cloudB" )
    filenameB="${filenameB%.*}"
    echo "--------------------------------------------------------------------------------"
    echo "[BASH] Running registration between cloud $filenameA.pcd and $filenameB.pcd"
    echo "--------------------------------------------------------------------------------"
    cd $1
    aicp-registration -a $1$filenameA.pcd -b $1$filenameB.pcd
    cd $cwd
  done
done
rm -f corrected_poses.txt; cat $(ls -tr *.txt) > corrected_poses.txt
