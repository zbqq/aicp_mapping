#!/bin/bash
# Run: bash run_registration.sh <path_to_clouds>
# You must be in the folder of the bash
# The clouds and the ground truth poses must be at <path_to_clouds>
# The clouds must be enumerated by order of appearance
cwd=$(pwd)
mkdir -p $1/corrected_poses;
COUNTERA=0
TOTCLOUDS=31
for cloudA in $( ls $1/PointCloud*.pcd | sort -n );
do
  if (($COUNTERA<$TOTCLOUDS));
  then
    COUNTERA=$[$COUNTERA+1]
    filenameA=$( basename "$cloudA" )
    filenameA="${filenameA%.*}"
    COUNTERB=0
    for cloudB in $( ls $1/PointCloud*.pcd | sort -n );
    do
      if (($COUNTERB<$TOTCLOUDS));
      then
        COUNTERB=$[$COUNTERB+1]
        filenameB=$( basename "$cloudB" )
        filenameB="${filenameB%.*}"
	echo "--------------------------------------------------------------------------------"
	echo "[BASH] Running registration between cloud $filenameA.pcd and $filenameB.pcd"
	echo "--------------------------------------------------------------------------------"
	cd $1
	aicp-registration -a $1$filenameA.pcd -b $1$filenameB.pcd
	cd $cwd
      fi
    done
  fi
done
rm -f corrected_poses.txt; cat $(ls -tr *.txt) > corrected_poses.txt
