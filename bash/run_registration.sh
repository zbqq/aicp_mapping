#!/bin/bash
# You must be in the folder of the clouds
# The clouds must be enumerated by order of appearance
mkdir -p $1/estimated_poses;
for cloudA in $( ls $1/PointCloud*.pcd | sort -n );
do
  filenameA=$( basename "$cloudA" )
  filenameA="${filenameA%.*}"
  for cloudB in $( ls $1/PointCloud*.pcd | sort -n );
  do
    filenameB=$( basename "$cloudB" )
    filenameB="${filenameB%.*}"
    echo "Running registration between cloud $filenameA.pcd and $filenameB.pcd"
    #pcl_vtk2pcd "$1/$filename.vtk" "$1/pcd/$filename.pcd"
  done
done
