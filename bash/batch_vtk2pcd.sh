#!/bin/bash
# Run: bash batch_vtk2pcd.sh <path_to_clouds>
mkdir -p $1/pcd;
for i in $( ls $1/*.vtk );
do
  filename=$( basename "$i" )
  filename="${filename%.*}"
  pcl_vtk2pcd "$1/$filename.vtk" "$1/pcd/$filename.pcd"
done
