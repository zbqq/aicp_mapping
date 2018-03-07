#!/bin/bash
# Run: bash kitti2pcd_no_ground.sh <path_to_sequences_folder>
# You must be in the folder of the bash
# The binary clouds must be at e.g. <path_to_sequences_folder>/00/velodyne/
# The .pcd clouds will be stored at e.g. <path_to_sequences_folder>/00/pcd/
cd $1
SEQCOUNT=0
for sequence in $(ls -1 | sort -n );
do
  cd $1
  if (($SEQCOUNT>1));
  then
    seq_name=$(printf "%02.0f\n" $(bc<<<$SEQCOUNT))
    # echo "$1${seq_name}"
    FILECOUNT=0
    mkdir -p $1${seq_name}/pcd;
    cd $1${seq_name}/velodyne
    for cloud in $(ls -1 | sort -n );
    do
      cd $1${seq_name}/velodyne
      file_name=$(printf "%000006.0f\n" $(bc<<<$FILECOUNT))
      # echo "$file_name"
      # echo $(pwd)/../pcd/${file_name}.pcd
      kitti2pcd -i $file_name.bin -o $(pwd)/../pcd/${file_name}.pcd
  	  # echo "---------------------------------------------------------------------------------"
  	  # echo "[BASH] Converting KITTI Velodyne point cloud ${file_name}.bin to ${file_name}.pcd"
  	  # echo "---------------------------------------------------------------------------------"
      # pcl-remove-ground -i $(pwd)/../pcd/${file_name}.pcd -o $(pwd)/../pcd/${file_name}.pcd
      # echo "---------------------------------------------------------"
      # echo "[BASH] Removing ground points from cloud ${file_name}.pcd"
      # echo "---------------------------------------------------------"
      FILECOUNT=$[$FILECOUNT+1]
    done
  fi
  SEQCOUNT=$[$SEQCOUNT+1]
done
