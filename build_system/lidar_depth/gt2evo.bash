#!/bin/bash
source ./build_system/preamble.bash

output_files=("$GT_POSE_EVO_PATH")
check_files output_files

GT_POSE_EVO_PATH_TMP="${GT_POSE_EVO_PATH%.*}_tmp.${GT_POSE_EVO_PATH##*.}"

# Run media generation
echo -e "${BLUE}Starting conversion of GT pose${NC}"

python3 build_system/lidar_depth/gt2evo.py \
  --h5_gt_pose "$GT_POSE_PATH" \
  --out_evo_pose "$GT_POSE_EVO_PATH_TMP"
# Check if program exited with error
if [ $? -ne 0 ]; then
  echo -e "${RED}Error creating media files for $H5_PATH${NC}"
  exit 1
fi
mv "$GT_POSE_EVO_PATH_TMP" "$GT_POSE_EVO_PATH"
