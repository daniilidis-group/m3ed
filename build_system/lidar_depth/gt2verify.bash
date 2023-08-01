#!/bin/bash
source ./build_system/preamble.bash

# output_files=("$DEPTH_VIDEO_RAW" "$DEPTH_EVENTS_VIDEO_RAW")
# check_files output_files
# if CHECK_POSE_RETURN is 0, skip this file
if [ $CHECK_POSE_RETURN -eq 0 ]; then
  echo -e "${BLUE}Skipping GT Check for $H5_PATH${NC}"
  exit 0
fi

# Run media generation
echo -e "${BLUE}Starting GT Check${NC}"
python3 build_system/lidar_depth/gt2verify.py \
  --h5_depth "$GT_POSE_PATH" \
  --absolute_error "$ABSOLUTE_POSITION_ERROR" \
  --pose_stats "$TMP_PATH/pose_stats.txt"
# Check if program exited with error
if [ $? -ne 0 ]; then
  echo -e "${RED}Error creating media files for $H5_PATH${NC}"
  exit 1
fi
