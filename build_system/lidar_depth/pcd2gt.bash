#!/bin/bash
source ./build_system/preamble.bash

# Do not run this script for any calibration bags.
if [[ "$BAG_NAME" == *"calib"* ]]; then
  echo -e "${YELLOW}Skipping calibration bag: $BAG_NAME${NC}"
  exit 0
fi

# output_files includes TRAJ_PATH, PCD_LOCAL_PATH, and GT_PATH
output_files=("$GT_POSE_PATH" "$GT_DEPTH_PATH")
check_files output_files
check_free_space fixed

# Generate GT
echo -e "${BLUE}Starting GT generation${NC}"
python3 build_system/lidar_depth/fasterlio_gt.py \
  --pcd_folder "$PCD_LOCAL_PATH" \
  --traj_fn "$TRAJ_PATH" \
  --timesync_fn  "$TIME_CORRECTIONS_PATH" \
  --calib_h5 "$H5_PATH" \
  --pose_h5_fn "$GT_POSE_PATH.tmp" \
  --depth_h5_fn "$GT_DEPTH_PATH.tmp" \
  --scan_aggregation "$DEPTH_SCAN_AGGREGATION"
if [ $? -ne 0 ]; then
  echo -e "${RED}Error generating GT${NC}"
  exit 1
fi
  echo -e "${GREEN}GT generation finished${NC}"

# Move file to final destination
mv "$GT_POSE_PATH.tmp" "$GT_POSE_PATH"
mv "$GT_DEPTH_PATH.tmp" "$GT_DEPTH_PATH"
