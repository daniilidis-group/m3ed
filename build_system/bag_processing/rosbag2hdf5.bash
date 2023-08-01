#!/bin/bash
source ./build_system/preamble.bash

# H5_PATH is the only file in the output_files array
if [ "$IS_TEST" -eq 1 ]; then
  output_files=("$H5_PATH" "$STRIPPED_H5_PATH")
else
  output_files=("$H5_PATH")
fi

check_files output_files
check_free_space bag_multiple

H5_PATH_TMP="$H5_PATH.tmp"

# Run bag processing
echo -e "${BLUE}Converting bag to hdf5${NC}"
python3 build_system/bag_processing/rosbag2hdf5.py --bag "$BAG_PATH" \
  --h5fn "$H5_PATH_TMP" \
  --offset "$TIME_CORRECTIONS_PATH" \
  --camchain "$CALIB_CAMCHAIN_PATH" \
  --lidar_calib "$CALIB_LIDAR_PATH" \
  --imu_chain "$CALIB_IMU_PATH"
if [ $? -ne 0 ]; then
  echo -e "${RED}Error converting bag to hdf5${NC}"
  exit 1
fi
echo -e "${GREEN}Done converting bag to hdf5${NC}"

# Move the temporary file to the final location
chmod 666 "$H5_PATH_TMP"
mv "$H5_PATH_TMP" "$H5_PATH"

# Get the HDF5 size in KB
BAG_FILE_SIZE_KB=$(du -k "$BAG_PATH" | cut -f1)
H5_FILE_SIZE=$(du -k "$H5_PATH" | cut -f1)

echo -e "${YELLOW}Bag file size in KB: $BAG_FILE_SIZE_KB${NC}"
echo -e "${YELLOW}H5 file size in KB: $H5_FILE_SIZE${NC}"

# Calculate and print the ratio between file sizes
echo -e "${GREEN}Ratio: ${BOLD}$(echo "scale=2; $H5_FILE_SIZE / $BAG_FILE_SIZE_KB" | bc)${NC}"

# check if IS_TEST is 1, and strip the output bag if so
if [ "$IS_TEST" -eq 1 ]; then
  echo -e "${BLUE}Stripping output bag${NC}"
  STRIPPED_H5_PATH_TMP="$STRIPPED_H5_PATH.tmp"
    python3 build_system/bag_processing/hdf52stripped.py \
    --source "$H5_PATH" --destination $"$STRIPPED_H5_PATH_TMP"
  if [ $? -ne 0 ]; then
    echo -e "${RED}Error converting bag to hdf5${NC}"
    exit 1
  fi
  echo -e "${GREEN}Done stripping bag${NC}"
  chmod 666 "$STRIPPED_H5_PATH_TMP"
  mv "$STRIPPED_H5_PATH_TMP" "$STRIPPED_H5_PATH"
fi
