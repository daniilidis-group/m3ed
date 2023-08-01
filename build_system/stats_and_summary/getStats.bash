#!/bin/bash
source ./build_system/preamble.bash

output_files=("$STATS_PATH")
check_files output_files

echo -e "${YELLOW}Getting stats from bag file: ${BOLD}${BAG_NAME}${NC}"
echo -e "${YELLOW}H5 file: ${BOLD}${H5_PATH}${NC}"

ROS_INFO=$(rosbag info "${BAG_PATH}")
# Get the start date using rosbag info, grep, and awk
start_date=$(echo "${ROS_INFO}" | grep -oP 'start\s*:\s*\K.+')
# Get the duration using rosbag info, grep, and awk
duration=$(echo "${ROS_INFO}"| grep -oP 'duration\s*:\s*\K.+')
# Get the raw bag file size
bag_file_size_kb=$(du -k "${BAG_PATH}" | cut -f1)

# Statistics from the H5 file
h5_file_size_kb=$(du -k "${H5_PATH}" | cut -f1)
# if this is a bag that has calib and lidar in the name, the number of events is zero
if [[ $BAG_PATH == *"calib"* ]] && [[ $BAG_PATH == *"lidar"* ]]; then
  events_left=0
  events_right=0
else
  events_left=$(h5ls -r "${H5_PATH}" | grep "prophesee/left/p" | awk '{print $3}'|sed 's/{\([0-9]*\)\/Inf}/\1/')
  events_right=$(h5ls -r "${H5_PATH}" | grep "prophesee/right/p" | awk '{print $3}'|sed 's/{\([0-9]*\)\/Inf}/\1/')
fi
ovc_left=$(h5ls -r "${H5_PATH}" | grep "ovc/left/data" | awk '{print $3}'|sed 's/{\([0-9]*\)\/.*/\1/')
ovc_right=$(h5ls -r "${H5_PATH}" | grep "ovc/right/data" | awk '{print $3}'|sed 's/{\([0-9]*\)\/.*/\1/')

# Check that all the statistics are not empty
if [ -z "$start_date" ] || [ -z "$duration" ] || [ -z "$bag_file_size_kb" ] || [ -z "$h5_file_size_kb" ] || [ -z "$events_left" ] || [ -z "$events_right" ] || [ -z "$ovc_left" ] || [ -z "$ovc_right" ]; then
  echo -e "${RED}Some stats are empty${NC}"
  exit 1
fi

# Write the start date and duration to a YAML file
cat > "${STATS_PATH}" << EOL
start_date: ${start_date}
duration: ${duration}
raw_bag_file_size_kb: ${bag_file_size_kb}
h5_file_size_kb: ${h5_file_size_kb}
events_left: ${events_left}
events_right: ${events_right}
ovc_left_images: ${ovc_left}
ovc_right_images: ${ovc_right}
EOL

chmod 666 "${STATS_PATH}"

echo -e "${YELLOW}Stats written to: ${BOLD}${STATS_PATH}${NC}"

# Create validation images for the LiDAR calibration for non "calib" bags
# if [[ $BAG_PATH != *"calib"* ]]; then
#   python3 lidar/lidar_calib.py \
#     --h5fn "$H5_PATH" --confirm_only \
#     --percentage 0.25 --confirm_fn "$TMP_PATH/calib_validation1.png" \
#     --npz_fn "$CALIB_LIDAR_PATH"
#   python3 lidar/lidar_calib.py \
#     --h5fn "$H5_PATH" --confirm_only \
#     --percentage 0.5 --confirm_fn "$TMP_PATH/calib_validation2.png" \
#     --npz_fn "$CALIB_LIDAR_PATH"
#   python3 lidar/lidar_calib.py \
#     --h5fn "$H5_PATH" --confirm_only \
#     --percentage 0.75 --confirm_fn "$TMP_PATH/calib_validation3.png" \
#     --npz_fn "$CALIB_LIDAR_PATH"
# fi
