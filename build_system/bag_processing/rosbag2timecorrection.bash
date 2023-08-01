#!/bin/bash
source ./build_system/preamble.bash

# TIME_CORRECTIONS_PATH is the only output file in the array
output_files=("$TIME_CORRECTIONS_PATH")
check_files output_files


# Run bag processing
echo -e "${BLUE}Extracting time corrections from $BAG_NAME${NC}"
python3 build_system/bag_processing/rosbag2timecorrection.py --bag "$BAG_PATH" --time_fn "$TIME_CORRECTIONS_PATH.tmp"
if [ $? -ne 0 ]; then
  echo -e "${RED}Error extracting time corrections${NC}"
  exit 1
fi
echo -e "${GREEN}Time corrections extracted${NC}"

# Move output file to correct location
mv "$TIME_CORRECTIONS_PATH.tmp" "$TIME_CORRECTIONS_PATH"

echo -e "${YELLOW}Bag file $BAG_NAME time corrections saved to $TIME_CORRECTIONS_PATH${NC}"
