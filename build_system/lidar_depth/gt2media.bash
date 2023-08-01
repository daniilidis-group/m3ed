#!/bin/bash
source ./build_system/preamble.bash

output_files=("$DEPTH_VIDEO_RAW" "$DEPTH_EVENTS_VIDEO_RAW")
check_files output_files

check_free_space fixed

DEPTH_VIDEO_RAW_TMP="${DEPTH_VIDEO_RAW%.*}_tmp.${DEPTH_VIDEO_RAW##*.}"
DEPTH_EVENTS_VIDEO_RAW_TMP="${DEPTH_EVENTS_VIDEO_RAW%.*}_tmp.${DEPTH_EVENTS_VIDEO_RAW##*.}"

# Run media generation
echo -e "${BLUE}Starting depth video generation${NC}"
python3 build_system/lidar_depth/gt2media.py \
  --h5_depth "$GT_DEPTH_PATH" --out "$DEPTH_VIDEO_RAW_TMP"
# Check if program exited with error
if [ $? -ne 0 ]; then
  echo -e "${RED}Error creating media files for $H5_PATH${NC}"
  exit 1
fi
mv "$DEPTH_VIDEO_RAW_TMP" "$DEPTH_VIDEO_RAW"
echo -e "${GREEN}Depth video raw created${NC}"

# Compress video
echo -e "${BLUE}Compressing${NC}"
compress_with_ffmpeg "$DEPTH_VIDEO_RAW"
echo -e "${GREEN}Compression finished${NC}"

echo -e "${BLUE}Starting depth events video generation${NC}"
python3 build_system/lidar_depth/gt2media.py \
  --h5_depth "$GT_DEPTH_PATH" \
  --h5_events "$H5_PATH" \
  --out "$DEPTH_EVENTS_VIDEO_RAW_TMP"
# Check if program exited with error
if [ $? -ne 0 ]; then
  echo -e "${RED}Error creating media files for $H5_PATH${NC}"
  exit 1
fi
mv "$DEPTH_EVENTS_VIDEO_RAW_TMP" "$DEPTH_EVENTS_VIDEO_RAW"
echo -e "${GREEN}Depth video raw created${NC}"

# Compress video
echo -e "${BLUE}Compressing${NC}"
compress_with_ffmpeg "$DEPTH_EVENTS_VIDEO_RAW"
echo -e "${GREEN}Compression finished${NC}"
