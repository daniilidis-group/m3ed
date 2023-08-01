#!/bin/bash

source ./build_system/preamble.bash

output_files=("$EVENTS_VIDEO_RAW" "$RGB_VIDEO_RAW")
check_files output_files
check_free_space fixed

# create tmp variables for videos
EVENTS_VIDEO_RAW_TMP="${EVENTS_VIDEO_RAW%.*}_tmp.${EVENTS_VIDEO_RAW##*.}"
RGB_VIDEO_RAW_TMP="${RGB_VIDEO_RAW%.*}_tmp.${RGB_VIDEO_RAW##*.}"

# Run bag processing
echo -e "${BLUE}Generating raw videos${NC}"
python3 build_system/bag_processing/hdf52media.py \
  --h5fn "$H5_PATH" \
  --out_events_gray "$EVENTS_VIDEO_RAW_TMP" \
  --out_rgb "$RGB_VIDEO_RAW_TMP"
if [ $? -ne 0 ]; then
  echo -e "${RED}Error creating media files for $H5_PATH${NC}"
  rm "$EVENTS_VIDEO_RAW_TMP" "$RGB_VIDEO_RAW_TMP"
  exit 1
fi
echo -e "${GREEN}Raw videos finished${NC}"
mv "$EVENTS_VIDEO_RAW_TMP" "$EVENTS_VIDEO_RAW"
mv "$RGB_VIDEO_RAW_TMP" "$RGB_VIDEO_RAW"

# Compress videos
echo -e "${BLUE}Generating compressed videos${NC}"
compress_with_ffmpeg "$EVENTS_VIDEO_RAW"
compress_with_ffmpeg "$RGB_VIDEO_RAW"
echo -e "${GREEN}Compressed videos finished${NC}"
