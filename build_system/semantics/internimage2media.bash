#!/bin/bash
source ./build_system/preamble.bash

output_files=("$SEMANTICS_VIDEO_RAW")
check_files output_files

check_free_space fixed

SEMANTICS_VIDEO_RAW_TMP="${SEMANTICS_VIDEO_RAW%.*}_tmp.${SEMANTICS_VIDEO_RAW##*.}"

# Run media generation
echo -e "${BLUE}Starting semantics video generation${NC}"
python3 build_system/semantics/internimage2media.py \
  --semantics_h5 "$INTERNIMAGE_PATH" \
  --events_h5 "$H5_PATH" \
  --outfn "$SEMANTICS_VIDEO_RAW_TMP"
# Check if program exited with error
if [ $? -ne 0 ]; then
  echo -e "${RED}Error creating media files for $H5_PATH${NC}"
  exit 1
fi
mv "$SEMANTICS_VIDEO_RAW_TMP" "$SEMANTICS_VIDEO_RAW"
echo -e "${GREEN}Semantics raw created${NC}"

# Compress video
echo -e "${BLUE}Compressing${NC}"
compress_with_ffmpeg "$SEMANTICS_VIDEO_RAW"
echo -e "${GREEN}Compression finished${NC}"
