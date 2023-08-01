#!/usr/bin/env python3
import os
import sys
import yaml
import pdb
import numpy as np
import cv2
import h5py
from matplotlib import cm
from colorama import Fore, Style

NAX_NUM_EVENTS = 1000000

if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--h5_depth", help="Input containing the depth", required=True)
    ap.add_argument("--outfn", help="Output video file", required=True)
    ap.add_argument("--h5_events", help="Input containing the events")
    # Add a debug flag argument
    ap.add_argument("--debug", help="Debug flag", action="store_true")
    ap = ap.parse_args()

    # Check that the input depth file exists
    if not os.path.exists(ap.h5_depth):
        sys.exit(f"Input file does not exist: {ap.h5_depth}")

    # Check that the input for events exists
    if ap.h5_events is not None:
        if not os.path.exists(ap.h5_events):
            sys.exit("Input file does not exist: {}".format(ap.h5_events))
        else:
            use_events = True
            print(Fore.YELLOW + "Creating video with events" + Style.RESET_ALL)
    else:
        use_events = False
        print(Fore.YELLOW + "Creating video without events" + Style.RESET_ALL)

    print(Fore.BLUE + "Loading depth file..." + Style.RESET_ALL)
    with h5py.File(ap.h5_depth, "r") as f:
        imgs = f["/depth/prophesee/left"][:]
        map = f["/ts_map_prophesee_left"][:]
    num_frames, height, width = imgs.shape

    # replace all 0 with np.inf
    imgs[imgs == 0] = np.inf

    min_depth = np.min(imgs)
    # Get the max depth as the percentile 99 that is not inf
    max_depth = np.percentile(imgs[imgs != np.inf], 99.7)

    # Initialize the video writer
    fourcc = cv2.VideoWriter_fourcc(*"FFV1")
    fps = 10
    video_writer = cv2.VideoWriter(ap.outfn, fourcc,
                                   fps, (width, height),
                                   isColor=True)

    if use_events:
        events_file = h5py.File(ap.h5_events, "r")
        g = events_file["/prophesee/left"]
        cmap = cm.get_cmap("summer")
    else:
        cmap = cm.get_cmap("jet")

    # Process each frame and write to the video
    for i in range(num_frames-1):
        # Print without newline
        if ap.debug:
            print("\rProcessing frame {}/{}".format(i, num_frames), end="")


        # Read the frame from the dataset
        depth_frame = imgs[i, :, :]
        depth_frame_clipped = np.clip(depth_frame, min_depth, max_depth)
        depth_frame_clipped[depth_frame == np.inf] = np.inf

        # Map the depth values to a colormap
        depth_frame_normalized = (depth_frame_clipped - min_depth) / (max_depth - min_depth)
        depth_frame_colored = (cmap(depth_frame_normalized)[:, :, :3] * 255).astype(np.uint8)

        # Map all the inf values to black
        depth_frame_colored[depth_frame == np.inf] = 0

        if use_events:
            ev_x = g["x"][map[i]:map[i+1]]
            ev_y = g["y"][map[i]:map[i+1]]
            ev_p = g["p"][map[i]:map[i+1]]
            ev_t = g["t"][map[i]:map[i+1]]

            min_t = np.min(ev_t)
            max_t = min_t + 5e3
            idx = np.logical_and(ev_t > min_t, ev_t < max_t)

            # Create indices to sample the event stream
            # idx = np.random.randint(0, len(ev_x),
            #                        np.min((NAX_NUM_EVENTS, len(ev_x))))

            ev_x = ev_x[idx]
            ev_y = ev_y[idx]
            ev_p = ev_p[idx]
            # Color the events
            ev_p = ev_p*255
            color = np.array([ev_p, np.zeros_like(ev_p), 255-ev_p])
            color = color // 2
            depth_frame_colored[ev_y, ev_x, :] = color.T

        # Write the frame to the video
        video_writer.write(depth_frame_colored)
    if ap.debug:
        print("")
    # Release the video writer and close the HDF5 file
    video_writer.release()
    if use_events:
        events_file.close()
