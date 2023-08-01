#!/usr/bin/env python3
import os
import sys
import yaml
import pdb
import numpy as np
import cv2
import h5py
from matplotlib import cm
import matplotlib.pyplot as plt
from colorama import Fore, Style

# https://github.com/open-mmlab/mmsegmentation/blob/00790766aff22bd6470dbbd9e89ea40685008395/mmseg/utils/class_names.py#L249C1-L249C1
def cityscapes_palette():
    """Cityscapes palette for external use."""
    return [[128, 64, 128], [244, 35, 232], [70, 70, 70], [102, 102, 156],
            [190, 153, 153], [153, 153, 153], [250, 170, 30], [220, 220, 0],
            [107, 142, 35], [152, 251, 152], [70, 130, 180], [220, 20, 60],
            [255, 0, 0], [0, 0, 142], [0, 0, 70], [0, 60, 100], [0, 80, 100],
            [0, 0, 230], [119, 11, 32]]

def generate_rgb_check_video(ap):
    # get all the semantic labels

    img_w = 1280
    img_h = 720

    out_w = img_w * 3
    out_h = img_h
    fps = 25

    # Initialize the video writer
    fourcc = cv2.VideoWriter_fourcc(*"FFV1")
    fn_parts = os.path.splitext(ap.outfn)
    filename = ''.join([fn_parts[0], '_rgb_check', fn_parts[1]])
    video_writer = cv2.VideoWriter(filename, fourcc,
                                   fps, (out_w, out_h),
                                   isColor=True)
    print(Fore.BLUE + "Loading semantics and writing video" + Style.RESET_ALL)
    palette = np.array(cityscapes_palette())
    # Add a last class to palette, for the background
    palette = np.vstack((palette, np.array([0, 0, 0])))

    with h5py.File(ap.events_h5, "r") as f, h5py.File(ap.semantics_h5, "r") as sems:
        # Read the events dataset
        # events_dataset = f["/depth/prophesee/left"][:]
        predictions = sems["/predictions"]
        ts = sems["/ts"]
        warped_image = sems["/warped_image"]
        rgb_image = f["/ovc/rgb/data"]

        for i in range(len(predictions)-1):
            # remap all the 255 to 19 so they are plotted black
            img = predictions[i]
            img[img == 255] = 19
            color_img = palette[img]
            color_img = color_img.squeeze(2).astype(np.uint8)

            write_frame = np.zeros((out_h, out_w, 3), dtype=np.uint8)

            write_frame[:,0*img_w:1*img_w,:] = warped_image[i,:img_h,:img_w,:]
            write_frame[:,1*img_w:2*img_w,:] = color_img[:img_h,:img_w,:]
            write_frame[:,2*img_w:3*img_w,:] = rgb_image[i,:img_h,:img_w,:]

            video_writer.write(write_frame)

            if ap.debug:
                print(Fore.GREEN + "Writing frame: {}".format(i) +
                      Style.RESET_ALL, end="\r")
    video_writer.release()

def generate_event_overlay_video(ap):
    # get all the semantic labels

    width = 1280
    height = 720
    fps = 25

    # Initialize the video writer
    fourcc = cv2.VideoWriter_fourcc(*"FFV1")
    video_writer = cv2.VideoWriter(ap.outfn, fourcc,
                                   fps, (width, height),
                                   isColor=True)
    print(Fore.BLUE + "Loading semantics and writing video" + Style.RESET_ALL)
    palette = np.array(cityscapes_palette())
    # Add a last class to palette, for the background
    palette = np.vstack((palette, np.array([0, 0, 0])))

    with h5py.File(ap.events_h5, "r") as f, h5py.File(ap.semantics_h5, "r") as sems:
        # Read the events dataset
        # events_dataset = f["/depth/prophesee/left"][:]
        pl = events_dataset = f["/prophesee/left"]
        predictions = sems["/predictions"]
        ts = sems["/ts"]
        ts_map_prophesee_left_t = sems["/ts_map_prophesee_left_t"]
        warped_image = sems["/warped_image"]

        for i in range(len(predictions)-1):
            # remap all the 255 to 19 so they are plotted black
            img = predictions[i]
            img[img == 255] = 19
            color_img = palette[img]
            color_img = color_img.squeeze(2).astype(np.uint8)

            # Get the events for the corresponding frame
            pl = f["/prophesee/left"]
            ev_x = pl["x"][ts_map_prophesee_left_t[i]:ts_map_prophesee_left_t[i + 1]]
            ev_y = pl["y"][ts_map_prophesee_left_t[i]:ts_map_prophesee_left_t[i + 1]]
            ev_p = pl["p"][ts_map_prophesee_left_t[i]:ts_map_prophesee_left_t[i + 1]]
            ev_t = pl["t"][ts_map_prophesee_left_t[i]:ts_map_prophesee_left_t[i + 1]]
            min_t = ev_t.min()
            max_t = min_t + 5e3
            idx = np.logical_and(ev_t >= min_t, ev_t <= max_t)
            ev_x = ev_x[idx]
            ev_y = ev_y[idx]
            ev_p = ev_p[idx]*255
            ev_t = ev_t[idx]

            # Color events in video
            color = np.array([ev_p, np.zeros_like(ev_p), 255-ev_p])
            color = color//2

            # Merge them
            color_img[ev_y, ev_x, :] = color.T
            color_image_bgra = cv2.cvtColor(color_img, cv2.COLOR_RGBA2BGRA)
            video_writer.write(color_image_bgra[:, :, :3])

            if ap.debug:
                print(Fore.GREEN + "Writing frame: {}".format(i) +
                      Style.RESET_ALL, end="\r")
    video_writer.release()

if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--events_h5", help="Input HDF5 file with camera data", required=True)
    ap.add_argument("--semantics_h5", help="Input HDF5 file from InternImage", required=True)
    ap.add_argument("--outfn", help="Output video file", required=True)
    # Add a debug flag argument
    ap.add_argument("--debug", help="Debug flag", action="store_true")
    ap = ap.parse_args()

    # Check that the input file exists
    if not os.path.exists(ap.semantics_h5):
        sys.exit("Input file does not exist: {}".format(ap.semantics_h5))

    if not os.path.exists(ap.events_h5):
        sys.exit("Input file does not exist: {}".format(ap.events_h5))

    generate_event_overlay_video(ap)
    # generate_rgb_check_video(ap)
