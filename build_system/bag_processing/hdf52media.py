#!/usr/bin/env python3
import argparse
import os
import h5py
import numpy as np
import cv2
import sys
import pdb
from colorama import Fore, Style

# Hardcoded topic names from the h5
OVC_LEFT_TOPIC = ["ovc", "left", "data"]
OVC_RIGHT_TOPIC = ["ovc", "right", "data"]
OVC_RGB_TOPIC = ["ovc", "rgb", "data"]
OVC_TS_TOPIC = ["ovc", "ts"]
OVC_MAP_LEFT = ["ovc", "ts_map_prophesee_left_t"]
OVC_MAP_RIGHT = ["ovc", "ts_map_prophesee_right_t"]
PROPHESEE_LEFT_TOPIC = ["prophesee", "left"]
PROPHESEE_RIGHT_TOPIC = ["prophesee", "right"]


def generate_rgb_gray_vid(h5, output_file, verbose=True):
    print("SYNC RGB/GRAY Video Generation")

    # Filename is built based on the list of topics
    # TODO: change this for other video topics
    filename = "rgb_gray_"
    ts = h5[OVC_TS_TOPIC[0]][OVC_TS_TOPIC[1]][:]
    img_rgb = h5[OVC_RGB_TOPIC[0]][OVC_RGB_TOPIC[1]][OVC_RGB_TOPIC[2]]
    img_left = h5[OVC_LEFT_TOPIC[0]][OVC_LEFT_TOPIC[1]][OVC_LEFT_TOPIC[2]]
    img_right = h5[OVC_RIGHT_TOPIC[0]][OVC_RIGHT_TOPIC[1]][OVC_RIGHT_TOPIC[2]]

    # Get image properties from data
    img_w = img_rgb[0].shape[1]
    img_h = img_rgb[0].shape[0]

    # Get output image shape
    out_w = img_w * 3
    out_h = img_h
    fps = np.round(1e6/np.mean(np.diff(ts[:]))).astype(int)

    # Create VideoWriter and iterate over the frames
    fourcc = cv2.VideoWriter_fourcc(*"FFV1")
    video = cv2.VideoWriter(output_file, fourcc,
                            fps, (out_w, out_h), True)

    # Loop over the timestamps
    for n, _ in enumerate(ts):
        if verbose:
            print(f"RGB - Processing frame {n}/{len(ts)}", end="\r")
        write_frame = np.zeros((out_h,out_w,3) ,dtype=img_left[n,...].dtype)
        write_frame[:,0:img_w,:] = img_left[n, ...]
        write_frame[:,img_w:img_w*2,:] = img_rgb[n, ...]
        write_frame[:,img_w*2:img_w*3,:] = img_right[n, ...]

        video.write(write_frame)

    video.release()
    os.chmod(output_file, 0o666)


def generate_rgb_vid(h5, output_file, verbose=True):
    print("RGB Video Generation")

    # Filename is built based on the list of topics
    # TODO: change this for other video topics
    filename = "rgb_"
    ts = h5[OVC_TS_TOPIC[0]][OVC_TS_TOPIC[1]][:]
    img = h5[OVC_RGB_TOPIC[0]][OVC_RGB_TOPIC[1]][OVC_RGB_TOPIC[2]]

    # Get image properties from data
    img_w = img[0].shape[1]
    img_h = img[0].shape[0]

    # Get output image shape
    out_w = img_w
    out_h = img_h
    fps = np.round(1e6/np.mean(np.diff(ts[:]))).astype(int)

    # Create VideoWriter and iterate over the frames
    fourcc = cv2.VideoWriter_fourcc(*"FFV1")
    video = cv2.VideoWriter(output_file, fourcc,
                            fps, (out_w, out_h), True)

    # Loop over the timestamps
    for n, _ in enumerate(ts):
        if verbose:
            print(f"RGB - Processing frame {n}/{len(ts)}", end="\r")
        write_frame = img[n, ...]
        video.write(write_frame)
    video.release()
    os.chmod(output_file, 0o666)


def generate_synchronized_vid(h5, output_file, verbose, events=True, stereo=True):
    print("Synchronized Video Generation")
    # imgs has the pointrs to the hdf5 datasets
    imgs = []
    ev_datasets = []

    # Filename is built based on the list of topics
    # TODO: change this for other video topics
    filename = "imgs_"

    # Check that we have all the topics we need for this
    imgs.append(h5[OVC_LEFT_TOPIC[0]][OVC_LEFT_TOPIC[1]][OVC_LEFT_TOPIC[2]])
    imgs.append(h5[OVC_RIGHT_TOPIC[0]][OVC_RIGHT_TOPIC[1]][OVC_RIGHT_TOPIC[2]])
    ts = h5[OVC_TS_TOPIC[0]][OVC_TS_TOPIC[1]]
    if events:
        map_left = h5[OVC_MAP_LEFT[0]][OVC_MAP_LEFT[1]]
        map_right = h5[OVC_MAP_RIGHT[0]][OVC_MAP_RIGHT[1]]
        ev_datasets.append(h5[PROPHESEE_LEFT_TOPIC[0]][PROPHESEE_LEFT_TOPIC[1]])
        ev_datasets.append(h5[PROPHESEE_RIGHT_TOPIC[0]][PROPHESEE_RIGHT_TOPIC[1]])

    # Get image properties from data
    img_w = imgs[0].shape[2]
    img_h = imgs[0].shape[1]

    # Get output image shape
    dvs_width = 1280
    dvs_height = 720
    border = 10
    out_w = img_w * 2 + border
    if events:
        assert img_w == dvs_width
        out_h = img_h + dvs_height + border
    else:
        out_h = img_h
    fps = np.round(1e6/np.mean(np.diff(ts[:]))).astype(int)

    # Create VideoWriter and iterate over the frames
    fourcc = cv2.VideoWriter_fourcc(*"FFV1")
    video = cv2.VideoWriter(output_file, fourcc,
                            fps, (out_w, out_h), True)

    # Do not iterate over the last frame to avoid index problems
    for n, _ in enumerate(ts[:-1]):
        if verbose:
            print(f"Synchronized - Processing frame {n}/{len(ts)}", end="\r")
        write_frame = None
        for j in range(len(imgs)):
            # OVC Topics
            # Find the index of the closest timestamp
            out_img = imgs[j][n, ...]

            # The images are monocrome
            out_img = np.dstack((out_img, out_img, out_img))

            if j == 0:
                write_frame = out_img
            else:
                write_frame = np.hstack((write_frame,
                                         np.zeros((img_h, border, 3),
                                                  dtype=np.uint8),
                                         out_img))

        # Generate the frame with the event camera
        if events:
            x_l = ev_datasets[0]["x"][map_left[n]:map_left[n+1]]
            y_l = ev_datasets[0]["y"][map_left[n]:map_left[n+1]]
            p_l = ev_datasets[0]["p"][map_left[n]:map_left[n+1]]
            x_r = ev_datasets[1]["x"][map_right[n]:map_right[n+1]]
            y_r = ev_datasets[1]["y"][map_right[n]:map_right[n+1]]
            p_r = ev_datasets[1]["p"][map_right[n]:map_right[n+1]]

            p_l_pos = (p_l[np.newaxis] > 0).astype(np.uint8)*255
            p_l_neg = (p_l[np.newaxis] == 0).astype(np.uint8)*255
            p_r_pos = (p_r[np.newaxis] > 0).astype(np.uint8)*255
            p_r_neg = (p_r[np.newaxis] == 0).astype(np.uint8)*255

            ev_frame_l = np.zeros((dvs_height, dvs_width, 3), dtype=np.uint8)
            ev_frame_l[y_l, x_l, :] = np.vstack((p_l_pos,
                                                 np.zeros_like(p_l_pos),
                                                 p_l_neg)).T
            ev_frame_r = np.zeros((dvs_height, dvs_width, 3), dtype=np.uint8)
            ev_frame_r[y_r, x_r, :] = np.vstack((p_r_pos,
                                                 np.zeros_like(p_r_pos),
                                                 p_r_neg)).T

            ev_f = np.hstack((ev_frame_l,
                              np.zeros((dvs_height, border, 3), dtype=np.uint8),
                              ev_frame_r))

            write_frame = np.vstack((write_frame,
                                     np.zeros((border, dvs_width*2 + border, 3),
                                              dtype=np.uint8),
                                     ev_f))
        video.write(write_frame)
    video.release()
    os.chmod(output_file, 0o666)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Creates vids and imgs from h5.")
    # input h5 file
    parser.add_argument("--h5fn", help="The h5 file to convert.",
                        required=True)
    # output folder where we should store the results
    parser.add_argument("--out_events_gray",
                        help="The output file to store the events/gray video. If not set, no events/gray video will be generated")
    parser.add_argument("--no-events", action="store_false", dest="events",
                        help="Do not add events in out_events_gray.")
    parser.add_argument("--out_rgb", required=True,
                        help="The output file to store the rgb video. If not set, no rgb video will be generated")
    parser.add_argument("--out_all_ovc", action="store_true",
                        help="Output the synced video of all OVC sensors. If not set, no video will be generated.")
    parser.add_argument("--verbose", action="store_true", dest="verbose",
                        help="Be verbose.")
    args = parser.parse_args()
    # Check that the file exists
    if not os.path.isfile(args.h5fn):
        sys.exit("The input h5 file %s does not exist." % args.h5fn)

    print("Converting %s to videos." % args.h5fn)

    # Get output folder name from input file
    verbose = args.verbose

    # Open h5 file and run the generations
    with h5py.File(args.h5fn, 'r') as h5:
        # generate_plots_imu(h5, output_folder, [h5, output_folder, "data"])
        if (not ("calib" in args.h5fn and "lidar" in args.h5fn)):
            if args.out_events_gray is not None:
                # print in blue using colorama
                print(Fore.BLUE + "Generating events/gray video" + Style.RESET_ALL)
                generate_synchronized_vid(h5, args.out_events_gray,
                                          verbose, args.events)
            if args.out_rgb is not None:
                generate_rgb_vid(h5, args.out_rgb, verbose)
                print(Fore.BLUE + "Generating RGB video" + Style.RESET_ALL)
            if args.out_all_ovc:
                generate_rgb_gray_vid(h5, args.out_events_gray, verbose)
                print(Fore.BLUE + "Generating all OVC video" + Style.RESET_ALL)
