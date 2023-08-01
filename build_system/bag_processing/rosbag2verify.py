import rosbag
import sys
from event_array_py import Decoder
from collections import defaultdict
from tqdm import tqdm
import numpy as np
import os
import pdb
from colorama import Fore, Style
from scipy.spatial.distance import cdist


def topic_count_verify(bag_name, bag, topic, expected_freq, tolerance=0.02):
    print(Fore.GREEN,  bag_name, topic, "[starting]", Style.RESET_ALL)
    info = bag.get_type_and_topic_info(topic)[1][topic]
    duration = bag.get_end_time() - bag.get_start_time()
    count = info.message_count
    expected_count = duration * expected_freq

    if abs((count - expected_count)/expected_count) > tolerance:
        print(Fore.RED, bag_name, topic, "[failed]", Style.RESET_ALL)
        print(abs((count - expected_count)/expected_count))

        return False, topic, info

    print(Fore.GREEN, bag_name, topic, "[completed]", Style.RESET_ALL)
    return True, topic, info


def ovc_timing_verify(bag_name, bag, tolerance=0.1, verbose=False):
    print(Fore.GREEN,  bag_name, "[OVC TIMING CHECK]", "[starting]", Style.RESET_ALL)
    ovc_topics = ["/ovc/pps_cb", "/ovc/vectornav/imu", "/ovc/left/image_mono/compressed"]
    ros_timing_info = defaultdict(list)
    ros_seq_info = defaultdict(list)
    for topic, msg, t in tqdm(bag.read_messages(topics=ovc_topics),
                              disable=not verbose):
        if topic == "/ovc/pps_cb":
            ros_timing_info[topic].append(msg.stamp.to_nsec())
            ros_seq_info[topic].append(msg.seq)
        else:
            ros_timing_info[topic].append(msg.header.stamp.to_nsec())
            ros_seq_info[topic].append(msg.header.seq)

    ros_timing_info = {k:np.array(v) for k,v in ros_timing_info.items()}
    ros_seq_info = {k:np.array(v) for k,v in ros_seq_info.items()}

    # Fix this!! Idea -> searchsorted
    matching_inds = np.searchsorted( ros_timing_info['/ovc/left/image_mono/compressed'], ros_timing_info['/ovc/pps_cb'] ) - 1

    imager_ts = ros_timing_info['/ovc/left/image_mono/compressed'][matching_inds][2:-2]
    pps_cb_ts = ros_timing_info['/ovc/pps_cb'][2:-2]

    vnav_offset = (pps_cb_ts - imager_ts) / 2500000
    vnav_int_offset = vnav_offset.round() % 16

    inliers = vnav_int_offset == vnav_int_offset[0]

    passed = (inliers.sum() / inliers.shape[0]) > 0.9

    info = {
            "imager_ts": imager_ts.tolist(),
            "pps_cb_ts": pps_cb_ts.tolist(),
            "vnav_offset": vnav_offset.tolist(),
            "vnav_int_offset": vnav_int_offset.tolist(),
           }

    if not passed:
        print(Fore.RED, bag_name, "OVC_TIMING", "[failed]", Style.RESET_ALL)

    return passed, "OVC_TIMING", info


def event_trigger_verify(bag_name, bag, topic, verbose=True):
    print(Fore.GREEN, bag_name, topic, "[starting]", Style.RESET_ALL)
    decoder = Decoder()

    stats = defaultdict(int)
    triggers = []
    event_rate = []

    for topic, msg, t in tqdm(bag.read_messages(topics=topic),
                              disable=not verbose):
        decoder.decode_bytes(msg.encoding, msg.width, msg.height,
                             msg.time_base, msg.events)
        cd_events = decoder.get_cd_events()
        stats["cd_event_count"] += len(cd_events)
        event_rate.append(stats['cd_event_count'] / (cd_events['t'][-1]-cd_events['t'][0]))

        trig_events = decoder.get_ext_trig_events()
        stats["trig_event_count"] += len(trig_events)
        for e in trig_events:
            triggers.append(e)

    triggers = np.array(triggers)

    pos_dt = np.diff(triggers[triggers['p'] == 0]['t']).astype(float) / 1e6
    neg_dt = np.diff(triggers[triggers['p'] == 1]['t']).astype(float) / 1e6

    stats['event_rate_mean'] = np.mean(event_rate)
    stats['event_rate_std'] = np.std(event_rate)
    stats['event_rate_max'] = np.max(event_rate)
    stats['event_rate_min'] = np.min(event_rate)

    stats['pos_trigger_dt_mean'] = np.mean(pos_dt)
    stats['neg_trigger_dt_mean'] = np.mean(neg_dt)
    stats['pos_trigger_dt_std'] = np.std(pos_dt)
    stats['neg_trigger_dt_std'] = np.std(neg_dt)
    stats['pos_trigger_dt_max'] = np.max(pos_dt)
    stats['neg_trigger_dt_max'] = np.max(neg_dt)

    if (np.any(stats['pos_trigger_dt_max'] > 1.5) or
            np.any(stats['neg_trigger_dt_max'] > 1.5)):
        print(Fore.RED, bag_name, topic, "[failed]", Style.RESET_ALL)
        return False, topic, stats

    print(Fore.GREEN, bag_name, topic, "[completed]", Style.RESET_ALL)

    return True, topic, stats


def verifications(bag):
    # Check if bag exists in the filesystem
    if not os.path.exists(bag):
        print("Bag not found: {}".format(bag))
        sys.exit(1)

    # Create a rosbag object
    bag = rosbag.Bag(bag)

    # Print only the bag without the path
    bag_name = bag.filename.split('/')[-1]

    verifications = []
    try:
        # calib_lidar bags are generally shorter, and thus the tolerance
        # calculation can fail (as we only have a few messages)
        # Increasing the tolerance for those bags helps
        if "calib" in bag_name and "lidar" in bag_name:
            print(Fore.YELLOW, "Increasing tolerance due to lidar calib bag",
                  Style.RESET_ALL)
            tolerance = 0.5
        else:
            tolerance = 0.02

        verifications.append(ovc_timing_verify(bag_name, bag))

        # Do not run GPS tests on calibration, indoor, or tunnel bags
        if ("calib" not in bag_name and
                "indoor" not in bag_name and
                "schuylkill_tunnel" not in bag_name):
            verifications.append(topic_count_verify(bag_name, bag,
                                                    "/ovc/gps_trigger",
                                                    1.0, tolerance))
            verifications.append(topic_count_verify(bag_name, bag,
                                                    "/ublox/fix",
                                                    5.0, tolerance))
        else:
            print(Fore.YELLOW, "Gps checks skipped in indoor bags",
                  "or in Schuylkill tunnel bags", Style.RESET_ALL)

        # Check all the other common topics
        verifications.append(topic_count_verify(bag_name, bag,
                                                "/ovc/pps_cb",
                                                1.0, tolerance))
        verifications.append(topic_count_verify(bag_name, bag,
                                                "/ovc/vectornav/imu",
                                                400.0, tolerance))
        verifications.append(topic_count_verify(bag_name, bag,
                                                "/ovc/vectornav/mag",
                                                400.0, tolerance))
        verifications.append(topic_count_verify(bag_name, bag,
                                                "/ovc/left/image_mono/compressed",
                                                25.0, tolerance))
        verifications.append(topic_count_verify(bag_name, bag,
                                                "/ovc/right/image_mono/compressed",
                                                25.0, tolerance))
        verifications.append(topic_count_verify(bag_name, bag,
                                                "/ovc/rgb/image_color/compressed",
                                                25.0, tolerance))

        verifications.append(topic_count_verify(bag_name, bag,
                                                "/os_node/imu_packets",
                                                100.0, tolerance))
        verifications.append(topic_count_verify(bag_name, bag,
                                                "/os_node/sys_time",
                                                100.0, tolerance))
        verifications.append(topic_count_verify(bag_name, bag,
                                                "/os_node/lidar_packets",
                                                1280.0, tolerance))

        # Do not check events for lidar_calib bags
        if not ("calib" in bag_name and "lidar" in bag_name):
            verifications.append(event_trigger_verify(bag_name, bag,
                                                      "/prophesee/right/events",
                                                      args.verbose))
            verifications.append(event_trigger_verify(bag_name, bag,
                                                      "/prophesee/left/events",
                                                      args.verbose))
        else:
            print(Fore.YELLOW, "Event checks skipped in lidar_calib bags",
                  Style.RESET_ALL)

    except Exception as e:
        print(Fore.RED, bag_name, "Exception: ", e, Style.RESET_ALL)
        exit_code = 1
    else:
        exit_code = 0

    for v in verifications:
        if not v[0] or args.verbose:
            print("========================")
            print(v[0])
            print(v[1])
            print(v[2])
            print("========================")
        if not v[0]:
            exit_code = 1
    sys.exit(exit_code)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag',
                        help='List of bags to process, separated by a space')
    # verbose mode
    parser.add_argument('--verbose', action='store_true',
                        help='Set Verbose Mode')
    args = parser.parse_args()
    verifications(args.bag)
