import roslib
import rospy
import rosbag
from sensor_msgs.msg import CompressedImage, Image

import argparse
import os

from pprint import pprint
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()
import pdb
from collections import defaultdict

from tqdm import tqdm

from scipy.stats import linregress
from scipy.spatial.distance import cdist

from event_array_py import Decoder

import pandas as pd

def event_frame(cd_events, width=1280, height=720):
    frame = np.zeros((height, width, 3))
    frame[cd_events['y'].astype(int), cd_events['x'].astype(int)] = 1
    return frame

def compute_skew_offset(fine_timing, verbose=True):
    original_timing = fine_timing.copy()

    # Fix the start of the sequence
    start_offset = original_timing.min()
    offset_timing = original_timing - start_offset

    # Calculate an average skew
    num_samples = offset_timing.shape[0]
    # Dividing by a small integer is beneficial here
    # So we divide by the index and subtract out a full second to know our potentail skews
    skew_candidates = offset_timing.divide(offset_timing['idx'], axis='rows').tail(-1) - 1000000

    skew_proposals = skew_candidates
    skew = {}

    for k in skew_candidates.columns:
        sensor_skew_proposals = skew_proposals[k]
        inliers = (sensor_skew_proposals - sensor_skew_proposals.mean()).abs() <= (sensor_skew_proposals.std()) + 1
        skew[k] = [sensor_skew_proposals[inliers].mean()]

    skew = pd.DataFrame(data=skew)

    additive_skew = skew.loc[np.repeat(skew.index, num_samples)].reset_index(drop=True)
    additive_skew = additive_skew.multiply(offset_timing['idx'], axis='rows').astype(int)

    skewed_timing = offset_timing - additive_skew

    fine_offsets = {}

    for k in skew_candidates.columns:
        fine_offsets[k] = {"sensor": {
                "offset": int(start_offset[k]),
                "skew": float(skew[k]),
                "info": "skew is us per sec - offset is in us",
                }}
    del fine_offsets['idx']

    return fine_offsets

def apply_static_per_topic(fine_timing):
    # Fix the start
    static_offset = fine_timing.copy()
    return static_offset - static_offset.min()

def get_sensor_correlated(timing_info, verbose=True):
    matched_to = [t['correlation']['matched_to'] for t in timing_info.values()]
    if not all([mt == matched_to[0] for mt in matched_to]):
        raise ValueError("All must be matched to the same topic")
    primary_topic = matched_to[0]

    num_samples = timing_info[primary_topic]['sensor'].shape[0]
    num_sensors = len(timing_info)+2 # add one for IDEAL reference and one for IDX

    fine_time_info = defaultdict(list)
    all_msgs = defaultdict(list)

    for i in range(num_samples):
        # We track idx in case of lost
        # We track an ideal pps for a reference
        sample_set = {"idx": i, "ideal": i*1000000}
        msgs = {}

        for topic, info in timing_info.items():
            # Grab mapping that is indexed by sample and returns a primary index (i.e.   p_idx = matching_inds[s_idx])
            topic_idx = info['correlation']['matching_inds'][i]
            # Get the locations of the samples that correlate with the primary
            is_inlier = info['correlation']['inliers'][i]
            if not is_inlier:
                continue
            sample_set[topic] = info['sensor'][topic_idx]
            if verbose and 'msg' in info:
                msgs[topic] = info['msg'][int(topic_idx)]


        # If we don't have all the sensors, skip
        if len(sample_set) == num_sensors:
            for k,v in sample_set.items():
                fine_time_info[k].append(v)
            for k,v in msgs.items():
                all_msgs[k].append(v)

    fine_timing = pd.DataFrame(data=fine_time_info)

    def plot_idx(idx, surrounding=3):
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(2,surrounding*2+1,sharex=True,sharey=True)

        for i in range(-surrounding,surrounding+1):
            axes[0,i].imshow( all_msgs['/ovc/pps_cb'][i+idx] )
            axes[1,i].imshow( all_msgs['/prophesee/left/events'][i+idx] )

    return fine_timing

def match_rough_timings(time_a, time_b):
    timing_distances = cdist( time_a.reshape(-1,1), time_b.reshape(-1,1) )
    matching_inds = np.argmin( timing_distances, axis=0 ).squeeze()
    return time_a[matching_inds], time_b, matching_inds

def get_rough_time_correlations(timing_info, primary_topic, verbose=True):
    # Find offset from primary topic in ROS time
    time_calibs = {}

    # We define the primary topic as the first topic in the list given
    # in this case it will be ovc/pps
    topics = list(timing_info.keys())
    for topic in topics:
        A, B, matching_inds = match_rough_timings(timing_info[topic]['ros'], timing_info[primary_topic]['ros'])
        bag_A, bag_B, matching_inds_bag = match_rough_timings(timing_info[topic]['bag'], timing_info[primary_topic]['bag'])

        if (matching_inds == 0).all():
            print("THIS IS A WARNING THAT WE ARE UTILIZING THE BAG TIMES TO COMPUTE THE CORRELATION")
            matching_inds = matching_inds_bag
            RELAX = True
        else:
            RELAX = False

        assert np.all(matching_inds_bag == matching_inds)

        offset = B - A

        inliers = (abs(offset) >= 0) if RELAX else (abs(offset) < 10000) # 10ms -> 10,000 us

        timing_info[topic]["rough_sync"] = {
                'offsets': offset,
                'mean': float(np.mean( offset[inliers] )),
                'min': float(np.min( offset[inliers] )),
                'max': float(np.max( offset[inliers] )),
                'std': float(np.std( offset[inliers] )),
                'inliers': float(np.sum( inliers ) / offset.shape[0]),
                "relaxed": RELAX,
                }

        timing_info[topic]["internal_clock"] = {
                "mean": float(np.mean( np.diff( A ) )),
                "min": float(np.min( np.diff( A ) )),
                "max": float(np.max( np.diff( A ) )),
                "std": float(np.std( np.diff( A ) )),
                "relaxed": RELAX,
                }

        timing_info[topic]["correlation"] = {
                "matched_to": primary_topic,
                "matching_inds": matching_inds,
                "inliers": inliers,
                "relaxed": RELAX,
                }

    # import matplotlib.pyplot as plt
    # pdb.set_trace()
    # fig, axes = plt.subplots(1,2,sharex=True,sharey=True)
    # axes[0].imshow( timing_info['/ovc/pps_cb']['msg'][ 10 ] )
    # axes[1].imshow( timing_info['/prophesee/left/events']['msg'][timing_info['/prophesee/left/events']['correlation']['matching_inds'][10]] )
    # pdb.set_trace()

    return timing_info


def get_timing_info(bag, topics=[], verbose=True):
    timing_info = {}

    for topic in topics:
        timing_info[topic] = {}
        if 'events' in topic:
            timing_info[topic]['decoder'] = Decoder()
            timing_info[topic]['nevents'] = 0
            timing_info[topic]['ros'] = []
            timing_info[topic]['bag'] = []
            timing_info[topic]['sensor'] = []
            timing_info[topic]['msg'] = []
        else:
            timing_info[topic]['ros'] = []
            timing_info[topic]['bag'] = []
            timing_info[topic]['sensor'] = []
            timing_info[topic]['seq'] = []
            timing_info[topic]['msg'] = []

    # Pull relevant timing information from every sensor
    for topic, msg, t in tqdm(bag.read_messages(topics=topics),
                              disable=not verbose):
        if 'events' in topic:
            d = timing_info[topic]['decoder']
            d.decode_bytes(msg.encoding, msg.width, msg.height, msg.time_base, msg.events)

            cd_events = d.get_cd_events()
            timing_info[topic]['nevents'] += cd_events.shape[0]
            trig_events = d.get_ext_trig_events()

            # No trigger event. Ignore.
            if len(trig_events) == 0:
                continue

            for e in trig_events:
                if e['p'] == 1:
                    continue

                # Convert microseconds to us
                cd_t = cd_events[0]['t']
                trig_t = e['t'] * 1

                if trig_t <= cd_t:
                    timing_info[topic]['bag'].append(t.to_nsec() / 1000)
                    timing_info[topic]['ros'].append(msg.header.stamp.to_nsec() / 1000)
                    timing_info[topic]['sensor'].append(trig_t)
                    if verbose:
                        timing_info[topic]['msg'].append(event_frame(cd_events))
                else:
                    timing_info[topic]['bag'].append(t.to_nsec() / 1000)
                    timing_info[topic]['ros'].append(msg.header.stamp.to_nsec() / 1000 + (trig_t - cd_t) )
                    timing_info[topic]['sensor'].append(trig_t)
                    if verbose:
                        timing_info[topic]['msg'].append(event_frame(cd_events))
        elif 'pps' in topic:
            timing_info[topic]['bag'].append(t.to_nsec() / 1000)
            timing_info[topic]['ros'].append(msg.stamp.to_nsec() / 1000)
            timing_info[topic]['sensor'].append(msg.stamp.to_nsec() / 1000)
            timing_info[topic]['seq'].append(msg.seq)
            if verbose:
                has_img = len(timing_info['/ovc/left/image_mono/compressed']['msg']) > 0
                img = np.zeros((720,1280)) if not has_img else timing_info['/ovc/left/image_mono/compressed']['msg'][-1]
                timing_info[topic]['msg'].append(img)
        elif 'vectornav' in topic:
            timing_info[topic]['bag'].append(t.to_nsec() / 1000)
            timing_info[topic]['ros'].append(msg.header.stamp.to_nsec() / 1000)
            timing_info[topic]['sensor'].append(msg.header.stamp.to_nsec() / 1000)
            timing_info[topic]['seq'].append(msg.header.seq)
        elif 'image' in topic:
            timing_info[topic]['bag'].append(t.to_nsec() / 1000)
            timing_info[topic]['ros'].append(msg.header.stamp.to_nsec() / 1000)
            timing_info[topic]['sensor'].append(msg.header.stamp.to_nsec() / 1000)
            if verbose:
                timing_info[topic]['msg'].append(bridge.compressed_imgmsg_to_cv2(msg))
        elif 'fix' in topic:
            timing_info[topic]['bag'].append(t.to_nsec() / 1000)
            timing_info[topic]['ros'].append(msg.header.stamp.to_nsec() / 1000)
            timing_info[topic]['sensor'].append(msg.header.stamp.to_nsec() / 1000)
        elif 'sys_time' in topic:
            timing_info[topic]['bag'].append(t.to_nsec() / 1000)
            timing_info[topic]['ros'].append(msg.header.stamp.to_nsec() / 1000)
            timing_info[topic]['sensor'].append(msg.sensor_time / 1000)

    # Convert all lists to numpy arrays
    for topic in timing_info.keys():
        if "events" in topic:
            del timing_info[topic]['decoder']
        for k in timing_info[topic].keys():
            if not 'msg' in k:
                timing_info[topic][k] = np.array(timing_info[topic][k]).astype(int)

    # Ouster IMU needs to be resampled to 1Hz on the top of each sensor second
    for topic in timing_info.keys():
        if 'sys_time' in topic: # For ouster times
            # The goal is to generate fake PPS signals to sync other sensors to
            # This is done by computing where the pulse would be at the top of each second
            sensor_to_ros_lr = linregress( timing_info[topic]['sensor'], timing_info[topic]['ros'] )
            sensor_times_to_query = np.arange( int(np.min(timing_info[topic]['sensor']/1000000))*1000000,
                                               int(np.max(timing_info[topic]['sensor']/1000000))*1000000, 1000000 ).astype(int)
            ros_times = (sensor_to_ros_lr.intercept + sensor_to_ros_lr.slope * sensor_times_to_query).astype(int)

            # repeat for bag times
            sensor_to_bag_lr = linregress( timing_info[topic]['sensor'], timing_info[topic]['bag'] )
            sensor_times_to_query = np.arange( int(np.min(timing_info[topic]['sensor']/1000000))*1000000,
                                               int(np.max(timing_info[topic]['sensor']/1000000))*1000000, 1000000 ).astype(int)
            bag_times = (sensor_to_bag_lr.intercept + sensor_to_bag_lr.slope * sensor_times_to_query).astype(int)

            A = timing_info[topic]
            if verbose:
                timing_info[topic] = {'bag': bag_times, 'ros': ros_times, 'sensor': sensor_times_to_query, 'original': A}
            else:
                timing_info[topic] = {'bag': bag_times, 'ros': ros_times, 'sensor': sensor_times_to_query}

    return timing_info

def correct_ovc(timing_info, ovc_pps="/ovc/pps_cb", ovc_imager="/ovc/left/image_mono/compressed", poll_offset=60, interupt_offset=13, verbose=True):
    # The OVC PPS and imager are out of phase by an integer number of vectornav clock cycles
    ovc_pps_info = timing_info[ovc_pps]
    ovc_pps_info['sensor'] -= poll_offset

    ovc_imager_info = timing_info[ovc_imager]
    ovc_imager_info['sensor'] -= interupt_offset

    matching_inds = np.searchsorted( ovc_imager_info['ros'], ovc_pps_info['ros'] ) - 1
    ovc_img_A = ovc_imager_info['ros'][matching_inds][2:-2] # Ignore the first and last two seconds
    ovc_pps_B = ovc_pps_info['ros'][2:-2]

    assert np.all(np.diff(ovc_pps_info['seq'])==1)
    assert np.all(np.diff(ovc_imager_info['seq'])==1)

    del timing_info[ovc_imager]

    ovc_offset = (ovc_img_A - ovc_pps_B) / 2500
    round_offset = ovc_offset.round() % 16
    ovc_info = {
            "dense_offsets": ovc_offset,
            "round_offsets": round_offset,
            "round_offset": int(round_offset[0]),
            }
    inliers = ovc_info["round_offsets"] == ovc_info['round_offset']
    assert (inliers.sum() / inliers.shape[0]) > 0.9

    timing_info[ovc_pps]['decimator_info'] = ovc_info

    return ovc_info

def compute_time_calibs(bag, topics=[], verbose=True, visualize=False):
    if visualize:
        import matplotlib.pyplot as plt

    timing_info = get_timing_info(bag, topics, verbose)

    ovc_dec_info = correct_ovc(timing_info, verbose=verbose)

    rough_time_info = get_rough_time_correlations(timing_info, topics[0], verbose=verbose)
    fine_time_info = get_sensor_correlated(timing_info, verbose=verbose)

    fine_offset = compute_skew_offset(fine_time_info, verbose=verbose)

    if visualize:
        plt.show()

    results = {}

    is_relaxed = max([rough_time_info[k]['rough_sync']['relaxed'] for k in rough_time_info.keys()])

    for k in rough_time_info.keys():
        # NOTE: These bags have bad synchronization for the PTP in ROS time, so we fail back to
        # bag time for the rough synchronization

        if is_relaxed:
            fn = bag.filename
            fn_key = os.path.basename(fn)
            assert fn_key in ['falcon_outdoor_day_penno_parking_1.bag',
                              'falcon_outdoor_day_penno_parking_2.bag',
                              'falcon_outdoor_day_penno_plaza.bag' ]
        # NOTE: This bag has a large skew from PTP
        elif fine_offset[k]['sensor']['skew'] > 1000:
            fn = bag.filename
            fn_key = os.path.basename(fn)
            assert fn_key in ['car_urban_night_penno_small_loop.bag']

        results[k] ={
                'correction': fine_offset[k]['sensor'],
                'rough_correction': rough_time_info[k]['rough_sync'],
                'bag_time': rough_time_info[k]['bag'],
                'ros_time': rough_time_info[k]['ros'],
                'sensor_time': rough_time_info[k]['sensor'],
                'ros_correlation': rough_time_info[k]['correlation'],
                }
    results['/ovc/pps_cb']['decimator'] = ovc_dec_info
    results['/prophesee/left/events']['nevents'] = rough_time_info['/prophesee/left/events']['nevents']
    results['/prophesee/right/events']['nevents'] = rough_time_info['/prophesee/right/events']['nevents']

    return results

def remove_np(container):
    if type(container) == list:
        return [remove_np(c) for c in container]
    elif type(container) == dict:
        return {k:remove_np(c) for k,c in container.items()}
    elif type(container) == np.ndarray:
        return container.tolist()
    else:
        return container

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', help='ROS bag name')
    parser.add_argument('--verbose', action='store_true', help='Set Verbose Mode')
    parser.add_argument('--no_file', action='store_true', help='Set to hold back time offset file')
    parser.add_argument('--time_fn', default=None, help="Name of the time offset file")
    args = parser.parse_args()

    # Open bag
    input_bag = rosbag.Bag(args.bag)

    timing_topics = [
            "/ovc/pps_cb",
            "/ovc/left/image_mono/compressed",
            "/os_node/sys_time",
            "/prophesee/right/events",
            "/prophesee/left/events",
            ]

    time_calibs = compute_time_calibs(input_bag, topics=timing_topics, verbose=args.verbose)

    time_calibs = remove_np(time_calibs)

    from yaml import load, dump
    if not args.no_file:
        if args.time_fn is None:
            time_calib_file = os.path.splitext(args.bag)[0] + "_time_calib.yml"
        else:
            time_calib_file = args.time_fn
        with open(time_calib_file, 'w') as f:
            f.write(dump(time_calibs, default_flow_style=None))
    else:
        print(dump(time_calibs))

    input_bag.close()
