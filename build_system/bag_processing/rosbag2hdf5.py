import roslib
import rospy
import rosbag
from sensor_msgs.msg import CompressedImage, Image
import subprocess
from datetime import datetime

import argparse
import os
from colorama import Fore, Style

import yaml
import h5py
import numpy as np
import math
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
import pdb
from collections import defaultdict
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import sys

from tqdm import tqdm

from threading import Thread, Condition

from ouster.client import (
    LidarPacket,
    ImuPacket,
    SensorInfo,
    Scans,
    Packets,
    ChanField,
    XYZLut,
    _utils,
)
from event_array_py import Decoder


class HDFHandler:
    UNITS_TO_SECONDS = 0 # FORCE DIVIDE BY ZERO IF NOT DEFINED

    TOPIC_TO_GROUP = {
            "/ublox/hp_fix" : "/ublox",
            "/ublox/navclock" : "/ublox",
            "/ovc/gps_trigger" : "/ublox",
            "/ovc/right/image_mono/compressed" : "/ovc/right",
            "/ovc/left/image_mono/compressed" : "/ovc/left",
            "/ovc/rgb/image_color/compressed" : "/ovc/rgb",
            "/ovc/vectornav/imu" : "/ovc/imu",
            "/prophesee/left/events" : "/prophesee/left",
            "/prophesee/right/events" : "/prophesee/right",
            "/os_node/lidar_packets" : "/ouster",
            "/os_node/imu_packets" : "/ouster/imu",
            }

    SINGLE_TIME_DOMAIN = True

    def __init__(self, h5file, topics, bag, time_calib):
        if type(topics) == str:
            topics = [topics]
        self.group_names = [self.TOPIC_TO_GROUP[t] for t in topics]
        self.groups = {t: h5file.require_group(g) for t,g in zip(topics, self.group_names)}
        self.group = h5file[self.get_root(self.group_names)]

        self.topics = topics

        self.topic_infos = []
        self.topic_counts = []

        for t in topics:
            self.topic_infos.append(bag.get_type_and_topic_info(t)[1][t])
            self.topic_counts.append(self.topic_infos[-1].message_count)

        if len(topics) == 1:
            self.topic_count = self.topic_counts[0]

        topic_list = list(time_calib.keys())
        topic_matches = [self.get_closest_topic(t, topic_list) for t in topics]

        if not all([topic_matches[0] == tm for tm in topic_matches]):
            if self.SINGLE_TIME_DOMAIN:
                raise NotImplementedError("Individual handlers must all be in the same time domain - if you are sure turn it off")

        self.global_time_calib = time_calib
        self.time_calib = time_calib[topic_matches[0]]

        self.skew_us_per_s = self.time_calib['correction']['skew']
        self.us_offset = self.time_calib['correction']['offset']

        self.ovc_pps_offset = int(time_calib['/ovc/pps_cb']['decimator']['round_offsets'][0] * 2500)

        self.bag_name = os.path.basename(bag.filename)

    def sensor_to_global_us(self, sensor_time_us):
        # Returns the ideal world time from the given sensor time

        # First we compute the offset time given the sensor offset and ovc offset
        offset_us = sensor_time_us - self.us_offset - self.ovc_pps_offset

        # We compute the ideal time, but this has some bad floating point properties
        # Accurate enough to find number of elapsed seconds
        new_time_us = offset_us / (1 + (self.skew_us_per_s/1e6))

        # We use the ideal number of seconds to directly compute the skew
        # that has occured in us
        skew_offset_us = (new_time_us/1e6) * self.skew_us_per_s

        # We return the final time
        return offset_us - skew_offset_us

    def get_root(self, topic_list):
        token_list = [t.split("/") for t in topic_list]
        root = token_list[0]

        for tk in token_list:
            tmp = []
            for r,t in zip(root,tk):
                if r == t:
                    tmp.append(r)
            root = tmp

        return "/" + "/".join(root)

    def get_closest_topic(self, topic, topic_list):
        # We do not have an exact match across the topics
        t_tokens = topic.split("/")
        max_sim = 0
        for tl in topic_list:
            tl_tokens = tl.split("/")
            c = 0
            for a,b in zip(t_tokens, tl_tokens):
                if a==b:
                    c += 1
            if c > max_sim:
                max_sim = c
                t = tl

        return t

    def process(self, msg, topic=None, bag_time=None):
        return

    def finish(self):
        return

    def primary_time_ds(self):
        raise NotImplementedError("Sub classes need to return this")


class GPS(HDFHandler):
    SINGLE_TIME_DOMAIN = False
    def __init__(self, h5file, bag, time_calib):
        topics = [
                 "/ublox/hp_fix",
                 "/ublox/navclock",
                 "/ovc/gps_trigger",
                 ]

        super().__init__(h5file, topics, bag, time_calib)

        num_msgs = self.topic_counts[0]
        
        self.ts_ds = self.group.create_dataset("ts", (num_msgs,), dtype='i8', compression='lzf', maxshape=(None,))
        self.navclock_ds = self.group.create_dataset("navclock", (num_msgs,), dtype='i8', compression='lzf', maxshape=(None,))
        self.utm_ds = self.group.create_dataset("utm", (num_msgs,3), dtype='f8', compression='lzf', maxshape=(None,3))
        self.fix_ds = self.group.create_dataset("fix", (num_msgs,3), dtype='f8', compression='lzf', maxshape=(None,3))

        self.message_buffers = defaultdict(list)

    def ovc_sensor_to_global_us(self, sensor_time_us):
        # Refer to sensor_to_global_us for comments
        time_calib = self.global_time_calib['/ovc/pps_cb']
        skew_us_per_s = time_calib['correction']['skew']
        us_offset = time_calib['correction']['offset']

        offset_us = sensor_time_us - us_offset - self.ovc_pps_offset
        new_time_us = offset_us / (1 + (skew_us_per_s/1e6))
        skew_offset_us = (new_time_us/1e6) * skew_us_per_s
        return offset_us - skew_offset_us

    def associate_messages(self):
        return

    def process(self, msg, topic=None, bag_time=None):
        self.message_buffers[topic].append( (int(bag_time.to_nsec()/1e3), msg) )

    def finish(self):
        hpfix_bag_time = np.array([v[0] for v in self.message_buffers['/ublox/hp_fix']])
        hpfix_ros_time = np.array([int(v[1].header.stamp.to_nsec()/1e3) for v in self.message_buffers['/ublox/hp_fix']])

        navclock_bag_time = np.array([v[0] for v in self.message_buffers['/ublox/navclock']])
        navclock_time = np.array([v[1].iTOW for v in self.message_buffers['/ublox/navclock']]) # This is global time
        navclock_second_mark = (navclock_time == ((navclock_time // 1000) * 1000))

        ovc_gps_trigger_bag_time = np.array([v[0] for v in self.message_buffers['/ovc/gps_trigger']])
        ovc_gps_trigger_time = np.array([int(v[1].stamp.to_nsec()/1e3) for v in self.message_buffers['/ovc/gps_trigger']])
        ovc_gps_trigger_time = self.ovc_sensor_to_global_us( ovc_gps_trigger_time )

        ovc_retime_mapping = np.array(self.ovc.retime_history['/ovc/left/image_mono/compressed'])
        pdb.set_trace()
        return

    def primary_time_ds(self):
        return self.ts_ds

    def attach_ovc(self, ovc):
        self.ovc = ovc

class Ouster(HDFHandler):
    UNITS_TO_SECONDS = 1e9

    def __init__(self, h5file, topic, bag, time_calib):
        super().__init__(h5file, topic, bag, time_calib)

        self.lidar_packets = []
        self.id_init = None
        ouster_info = self.get_ouster_info(bag)
        self.ouster_info = SensorInfo(ouster_info)

        scan_count_est = math.floor(self.topic_counts[0] / 128)

        self.lidar_buf_ds = self.group.create_dataset('data',
                                                      (scan_count_est,128,12609),
                                                      compression='lzf', dtype='u1',
                                                      chunks=(1,128,12609),
                                                      maxshape=(None,128,12609))
        self.group.create_dataset("metadata", data=ouster_info)
        self.lidar_start_ts_ds = self.group.create_dataset("ts_start", (scan_count_est,), dtype='i8', compression='lzf', maxshape=(None,))
        self.lidar_end_ts_ds = self.group.create_dataset("ts_end", (scan_count_est,), dtype='i8', compression='lzf', maxshape=(None,))

        self.imu_group = self.groups[ topic[-1] ]

        self.imu_ds = {}

        imu_count = scan_count_est * 20

        self.imu_ds['av'] = self.imu_group.create_dataset(
            "omega", [imu_count, 3], dtype="f8", maxshape=(None,3)
        )
        self.imu_ds['la'] = self.imu_group.create_dataset(
            "accel", [imu_count, 3], dtype="f8", maxshape=(None,3)
        )
        self.imu_ds['ts'] = self.imu_group.create_dataset(
            "ts", [imu_count], dtype="i8", compression="lzf", maxshape=(None,)
        )

        self.imu_idx = 0

    def primary_time_ds(self):
        return self.lidar_start_ts_ds

    def get_ouster_info(self, bag, topic="/os_node/metadata"):
        for topic, msg, t in bag.read_messages(topics=[topic]):
            ouster_metadata = msg.data
        return ouster_metadata

    def process(self, msg, topic=None, bag_time=None):
        if 'lidar' in topic:
            ouster_lp = LidarPacket(msg.buf, self.ouster_info)
            self.lidar_packets.append(
                (ouster_lp.measurement_id, ouster_lp.frame_id, msg.buf, ouster_lp.timestamp)
            )

            fp = self.lidar_packets[0]  # first
            cp = self.lidar_packets[-1]  # current

            have_full_scan = (0 in fp[0]) and (2047 in cp[0])
            have_half_scan = (not 0 in fp[0]) and (2047 in cp[0])

            if have_half_scan:
                self.lidar_packets = []

            if have_full_scan:
                lidar_sweep_start_ts = self.sensor_to_global_us( self.lidar_packets[0][3][0] / 1e3 )
                lidar_sweep_end_ts = self.sensor_to_global_us( self.lidar_packets[-1][3][-1] / 1e3 )

                # If the lidar sweep started before the start of all sensors, throw away
                if lidar_sweep_start_ts < 0:
                    self.lidar_packets = []
                    return

                # Only initialize the ID once we have started all sensors
                if self.id_init is None:
                    self.id_init = cp[1]
                self.cur_frame_id = cp[1] - self.id_init  ## INDEX

                frame_buf = np.empty((128, 12609), dtype=np.uint8)

                for idx, lp in enumerate(self.lidar_packets):
                    frame_buf[idx, :] = np.frombuffer(lp[2], dtype=np.uint8, count=12609)

                self.lidar_buf_ds[self.cur_frame_id, ...] = frame_buf

                # The start of the sweep is at the first packet's first timestamp
                self.lidar_start_ts_ds[self.cur_frame_id] = lidar_sweep_start_ts
                # The end of the sweep is at the last packet's last timestamp
                self.lidar_end_ts_ds[self.cur_frame_id] = lidar_sweep_end_ts

                self.lidar_packets = []  ## RESET
        elif 'imu' in topic:
            ouster_ip = ImuPacket(msg.buf, self.ouster_info)

            self.imu_ds['av'][self.imu_idx] = np.deg2rad(ouster_ip.angular_vel)
            self.imu_ds['la'][self.imu_idx] = 9.81 * ouster_ip.accel
            self.imu_ds['ts'][self.imu_idx] = self.sensor_to_global_us( ouster_ip.sys_ts / 1e3 )

            self.imu_idx += 1

    def finish(self):
        if self.id_init is None:
            return

        self.lidar_buf_ds.resize( (self.cur_frame_id+1,128,12609) )
        self.lidar_start_ts_ds.resize( (self.cur_frame_id+1,) )
        self.lidar_end_ts_ds.resize( (self.cur_frame_id+1,) )

        self.imu_ds['av'].resize( (self.imu_idx+1,3) )
        self.imu_ds['la'].resize( (self.imu_idx+1,3) )
        self.imu_ds['ts'].resize( (self.imu_idx+1,) )


class OVC(HDFHandler):

    def __init__(self, h5file, bag, time_calib):
        ovc_topics = ["/ovc/right/image_mono/compressed",
                      "/ovc/left/image_mono/compressed",
                      "/ovc/rgb/image_color/compressed",
                      "/ovc/vectornav/imu"]
        self.image_topics = ovc_topics[:-1]
        self.imu_topic = ovc_topics[-1]

        super().__init__(h5file, ovc_topics, bag, time_calib)

        self.ds = {}

        self.image_sizes = {}

        for topic, topic_count in zip(ovc_topics[:-1], self.topic_counts):
            image_size = self.get_image_size(topic, bag)
            self.image_sizes[topic] = image_size

            self.ds[topic] = {
                    "shape": tuple([topic_count] + image_size),
                    "chunk_shape": tuple([1] + image_size),
                    "max_shape": tuple([None] + image_size),
                    }

            self.ds[topic]['ds'] = self.groups[topic].create_dataset(
                "data",
                self.ds[topic]['shape'],
                compression="lzf",
                dtype="u1",
                chunks=self.ds[topic]['chunk_shape'],
                maxshape=self.ds[topic]['max_shape'],
            )
        self.ds['ts'] = self.group.create_dataset(
            "ts", [topic_count], dtype="i8", compression="lzf", maxshape=(None,)
        )

        self.imu_group = self.groups[ ovc_topics[-1] ]

        self.imu_ds = {}

        self.imu_ds['av'] = self.imu_group.create_dataset(
            "omega", [topic_count*16, 3], dtype="f8", maxshape=(None,3)
        )
        self.imu_ds['la'] = self.imu_group.create_dataset(
            "accel", [topic_count*16, 3], dtype="f8", maxshape=(None,3)
        )
        self.imu_ds['ts'] = self.imu_group.create_dataset(
            "ts", [topic_count*16], dtype="i8", compression="lzf", maxshape=(None,)
        )

        self.drift = 0.0
        self.retime_history = defaultdict(list)

        self.buffers = defaultdict(list)

        self.imager_idx = 0
        self.imu_idx = 0

        self.init_id = None
        self.rel_id = None

        self.has_started = False

        self.imu_problems = []

    def get_image_type(self, topic):
        return topic.split('/')[2]

    def get_image_size(self, topic, bag):
        for topic, msg, t in bag.read_messages(topics=[topic]):
            return list(self.get_image_from_msg(msg).shape)

    def sync_buffers(self, get_popped=False):
        times = [buf[0][0] for buf in self.buffers.values() if len(buf)>0]
        max_time = max(times)

        ret_val = True
        popped = defaultdict(list)

        for topic, buf in self.buffers.items():
            while len(buf) > 1 and buf[0][0] < max_time:
                popped[topic].append(buf.pop(0))
                if len(buf) == 1:
                    ret_val = (buf[0][0] == max_time)

        if get_popped:
            return ret_val, popped
        else:
            return ret_val

    def wait_for_start(self):
        ret_val = True
        for topic, buf in self.buffers.items():
            while len(buf) > 0 and buf[0][0] < -250:
                buf.pop(0)
                ret_val = False
            if len(buf) == 0:
                ret_val = False
        return ret_val

    def retime(self, seq_id, topic, original_time=None, use_original_time=False):
        rel_id = (seq_id - self.init_id[topic] + self.rel_id[topic]) 

        if use_original_time:
            return original_time, rel_id

        if "imu" in topic:
            new_time = rel_id * 2500
            self.retime_history[topic].append((original_time, new_time))
            return new_time, rel_id
        elif "image" in topic:
            new_time = rel_id * 40000
            self.retime_history[topic].append((original_time, new_time))
            self.drift = 0.5 * (new_time - original_time) + 0.5 * self.drift
            if not ( 40000 * round(original_time / 40000) == new_time ):
                assert self.bag_name in ["car_urban_night_penno_small_loop.bag",
                                         "falcon_outdoor_day_penno_parking_3.bag"]
                percent_error = ((new_time - original_time) - self.drift) / self.drift
                if percent_error > 0.01:
                    raise NotImplementedError("High change in drift is invalid")
            return new_time, rel_id
        else:
            raise NotImplementedError("Should not be possible to arrive here")

    def get_buf_times(self, idx, asdict=True):
        if asdict:
            return {k:buf[idx][0] for k,buf in self.buffers.items()}
        else:
            return [buf[idx][0] for buf in self.buffers.values()]


    def get_and_sync(self):
        # The data should be synced!!
        times = [buf[0][0] for buf in self.buffers.values()]

        readings = {k:self.buffers[k].pop(0) for k in self.image_topics}
        next_img_time = self.buffers[self.image_topics[0]][0][0]
        readings[self.imu_topic] = []
        while self.buffers[self.imu_topic][0][0] < next_img_time:
            readings[self.imu_topic].append( self.buffers[self.imu_topic].pop(0) )

        return readings

    def process_buffer(self):
        if len(self.buffers[self.imu_topic]) < 32:
            return

        if any([len(b)<=2 for b in self.buffers.values()]):
            return

        cur_data = self.get_and_sync()

        for img_topic in self.image_topics:
            buf_info = cur_data[img_topic]
            retime, rel_id = self.retime( buf_info[1], img_topic, buf_info[0] )
            self.ds[img_topic]['ds'][self.imager_idx] = buf_info[2]
        self.ds['ts'][self.imager_idx] = retime

        assert self.imager_idx == (rel_id - self.rel_id[img_topic])

        self.imager_idx += 1

        # Pull into local read buffers
        imu_local_read_bufs = {k: np.stack([buf[2][k] for buf in cur_data[self.imu_topic]], axis=0) for k in cur_data[self.imu_topic][0][2].keys()}

        # Create local write buffers
        imu_local_write_bufs = {k: np.nan*np.zeros((16,v.shape[0])) for k,v in cur_data[self.imu_topic][0][2].items()}
        imu_local_write_bufs['ts'] = np.nan*np.zeros((16,))

        # Get all initial timing information
        imu_ts = np.array([cd[0] for cd in cur_data[self.imu_topic]])
        imu_seq_id = np.array([cd[1] for cd in cur_data[self.imu_topic]])
        imu_retimed = np.array( [self.retime(seq_id, self.imu_topic, ts) for seq_id, ts in zip(imu_seq_id, imu_ts)] )
        imu_local_idx = ((imu_ts - imu_ts[0]) / 2500).round().astype(int)

        seq_vals, seq_ind, seq_count = np.unique(imu_local_idx, return_index=True, return_counts=True)
        max_repeats = np.max(seq_count)

        # This is the aggregate change in the relative ID
        self.rel_id[self.imu_topic] += 16 - imu_ts.shape[0]

        if imu_ts.shape[0] == 16: # 16 values map one way
            for k in imu_local_read_bufs.keys():
                imu_local_write_bufs[k] = imu_local_read_bufs[k]
            imu_local_write_bufs['ts'] = imu_retimed[:,0]
        elif max_repeats == 1: # all unique - mapping can be trusted
            idx_diff = np.diff(imu_local_idx)
            for idx in np.where(idx_diff == np.max(idx_diff))[0]:
                imu_retimed[(idx+1):,:] += (idx_diff[idx]-1) * np.array([2500,1])

            for k in imu_local_read_bufs.keys():
                imu_local_write_bufs[k][imu_local_idx] = imu_local_read_bufs[k]
            imu_local_write_bufs['ts'][imu_local_idx] = imu_retimed[:,0]
        elif max_repeats == 2: # Repeats of 2 are from immediately before or immediately after
            idx_diff = np.diff(imu_local_idx)

            for ind in np.where(seq_count==2)[0]:
                ind_a = ind
                ind_b = ind + 1 # np.unique returns the first - so the second should be immediately after

                if imu_local_idx[ind_a] != imu_local_idx[ind_b] and imu_local_idx[ind_b] == imu_local_idx[ind_b+1]:
                    imu_local_idx[ind_b] -= 1
                elif ind_b+1 >= imu_local_idx.shape[0]: # at the end
                    if imu_local_idx[ind_b] < 15:
                        imu_local_idx[ind_b] += 1
                elif imu_local_idx[ind_a-1]+1 == imu_local_idx[ind_a] and imu_local_idx[ind_b] != imu_local_idx[ind_b+1] - 1:
                    imu_local_idx[ind_b] += 1
                elif imu_local_idx[ind_a-1]+1 != imu_local_idx[ind_a] and imu_local_idx[ind_b] == imu_local_idx[ind_b+1] - 1:
                    imu_local_idx[ind_a] -= 1
                elif imu_local_idx[ind_a-1]+1 != imu_local_idx[ind_a] and imu_local_idx[ind_b] != imu_local_idx[ind_b+1] - 1:
                    imu_local_idx[ind_a] -= 1
                elif np.any(idx_diff > 1):
                    idx_errors = np.where(idx_diff > 1)[0]
                    idx_problem = np.argmin( np.abs( idx_errors - ind_a ) )
                    closest_idx = idx_errors[idx_problem]

                    if closest_idx < ind_a:
                        imu_local_idx[(closest_idx+1):ind_b] -= 1
                    elif closest_idx > ind_b:
                        imu_local_idx[ind_a:closest_idx] += 1
                else:
                    print(imu_local_idx)
                    raise NotImplementedError("Imu Local Index Pattern Not Supported")

            idx_diff = np.diff(imu_local_idx)

            if np.any(idx_diff < 0):
                print("==============")
                print(imu_local_idx)
                print(imu_ts)
                print(imu_retimed.T)
                print("==============")
                raise NotImplementedError("IDX must be increasing")

            for idx in np.where(idx_diff == np.max(idx_diff))[0]:
                imu_retimed[(idx+1):,:] += (idx_diff[idx]-1) * np.array([2500,1])

            for k in imu_local_read_bufs.keys():
                imu_local_write_bufs[k][imu_local_idx] = imu_local_read_bufs[k]
            imu_local_write_bufs['ts'][imu_local_idx] = imu_retimed[:,0]
        else:
            print("==============")
            print(imu_local_idx)
            print(imu_ts)
            print(imu_retimed.T)
            print("==============")
            raise NotImplementedError("MISSING TOO MANY IMU READINGS!!!")

        for k,v in imu_local_write_bufs.items():
            # Skip q, as it is zero or non reliable
            if k == 'q':
                continue
            self.imu_ds[k][self.imu_idx:self.imu_idx+16] = v

        self.imu_idx += 16

        assert (self.imu_idx == self.imager_idx * 16)
        assert ( self.imu_ds['ts'][self.imu_idx-16] == self.ds['ts'][self.imager_idx-1] )

    def process(self, msg, topic=None, bag_time=None):
        time = self.sensor_to_global_us(msg.header.stamp.to_nsec() / 1e3)
        seq_id = msg.header.seq

        assert len(self.buffers[topic]) == 0 or seq_id-1 == self.buffers[topic][-1][1]

        if 'image' in topic:
            self.buffers[topic].append( (time, seq_id, self.get_image_from_msg(msg)) )
        elif 'imu' in topic:
            self.buffers[topic].append( (time, seq_id, self.get_imu_from_msg(msg)) )

        if len(self.buffers) == len(self.topics):
            if not self.has_started:
                if self.sync_buffers():
                    if self.wait_for_start():
                        self.init_id = {k:buf[0][1] for k,buf in self.buffers.items()}
                        self.rel_id = {}
                        for k,buf in self.buffers.items():
                            if 'image' in k:
                                self.rel_id[k] = round(buf[0][0] / 40000)
                            elif 'imu' in k:
                                self.rel_id[k] = round(buf[0][0] / 2500)
                            else:
                                raise NotImplementedError("Nothing but images and imus")

                        self.has_started = True
            else:
                self.process_buffer()

    def finish(self):
        for k, ds in self.ds.items():
            if 'ts' in k:
                ds.resize((self.imager_idx,))
                continue

            final_ds_shape = tuple([self.imager_idx] + list(ds['shape'])[1:])
            ds['ds'].resize(final_ds_shape)

        self.imu_ds['av'].resize((self.imu_idx,3))
        self.imu_ds['la'].resize((self.imu_idx,3))
        self.imu_ds['ts'].resize((self.imu_idx,))

    def primary_time_ds(self):
        return self.ds['ts']

    def get_image_from_msg(self, msg):
        img = bridge.compressed_imgmsg_to_cv2(msg)
        if len(img.shape) == 2:
            img = img[:, :, None]
        return img

    def get_imu_from_msg(self, msg):
        return {
                'q': np.array(
                    [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
                    ),
                'av':  np.array(
                    [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
                    ),
                'la': np.array(
                    [msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]
                    ),
                }


class Event(HDFHandler):
    UNITS_TO_SECONDS = 1e6

    def __init__(self, h5file, topic, bag, time_calib):
        super().__init__(h5file, topic, bag, time_calib)

        event_count = time_calib[topic]['nevents'] # self.get_event_count(bag, topic)
        self.chunk_size = 40000

        self.x_ds = self.group.create_dataset(
            "x",
            (event_count,),
            compression="lzf",
            dtype="u2",
            chunks=(self.chunk_size,),
            maxshape=(None,),
        )
        self.y_ds = self.group.create_dataset(
            "y",
            (event_count,),
            compression="lzf",
            dtype="u2",
            chunks=(self.chunk_size,),
            maxshape=(None,),
        )
        self.t_ds = self.group.create_dataset(
            "t",
            (event_count,),
            compression="lzf",
            dtype="i8",
            chunks=(self.chunk_size,),
            maxshape=(None,),
        )
        self.p_ds = self.group.create_dataset(
            "p",
            (event_count,),
            compression="lzf",
            dtype="i1",
            chunks=(self.chunk_size,),
            maxshape=(None,),
        )

        self.x_buf = []  #
        self.y_buf = []  #
        self.t_buf = []  #
        self.p_buf = []  #
        self.count_buf = 0

        self.start = 0
        self.prev_msg = None

        self.has_index_map = False

        self.decoder = Decoder()

    def get_event_count(self, bag, topic):
        ## TODO - get count from verify or timing script
        count = 0
        d = Decoder()
        for topic, msg, t in bag.read_messages(topics=[topic]):
            d.decode_bytes(
                msg.encoding, msg.width, msg.height, msg.time_base, msg.events
            )
            cd_events = d.get_cd_events()
            count += cd_events.shape[0]
        return count

    def add_events(self, x, y, t, p):
        msg_shape = x.shape[0]

        if msg_shape == 0:
            return

        self.x_buf.append(x)
        self.y_buf.append(y)
        self.t_buf.append(t)
        self.p_buf.append(p)
        self.count_buf += msg_shape

    def flush_buffer(self, ignore_chunk=False):
        x = np.concatenate(self.x_buf)
        y = np.concatenate(self.y_buf)
        t = np.concatenate(self.t_buf)
        p = np.concatenate(self.p_buf)

        if not ignore_chunk:
            added_idx = self.chunk_size * (x.shape[0] // self.chunk_size)
        else:
            added_idx = x.shape[0]

        self.end_idx = self.start + added_idx

        self.x_ds[self.start : self.end_idx] = x[:added_idx]
        self.y_ds[self.start : self.end_idx] = y[:added_idx]
        self.t_ds[self.start : self.end_idx] = t[:added_idx]
        self.p_ds[self.start : self.end_idx] = p[:added_idx]

        self.start = self.start + added_idx

        if not ignore_chunk:
            self.x_buf = [x[added_idx:]]
            self.y_buf = [y[added_idx:]]
            self.t_buf = [t[added_idx:]]
            self.p_buf = [p[added_idx:]]
            self.count_buf = self.count_buf - added_idx
        else:
            self.x_buf = []
            self.y_buf = []
            self.t_buf = []
            self.p_buf = []
            self.count_buf = 0

    def process(self, msg, topic=None, bag_time=None):
        self.decoder.decode_bytes(
            msg.encoding, msg.width, msg.height, msg.time_base, msg.events
        )
        cd_events = self.decoder.get_cd_events()
        trig_events = self.decoder.get_ext_trig_events()

        t = self.sensor_to_global_us( cd_events['t'] ).astype(int)
        has_started = t >= 0

        x = cd_events['x'][has_started]
        y = cd_events['y'][has_started]
        t = t[has_started]
        p = cd_events['p'][has_started]

        self.add_events(x, y, t, p)

        if self.count_buf > self.chunk_size:
            self.flush_buffer()

    def finish(self):
        self.flush_buffer(ignore_chunk=True)
        self.x_ds.resize( (self.end_idx,) )
        self.y_ds.resize( (self.end_idx,) )
        self.t_ds.resize( (self.end_idx,) )
        self.p_ds.resize( (self.end_idx,) )

        self.compute_ms_index_map()

    def primary_time_ds(self):
        raise NotImplementedError("This would lead to something very expensive being computed")

    def index_map(self, index_map_ds, time_ds):
        index_map_ds_cl = 0

        remaining_times = time_ds[...].copy()
        cur_loc = 0
        chunk_size = 10000000
        num_events = self.t_ds.shape[0]

        while remaining_times.shape[0] > 0 and cur_loc < num_events:
            end = min( num_events, cur_loc+chunk_size )
            idx = cur_loc + np.searchsorted(self.t_ds[cur_loc:end], remaining_times)

            idx_remaining = (idx == end)
            idx_found = (idx < end)

            index_map_ds[index_map_ds_cl:index_map_ds_cl+idx_found.sum()] = idx[idx_found]

            remaining_times = remaining_times[idx_remaining]
            cur_loc = cur_loc + chunk_size
            index_map_ds_cl += idx_found.sum()

    def compute_ms_index_map(self):
        time_ds = np.arange(0, int(np.floor(self.t_ds[-1] / 1e3)) ) * 1e3
        index_map_ds = self.group.create_dataset("ms_map_idx", time_ds.shape, dtype="u8")

        self.index_map( index_map_ds, time_ds )

    def compute_index_map(self, sensor_processor):
        time_ds = sensor_processor.primary_time_ds()
        index_map_name = time_ds.name.split("/")[-1] + "_map" + self.t_ds.name.replace("/", "_")
        index_map_ds = sensor_processor.group.create_dataset(index_map_name, time_ds.shape, dtype="u8")

        self.index_map( index_map_ds, time_ds )


def compute_temporal_index_maps(event_processor, processors):
    print("Starting index map computation")
    processors = [p for p in processors if type(p) not in [Event]]

    for p in processors:
        event_processor.compute_index_map( p )


def plot_coordinate_frames(transformation_matrices):
    """ plot the coordinate frames for verification of the transformations """
    def set_axes_equal(ax):
        extents = np.array([getattr(ax, f"get_{dim}lim")() for dim in "xyz"])
        centers = np.mean(extents, axis=1)
        radius = 0.5 * np.max(np.abs(extents[:, 1] - extents[:, 0]))
        for center, dim in zip(centers, "xyz"):
            getattr(ax, f"set_{dim}lim")(center - radius, center + radius)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    for T in transformation_matrices:
        R = T[:3, :3]
        t = T[:3, 3]
        x_axis = R[:, 0]
        y_axis = R[:, 1]
        z_axis = R[:, 2]

        ax.quiver(t[0], t[1], t[2], x_axis[0], x_axis[1], x_axis[2], 
                  color="r", length=0.1)
        ax.quiver(t[0], t[1], t[2], y_axis[0], y_axis[1], y_axis[2],
                  color="g", length=0.1)
        ax.quiver(t[0], t[1], t[2], z_axis[0], z_axis[1], z_axis[2],
                  color="b", length=0.1)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    set_axes_equal(ax)
    plt.show()


def insert_calibration(h5_file, camchain, lidar_calib, imu_chain):
    # Check that camchain is not none and it exists, and read it
    if camchain is None:
        raise ValueError("camchain is None")
    if not os.path.exists(camchain):
        raise ValueError("camchain file does not exist")

    # Get the IMU to ovc left transformation
    with open(imu_chain, "r") as yaml_file:
        imuchain_data = yaml.safe_load(yaml_file)
    assert "/ovc/left" in imuchain_data["cam0"]["rostopic"]
    ol_T_imu = np.array(imuchain_data["cam0"]["T_cam_imu"], dtype=np.float64)

    # Read the YAML file with the camera calibrations
    with open(camchain, "r") as yaml_file:
        camchain_data = yaml.safe_load(yaml_file)

    # Open the npz file with the LiDAR calibrations
    if lidar_calib is None:
        raise ValueError("lidar_calib is None")
    with np.load(lidar_calib) as lidar_data:
        T_c_l = lidar_data["T_c_l"]

    # Hold all the transformation for viz purposes
    transformation_matrices = []

    # Write the data to an HDF5 file
    with h5py.File(h5_file, "a") as hdf_file:

        # Write camera calibration data
        for cam_key, cam_data in camchain_data.items():
            rostopic = cam_data.pop("rostopic")

            if not 'T_cn_cnm1' in cam_data.keys():
                # This is the reference camera
                assert "prophesee" in rostopic and "left" in rostopic
                T_to_prophesee_left = np.array([[1, 0, 0, 0],[0, 1, 0, 0], 
                                               [0, 0, 1, 0], [0, 0, 0, 1]])
            else:
                T_to_prev = np.array(cam_data.pop("T_cn_cnm1"))
                T_to_prophesee_left = T_to_prophesee_left @ T_to_prev

            if "prophesee" in rostopic:
                camera_side = rostopic.split("/")[-2]
                cam_group = hdf_file["prophesee"][camera_side]
            elif "ovc" in rostopic:
                camera_side = rostopic.split("/")[-3]
                cam_group = hdf_file["ovc"][camera_side]

            calib_group = cam_group.create_group("calib")

            for data_key, data_value in cam_data.items():
                if data_key == "cam_overlaps":
                    continue
                if data_key == "T_cn_cnm1":
                    continue
                if isinstance(data_value, list):
                    data_value = np.array(data_value)
                calib_group[data_key] = data_value
            calib_group["T_to_prophesee_left"] = np.linalg.inv(T_to_prophesee_left)
            transformation_matrices.append(calib_group["T_to_prophesee_left"][:])
            if "ovc" in rostopic and "left" in rostopic:
                pl_T_ol = calib_group["T_to_prophesee_left"][:]

        # Compute the transofrmation from IMU to prophesee_left
        pl_T_imu = pl_T_ol @ ol_T_imu
        transformation_matrices.append(pl_T_imu)
        imu_group = hdf_file["/ovc/imu"]
        imu_calib_group = imu_group.create_group("calib")
        imu_calib_group["T_to_prophesee_left"] = pl_T_imu

        # Compute the transformation from LiDAR to T_to_prophesee_left
        pl_T_lid = pl_T_ol @ T_c_l
        # print(pl_T_lid @ np.array([[0, 0, 0, 1]]).T*1000)
        transformation_matrices.append(pl_T_lid)
        lidar_group = hdf_file["ouster"]
        lidar_calib_group = lidar_group.create_group("calib")
        lidar_calib_group["T_to_prophesee_left"] = pl_T_lid

    # plot_coordinate_frames(transformation_matrices)


def process_bag(filename, h5fn=None, offsetfn=None, 
                camchainfn=None, lidar_calib=None,
                imuchainfn=None,
                verbose=True, disable_events=False):
    # Get time offset info
    if offsetfn is None:
        offsetfn = os.path.splitext(args.bag)[0] + "_time_calib.yml"

    print(Fore.GREEN, "Time offset: ", offsetfn, Style.RESET_ALL)
    with open(offsetfn, 'r') as f:
        time_calib = yaml.safe_load(f)

    # print camchain and lidar_calib files
    if camchainfn:
        print(Fore.GREEN, "Camchain: ", camchainfn, Style.RESET_ALL)

    if imuchainfn:
        print(Fore.GREEN, "Imu chain: ", imuchainfn, Style.RESET_ALL)

    if lidar_calib:
        print(Fore.GREEN, "Lidar calib: ", lidar_calib, Style.RESET_ALL)

    # Open bag
    bag = rosbag.Bag(filename)
    if h5fn is None:
        h5fn = os.path.splitext(args.bag)[0] + ".h5"

    print(Fore.GREEN, "Output file: ", h5fn, Style.RESET_ALL)
    # Print only the bag without the path
    bag_name = bag.filename.split('/')[-1]

    h5file = h5py.File(h5fn, "w")

    # Add metadata to h5 file
    version = subprocess.check_output(["git", "describe", "--tags", "--long"]).decode("utf-8").strip()
    h5file.attrs["version"] = version
    h5file.attrs["raw_bag_name"] = bag_name
    h5file.attrs["creation_date"] = str(datetime.now())

    processors = [
        Ouster(h5file, ["/os_node/lidar_packets","/os_node/imu_packets"], bag, time_calib),
        OVC(h5file, bag, time_calib),
        # GPS(h5file, bag, time_calib),
    ]

    # processors[-1].attach_ovc(processors[-2])

    if not disable_events and not ("calib" in bag_name and "lidar" in bag_name):
        processors.append(Event(h5file, "/prophesee/right/events", bag, time_calib))
        processors.append(Event(h5file, "/prophesee/left/events", bag, time_calib))
        HAS_EVENTS = True
    else:
        print(Fore.YELLOW, "Events are not written for lidar_calib bags",
              Style.RESET_ALL)
        HAS_EVENTS = False

    # Get list of topics that will be processed
    topic_to_processor = {}
    for p in processors:
        for t in p.topics:
            topic_to_processor[t] = p
    topics = list(topic_to_processor.keys())

    print(Fore.GREEN, "Start processing...", Style.RESET_ALL)
    for topic, msg, t in tqdm(bag.read_messages(topics=topics),
                              disable=not verbose,):
        topic_to_processor[topic].process(msg, topic, t)

    for p in processors:
        p.finish()

    if HAS_EVENTS:
        print(Fore.GREEN, "Computing index maps for lidar and imagers")
        compute_temporal_index_maps(processors[-1], processors)
        compute_temporal_index_maps(processors[-2], processors)

    # Insert the calibration into the bags
    if "calib" not in bag_name and args.camchain is not None:
        insert_calibration(h5fn, camchainfn, lidar_calib, imuchainfn)

    # Set permissions
    os.chmod(h5fn, 0o666)
    print(Fore.GREEN, "Output file: ", h5fn, Style.RESET_ALL)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", help="ROS bag name")
    parser.add_argument("--h5fn", default=None, help="HDF5 file name")
    parser.add_argument("--offset", default=None, help="Time offset File")
    parser.add_argument("--camchain", default=None, help="Camchain file")
    parser.add_argument("--lidar_calib", default=None,
                        help="LiDAR Calibration File")
    parser.add_argument("--imu_chain", default=None,
                        help="IMU chain file")
    parser.add_argument("--verbose", action="store_true",
                        help="Verbose output")
    parser.add_argument("--disable_events", action="store_true",
                        help="Disable events. Useful for debugging.")
    args = parser.parse_args()

    print(Fore.GREEN, "Processing bag: ", args.bag, Style.RESET_ALL)

    process_bag(args.bag, args.h5fn, args.offset,
                args.camchain, args.lidar_calib, args.imu_chain,
                args.verbose, args.disable_events)
