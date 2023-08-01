#!/usr/bin/env python3

import h5py
import pdb
import os
import numpy as np
import cv2
from scipy.spatial.transform import Rotation

from ouster.sdk.examples.open3d import viewer_3d
from ouster.client import LidarPacket, SensorInfo, Scans, Packets, ChanField, XYZLut, _utils
import open3d as o3d

from util import kalibr_to_opencv, get_cloud, load_clouds, load_trajectory, transform_inv, filter_global_cloud

def get_info_via_fasterlio(args):
    exp_name = os.path.basename(args.tmp_folder)

    import yaml
    with open(args.tmp_folder+"/"+exp_name+"_time_corrections.yaml", 'r') as f:
        timesync = yaml.safe_load(f)

    traj_file = args.tmp_folder+"/"+exp_name+".traj"
    traj = load_trajectory(traj_file, timesync)
    times = np.array([t['timestamp'] for t in traj])
    orig_times = np.array([t['orig_ts'] for t in traj])


    inds = list(range(traj[0]['idx'], traj[-1]['idx'], 20))
    print(inds)

    cloud = o3d.geometry.PointCloud()
    points = [load_clouds(args.out_folder+"/local_scans", idx) for idx in inds]
    # points = filter_global_cloud(points, method='dbscan', method_kwargs={"eps":0.10, "min_points":10})
    points = np.concatenate(points)
    cloud.points = o3d.utility.Vector3dVector(points)
    cloud.paint_uniform_color([1, 0.706, 0])
    cloud.estimate_normals()

    frames = []
    for idx in inds:
        coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0).transform( traj[idx]['L0_T_lidar'] )
        frames.append(coord)

    # points = (points @ traj_pose['Ln_T_L0'][:3,:3].T) + traj_pose['Ln_T_L0'][:3,3]
    # cloud = cloud.transform( traj_pose['Ln_T_L0'] )

    return cloud, frames

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--h5fn', help='HDF5 File')
    parser.add_argument('--percentage', type=float, help='Lidar index as percentage of sequence')
    parser.add_argument('--verbose', action='store_true', help='Set Verbose Mode')
    parser.add_argument('--confirm_only', action='store_true', help='Set to only confirm')
    parser.add_argument('--confirm_fn', help='Filename of figure to save out', default='tmp.pdf')
    parser.add_argument('--npz_fn', help='Save settings for the npz')
    parser.add_argument('--use_pcd', action='store_true', help="Use PCD from fasterlio")
    parser.add_argument('--tmp_folder', help='Tmp folder within M3ED_Build')
    parser.add_argument('--out_folder', help='Tmp folder within M3ED_Build')
    args = parser.parse_args()

    flio_cloud, flio_traj = get_info_via_fasterlio(args)

    o3d.visualization.draw_geometries([flio_cloud]+flio_traj)
