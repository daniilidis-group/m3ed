import os
import pdb
from functools import reduce
import multiprocessing
import subprocess
from datetime import datetime

import argparse

import h5py
import numpy as np
import open3d as o3d
import pandas as pd
import cv2
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

from tqdm import tqdm

from util import transform_inv, merge_pcs, load_clouds, load_trajectory, filter_global_cloud

def get_view(L0_points, camera_location, camera_calib):

    scan_aggregation = camera_calib["scan_aggregation"]

    assert isinstance(scan_aggregation, int) and  scan_aggregation >= 4 and scan_aggregation <= 400

    # print(f" - scan_aggregation: {scan_aggregation}")

    cam_idx = camera_location['idx']
    start_idx = max(0,cam_idx-scan_aggregation)
    stop_idx = min(cam_idx+scan_aggregation, len(L0_points))

    L0_cloud = o3d.geometry.PointCloud()
    L0_cloud.points = o3d.utility.Vector3dVector(np.concatenate( L0_points[start_idx:stop_idx] ))

    Ln_cloud = L0_cloud.transform(camera_location['Ln_T_L0'])
    Cn_cloud = Ln_cloud.transform(camera_calib["Cn_T_lidar"])

    z_limit = 250
    camera_fov = 64/180*np.pi # The ec FoV is 62, but we are taking a bit more
    xy_limit = z_limit * np.tan(camera_fov/2)

    bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-xy_limit, -xy_limit, 0.05),
                                               max_bound=(xy_limit, xy_limit, z_limit))
    Cn_cloud_crop = Cn_cloud.crop(bbox)

    diameter = np.linalg.norm(np.array(Cn_cloud_crop.get_max_bound()) -
                              np.array(Cn_cloud_crop.get_min_bound()))
    # https://github.com/isl-org/Open3D/blob/ff22900c958d0216c85305ee8b3841e5699f9d57/examples/python/geometry/point_cloud_hidden_point_removal.py#L24C4-L24C4
    hpr_radius = diameter * 100

    mesh, pt_map = Cn_cloud_crop.hidden_point_removal(np.zeros((3,)), hpr_radius)
    Cn_cloud_HPR = Cn_cloud_crop.select_by_index(pt_map)

    camera_points = np.array(Cn_cloud_HPR.points)

    rvecs = np.zeros((3,1)) # cv2.Rodrigues(np.eye(3))[0]
    tvecs = np.zeros((3,1))

    imgpts, _ = cv2.projectPoints(camera_points, rvecs, tvecs,
            camera_calib['K'], camera_calib['D'])

    width, height = camera_calib['resolution']

    imgpts = imgpts[:,0,:]
    valid_points = (imgpts[:, 1] >= 0) & (imgpts[:, 1] < height) & \
                   (imgpts[:, 0] >= 0) & (imgpts[:, 0] < width)
    imgpts = imgpts[valid_points,:]
    depth = camera_points[valid_points,2]

    idx_sorted = np.argsort(depth)

    depth_sorted = depth[idx_sorted]
    imgpts_sorted = imgpts[idx_sorted,:]

    images = []

    scales = 4

    for i in range(scales):
        images = [np.repeat(img,2,axis=1).repeat(2,axis=0) for img in images]

        cscale = 2 ** (scales - i - 1)
        image = np.zeros([height // cscale, width // cscale]) + np.inf
        image[imgpts[:,1].astype(int) // cscale, imgpts[:,0].astype(int) // cscale] = depth
        images.append(image)

    image[ image==0.0 ] = np.inf

    return image

def get_view_mp(mp_tuple):
    return get_view(*mp_tuple)

def index_map(event_t_ds, index_map_ds, time_ds):
    index_map_ds_cl = 0

    remaining_times = time_ds[...].copy()
    cur_loc = 0
    chunk_size = 10000000
    num_events = event_t_ds.shape[0]

    while remaining_times.shape[0] > 0 and cur_loc < num_events:
        end = min( num_events, cur_loc+chunk_size )
        idx = cur_loc + np.searchsorted(event_t_ds[cur_loc:end], remaining_times)

        idx_remaining = (idx == end)
        idx_found = (idx < end)

        index_map_ds[index_map_ds_cl:index_map_ds_cl+idx_found.sum()] = idx[idx_found]

        remaining_times = remaining_times[idx_remaining]
        cur_loc = cur_loc + chunk_size
        index_map_ds_cl += idx_found.sum()

def create_h5(h5fn, flio_trajectory, Cn_T_lidar, topic):
    Ln_T_cam = transform_inv(Cn_T_lidar)
    number_samples = len(flio_trajectory)

    h5file = h5py.File(h5fn, 'w')
    # Add metadata
    version = subprocess.check_output(["git", "describe", "--tags", "--long"]).decode("utf-8").strip()
    h5file.attrs["version"] = version
    h5file.attrs["creation_date"] = str(datetime.now())

    lidar_trajectory = h5file.create_dataset("Ln_T_L0", (number_samples,4,4), dtype='f8')
    cam_trajectory = h5file.create_dataset("Cn_T_C0", (number_samples,4,4), dtype='f8')
    h5file.attrs['cam_name'] = topic
    times = h5file.create_dataset("ts", (number_samples,), dtype='f8')

    for i, ft in enumerate(flio_trajectory):
        lidar_trajectory[i] = ft['Ln_T_L0']
        cam_trajectory[i] = Cn_T_lidar @ ft['Ln_T_L0'] @ Ln_T_cam
        times[i] = ft['timestamp']

    return h5file, number_samples

def save_depth_to_h5(h5file, num_samples, topic, map_data, resolution, num_mp=1,
                     verbose=False):
    depth = h5file.create_group("depth")

    topic = "/".join( topic.split("/")[0:3] )

    depth_shape = (num_samples,resolution[1],resolution[0])
    chunk_shape = (1,resolution[1],resolution[0])
    topic_ds = depth.create_dataset(topic[1:], depth_shape, dtype='f4', chunks=chunk_shape, compression='lzf' )

    if num_mp == 1:
        for i, packed_data in tqdm(enumerate(map_data), total=num_samples,
                                   disable=not verbose):
            topic_ds[i,...] = get_view_mp(packed_data)
    else:
        pool = multiprocessing.Pool(processes=num_mp)

        pool_iter = pool.imap(get_view_mp, map_data)

        for i, img in tqdm(enumerate(pool_iter), total=num_samples,
                           disable=not verbose):
            topic_ds[i,...] = img

        pool.close()

def check_trajectory(h5file, Cn_T_lidar):
    frames_a = [o3d.geometry.TriangleMesh.create_coordinate_frame()]
    frames_a_record = []
    frames_b = []
    frames_b_record = []
    for i in range(h5file['Ln_T_L0'].shape[0]):
        frames_a_record.append( transform_inv(h5file['Ln_T_L0'][i] @ transform_inv(Cn_T_lidar) ) )
        frames_a.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1).transform(frames_a_record[-1] ) )
        frames_a[-1].paint_uniform_color( np.array( [1.0, 0.647, 0.0] ))

    for i in range(h5file['Cn_T_C0'].shape[0]):
        frames_b_record.append( transform_inv(h5file['Cn_T_C0'][i]) )
        frames_b.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1).transform( frames_b_record[-1] ))
    o3d.visualization.draw_geometries( frames_a + frames_b )

    diff = []
    for a, b in zip(frames_a_record, frames_b_record):
        diff.append( transform_inv(a) @ b )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--pcd_folder", required=True,
                        help="Dir containing the pcd")

    parser.add_argument("--traj_fn", required=True,
                        help="Trajectory file name")

    parser.add_argument("--timesync_fn", required=True,
                        help="Time synchronization file")

    parser.add_argument("--calib_h5", required=True,
                        help="LiDAR calibration h5 file")

    parser.add_argument("--pose_h5_fn", required=True,
                        help="h5 file name")

    parser.add_argument("--depth_h5_fn",
                        help="h5 file name")

    parser.add_argument("--scan_aggregation", required=True,
                        help="scan aggregation")

    parser.add_argument("--cam", default="/prophesee/left",
                        help="Reference cam")

    parser.add_argument("--num_mp", default=1, type=int,
                        help="Number of processes. Default is 1")

    parser.add_argument("--skip", default=1, type=int,
                        help="Number of frames to skip. Default is 1")

    parser.add_argument("--only_traj", action='store_true',
                        help="If set, only trajectory will be processed")
    parser.add_argument("--verbose", action='store_true',
                        help="If set, will print progress")

    args = parser.parse_args()

    import yaml
    with open(args.timesync_fn, 'r') as f:
        timesync = yaml.safe_load(f)

    print("Loading trajectory")
    skip = args.skip
    flio_trajectory = load_trajectory(args.traj_fn, timesync)[::skip]

    print("Loading point clouds")
    clouds = load_clouds(args.pcd_folder)
    print("Filtering point clouds")
    clouds = filter_global_cloud(clouds)

    print("Loading calibs")
    calib_h5 = h5py.File(args.calib_h5, 'r')
    cam_calib = calib_h5[args.cam + "/calib"]

    prophesee_left_T_cam = calib_h5[args.cam + "/calib/T_to_prophesee_left"]
    prophesee_left_T_lidar = calib_h5["/ouster/calib/T_to_prophesee_left"]

    Cn_T_lidar = transform_inv(prophesee_left_T_cam) @ prophesee_left_T_lidar

    scan_aggregation = int(args.scan_aggregation)

    camera_calib = {"D": cam_calib['distortion_coeffs'][...],
                    "K": np.eye(3),
                    "model": cam_calib['distortion_model'][...],
                    "Cn_T_lidar": Cn_T_lidar,
                    "resolution": cam_calib['resolution'][...],
                    "scan_aggregation": scan_aggregation}

    camera_calib['K'][0, 0] = cam_calib['intrinsics'][0]
    camera_calib['K'][1, 1] = cam_calib['intrinsics'][1]
    camera_calib['K'][0, 2] = cam_calib['intrinsics'][2]
    camera_calib['K'][1, 2] = cam_calib['intrinsics'][3]

    print("Creating h5 file")
    h5file, nsamples = create_h5(args.pose_h5_fn, flio_trajectory,
                                 Cn_T_lidar, args.cam)
    check_trajectory(h5file, Cn_T_lidar)

    print("Computing time maps")
    time_map = h5file.create_dataset("ts_map" + args.cam.replace("/", '_'),
                                     h5file['ts'].shape, dtype='u8')
    index_map(calib_h5[args.cam+"/t"], time_map, h5file['ts'])
    h5file.close()

    if not args.only_traj:
        print("Starting depth image generation")
        map_data = [(clouds, t, camera_calib) for t in flio_trajectory]

        # Create depth h5 file and copy the contents from the trajectory h5
        print("Copying pose file into depth")
        depth_h5 = h5py.File(args.depth_h5_fn, 'w')
        with h5py.File(args.pose_h5_fn, 'r') as f:
            for k in f:
                f.copy(k, depth_h5)

        print("Processing depth to h5")
        # Add metadata
        version = subprocess.check_output(["git", "describe", "--tags", "--long"]).decode("utf-8").strip()
        depth_h5.attrs["version"] = version
        depth_h5.attrs["creation_date"] = str(datetime.now())
        save_depth_to_h5(depth_h5, nsamples, args.cam,
                         map_data, cam_calib['resolution'], args.num_mp,
                         args.verbose)
        depth_h5.close()
