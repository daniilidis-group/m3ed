import h5py
import pdb
import os
import numpy as np
import cv2
import pandas as pd
from scipy.spatial.transform import Rotation

import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button

from ouster.sdk.examples.open3d import viewer_3d
from ouster.client import LidarPacket, SensorInfo, Scans, Packets, ChanField, XYZLut, _utils
import open3d as o3d

from scipy.interpolate import splev, splrep, barycentric_interpolate
from scipy.spatial.transform import Rotation
from scipy import signal

def transform_inv(T):
    T_inv = np.eye(4)
    T_inv[:3,:3] = T[:3,:3].T
    T_inv[:3,3] = -1.0 * ( T_inv[:3,:3] @ T[:3,3] )
    return T_inv

def merge_pcs(pc_list):
    if len(pc_list) < 10:
        return reduce(lambda x,y: x+y, pc_list)
    else:
        n_pcs = len(pc_list) // 2
        return merge_pcs(pc_list[:n_pcs]) + merge_pcs(pc_list[n_pcs:])

def load_clouds(pcd_folder, idx=-1):
    pcd_files = [file for file in os.listdir(pcd_folder) if file.endswith('.pcd') and file != "scans.pcd"]
    pcd_files.sort(key=lambda x: int(x.split('_')[-1].split('.')[0]))

    if idx==-1:
        pcd_list = []
        for file in pcd_files:
            pcd_list.append( np.asarray(o3d.io.read_point_cloud(os.path.join(pcd_folder, file)).points) )

        return pcd_list
    else:
        return np.asarray(o3d.io.read_point_cloud(os.path.join(pcd_folder, pcd_files[idx])).points)


def filter_global_cloud(cloud_point_list, method='dbscan', method_kwargs={"eps":0.10, "min_points":10}):
    global_cloud = o3d.geometry.PointCloud()
    global_cloud.points = o3d.utility.Vector3dVector(np.concatenate( cloud_point_list ))

    cloud_shapes = [cp.shape[0] for cp in cloud_point_list]
    cloud_idx = np.cumsum([0]+cloud_shapes)

    if method == 'dbscan':
        labels = global_cloud.cluster_dbscan(**method_kwargs)
        keep_mask = (np.asarray(labels) >= 0)
    elif method == 'radius':
        new_cloud, labels = global_cloud.remove_radius_outlier(**method_kwargs)
        keep_mask = np.ones((len(global_cloud.points),),dtype=bool)
        keep_mask[labels] = 0
    elif method == 'statistical':
        new_cloud, labels = global_cloud.remove_statistical_outlier(**method_kwargs)
        keep_mask = np.ones((len(global_cloud.points),),dtype=bool)
        keep_mask[labels] = 0

    filtered_clouds = []

    for i in range(len(cloud_shapes)):
        start = cloud_idx[i]
        stop = cloud_idx[i+1]
        cur_cloud_filter = cloud_point_list[i][keep_mask[start:stop]]
        filtered_clouds.append(cur_cloud_filter)

    return filtered_clouds

def sensor_to_global_us(sensor_time_us, us_offset, skew_us_per_s, ovc_pps_offset):
    offset_us = sensor_time_us - us_offset - ovc_pps_offset
    new_time_us = offset_us / (1+ (skew_us_per_s/1e6))
    skew_offset_us = int((new_time_us/1e6) * skew_us_per_s)
    return offset_us - skew_offset_us

def load_trajectory(traj_fn, timesync, hdf5=None):
    # Trajectory header is
    #  #timestamp x y z q_x q_y q_z q_w
    # trajectory = pd.read_csv(traj_fn, delimiter=' ')

    dt = np.dtype([ ('#timestamp', 'f8'),
                    ('x', 'f8'),
                    ('y', 'f8'),
                    ('z', 'f8'),
                    ('q_x', 'f8'),
                    ('q_y', 'f8'),
                    ('q_z', 'f8'),
                    ('q_w', 'f8') ])
    data = np.fromfile(traj_fn, dtype=dt)
    header = data[0]
    footer = data[-1]
    assert all([(np.isnan(header[i]) and np.isnan(footer[i])) or header[i] == footer[i] for i in range(8)])
    trajectory = pd.DataFrame(data[1:-1], columns=data.dtype.names)

    flio_trajectory = []

    first_time = None

    for idx, t in trajectory.iterrows():
        L0_T_lidar = np.eye(4)
        L0_T_lidar[:3,:3] = Rotation.from_quat(
                np.array([t['q_x'], t['q_y'], t['q_z'], t['q_w']])
                ).as_matrix()
        L0_T_lidar[:3,3] = np.array( [t['x'],t['y'],t['z']] )
        Ln_T_L0 = transform_inv(L0_T_lidar)

        lidar_sync = timesync['/os_node/sys_time']['correction']
        corrected_ts = sensor_to_global_us(int(t['#timestamp'] * 1e6), lidar_sync['offset'], lidar_sync['skew'], timesync['/ovc/pps_cb']['decimator']['round_offset']*2500)

        if first_time is None:
            first_time = corrected_ts

        if hdf5 is not None:
            lidar_end_ts = hdf5['/ouster/ts_end'][...]
            closest_ind = np.searchsorted(lidar_end_ts, corrected_ts)
            closest_ind = np.argmin(np.abs(lidar_end_ts - corrected_ts))
            final_ts = lidar_end_ts[closest_ind]
        else:
            final_ts = corrected_ts

        if corrected_ts > 0:
            flio_trajectory.append({
                'idx': idx,
                "orig_ts": int(t['#timestamp']*1e6),
                "orig_corrected_ts": corrected_ts,
                "timestamp": final_ts,
                "Ln_T_L0": Ln_T_L0,
                "L0_T_lidar": L0_T_lidar,
                })

    return flio_trajectory

def convert_model_type(cam_model, dist_model):
    assert cam_model == "pinhole"

    if dist_model == "equidistant":
        return "fisheye"
    elif dist_model == "radtan":
        return "plumb_bob"
    else:
        raise NotImplementedError("[%s] is not a supported model" % dist_model)

def load_cam(cam_dict):
    intrinsics = cam_dict["intrinsics"]
    K = np.eye(3)
    np.fill_diagonal(K[:2,:2], intrinsics[:2])
    K[:2,2] = intrinsics[2:]
    M = convert_model_type(cam_dict['camera_model'], cam_dict['distortion_model'])
    D = np.zeros(5 if M == "plumb_bob" else 4)
    D[:4] = cam_dict["distortion_coeffs"]
    return K, D, M

def cam_dict(K, D, R, P, M, Size, name):
    return {
        "image_width": Size[0],
        "image_height": Size[1],
        "camera_name": name,
        "camera_matrix": K,
        "distortion_model": M,
        "distortion_coefficients": D,
        "rectification_matrix": R,
        "projection_matrix": P,
    }

def kalibr_to_opencv(calib, cam_a_topic, cam_b_topic):
    for k in calib.keys():
        topic = calib[k]['rostopic']
        if cam_a_topic in topic:
            cam_a = k
        if cam_b_topic in topic:
            cam_b = k

    K0, D0, M0 = load_cam(calib[cam_a])
    K1, D1, M1 = load_cam(calib[cam_b])

    T_01 = np.linalg.inv(np.array(calib[cam_b]["T_cn_cnm1"]))

    R = T_01[:3,:3]
    T = T_01[:3,3]
    Size = tuple(calib[cam_b]["resolution"])  # Assumes both cameras have same resolution
    assert M0 == M1

    if M0 == "plumb_bob":
        R0, R1, P0, P1 = cv2.stereoRectify(cameraMatrix1=K0, cameraMatrix2=K1,  distCoeffs1=D0, distCoeffs2=D1, imageSize=Size, R=R, T=T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)[:4]
    elif M0 == "fisheye":
        R0, R1, P0, P1 = cv2.fisheye.stereoRectify(K1=K0, K2=K1,  D1=D0, D2=D1, imageSize=Size, R=R, tvec=T, flags=cv2.CALIB_ZERO_DISPARITY)[:4]

    return {
            cam_a_topic: cam_dict(K0, D0, R0, P0, M0, Size, cam_a_topic),
            cam_b_topic: cam_dict(K1, D1, R1, P1, M1, Size, cam_b_topic),
            "baseline": T[0],
            "f": P0[0,0],
           }

def extract_all_chessboards(image):
    # image = np.tile(image[:,:,None], (1,1,3)).copy()
    image = image.copy()

    chessboards_info = []
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    for cb_config in checkerboard_configs:
        rowcol = cb_config[0]
        square_size = cb_config[1]
        border_size = cb_config[2]

        ret, corners = cv2.findChessboardCorners(image, rowcol)

        while ret:
            corners_subp = cv2.cornerSubPix(image, corners, (11, 11), (-1, -1), criteria)
            chessboards_info.append( {"corners": corners_subp, "square_size": square_size, "rowcol": rowcol, 'border_size': border_size} )

            pts = corners[[0,9,-1,-10],:,:]

            image = cv2.fillPoly(image,[pts.astype(int)],128)

            ret, corners = cv2.findChessboardCorners(image, rowcol)

    return chessboards_info

def range_for_field(f):
    if f in (ChanField.RANGE2, ChanField.SIGNAL2,
             ChanField.REFLECTIVITY2):
        return ChanField.RANGE2
    else:
        return ChanField.RANGE

def get_cloud_times(h5f, idx):
    info = SensorInfo(h5f['/ouster/metadata'][()])
    packet_buf = h5f['/ouster/data'][idx,...]
    packets = [LidarPacket(packet_buf[i], info) for i in range(packet_buf.shape[0])]
    print(packets[0].timestamp[0])
    print(packets[-1].timestamp[-1])
    print(h5f['/ouster/ts_start'][idx])
    print(h5f['/ouster/ts_end'][idx])

def get_cloud(h5f, idx):
    info = SensorInfo(h5f['/ouster/metadata'][()])
    packet_buf = h5f['/ouster/data'][idx,...]

    packets = [LidarPacket(packet_buf[i], info) for i in range(packet_buf.shape[0])]
    scans = Scans(Packets( packets, info ))
    scan = next(iter(scans))

    metadata = scans.metadata
    xyzlut = XYZLut(metadata)

    fields = list(scan.fields)
    aes = {}
    for field_ind, field in enumerate(fields):
        if field in (ChanField.SIGNAL, ChanField.SIGNAL2):
            aes[field_ind] = _utils.AutoExposure(0.02, 0.1, 3)
        else:
            aes[field_ind] = _utils.AutoExposure()

    cloud = o3d.geometry.PointCloud()
    from ouster import client
    xyz_field = scan.field(range_for_field(fields[field_ind]))
    signal_field = scan.field(fields[field_ind]).astype(float)
    xyz_destaggered = client.destagger(metadata, xyz_field)
    signal_destaggered = client.destagger(metadata, signal_field)

    xyz = xyzlut(xyz_field) # xyz_destaggered)
    key = signal_field # signal_destaggered

    aes[field_ind](key)
    from ouster.sdk.examples.open3d import colorize
    color_img = colorize(key)
    
    # prepare point cloud for Open3d Visualiser
    cloud.points = o3d.utility.Vector3dVector(xyz.reshape((-1, 3)))
    # cloud.colors = o3d.utility.Vector3dVector( np.tile(key.reshape((-1, 1)), (1,3)) )
    cloud.colors = o3d.utility.Vector3dVector( color_img.reshape((-1,3)) )

    return cloud

def get_event_image(h5f, start_idx, stop_idx):
    return

def get_images(h5f, idx):
    return {
            '/ovc/left': h5f['/ovc/left'][idx,...],
            '/ovc/right': h5f['/ovc/right'][idx,...],
            '/ovc/ts': h5f['/ovc/ts'][idx],
           }


def resample_imu(imu, imu_ts, sample_times):
    spl = splrep(imu_ts, imu)
    return splev( sample_times, spl)

def filter_imu_sample(imu, numtaps=7, f=0.10):
    coeffs = signal.firwin(numtaps, f)
    filtered = np.zeros(imu.shape)
    filtered[:,0] = signal.lfilter(coeffs, 1.0, imu[:,0])
    filtered[:,1] = signal.lfilter(coeffs, 1.0, imu[:,1])
    filtered[:,2] = signal.lfilter(coeffs, 1.0, imu[:,2])
    return filtered

def align_imus(ovc_omega, ovc_accel, ouster_omega, ouster_accel):
    """
    Solving ouster_R_ovc * ovc_omega = ouster_omega
    """
    ovc_measurements = np.concatenate( [ovc_omega] ).T
    ouster_measurements = np.concatenate( [ouster_omega] ).T

    ouster_R_ovc = (ouster_measurements @ np.linalg.pinv(ovc_measurements))
    U,S,Vh = np.linalg.svd(ouster_R_ovc)
    ouster_R_ovc = U@Vh

    ouster_R_ovc[:,0] = -ouster_R_ovc[:,0]
    ouster_R_ovc[:,2] = np.cross(ouster_R_ovc[:,0],ouster_R_ovc[:,1])

    assert np.all(np.isclose( np.linalg.det(ouster_R_ovc), 1.0 ))

    return ouster_R_ovc

def load_imu_based_calib(h5f):
    ovc_mask = np.isfinite(h5f['/ovc/imu/accel'][:,0])
    ovc_ts = h5f['/ovc/imu/ts'][...][ovc_mask]
    ovc_accel = h5f['/ovc/imu/accel'][...][ovc_mask,:]
    ovc_omega = h5f['/ovc/imu/omega'][...][ovc_mask,:]

    ouster_mask = np.isfinite(h5f['/ouster/imu/accel'][1000:-1000,0])
    ouster_ts = h5f['/ouster/imu/ts'][1000:-1000][ouster_mask]
    ouster_accel = h5f['/ouster/imu/accel'][1000:-1000][ouster_mask,:]
    ouster_omega = h5f['/ouster/imu/omega'][1000:-1000][ouster_mask,:]

    ovc_resampled_omega = np.stack( [resample_imu(ovc_omega[:,i], ovc_ts, ouster_ts) for i in range(3)], axis=-1 )
    ovc_resampled_accel = np.stack( [resample_imu(ovc_accel[:,i], ovc_ts, ouster_ts) for i in range(3)], axis=-1 )

    ouster_accel = filter_imu_sample(ouster_accel)
    ouster_omega = filter_imu_sample(ouster_omega)

    ovc_omega = filter_imu_sample(ovc_resampled_omega)
    ovc_accel = filter_imu_sample(ovc_resampled_accel)
    ovc_ts = ouster_ts

    try:
        ouster_R_ovc = align_imus(ovc_omega, ovc_accel, ouster_omega, ouster_accel)
    except:
        ouster_R_ovc = np.array([[ 0, 0, 1],
                                 [-1, 0, 0],
                                 [ 0,-1, 0]])

    return ouster_R_ovc
