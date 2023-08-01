#!/usr/bin/env python3

""" GUI for hand calibrating the LiDAR with the different sequences """

import h5py
import pdb
import os
import numpy as np
import cv2
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Slerp

from ouster.sdk.examples.open3d import viewer_3d
from ouster.client import LidarPacket, SensorInfo, Scans, Packets, ChanField, XYZLut, _utils
import open3d as o3d

from util import kalibr_to_opencv, get_cloud, load_clouds, load_trajectory, load_imu_based_calib

class ManualCalibrator:
    def __init__(self, cloud_image_list, camera_calib, R_imu, fn=None):
        # Prepare point cloud for better plotting
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(1,-30,-30), max_bound=(150,30,30))

        self.clouds = [c[0].crop(bbox) for c in cloud_image_list]
        self.cloud_points = [np.array(c.points) for c in self.clouds]
        self.cloud_colors = [np.array(c.colors) for c in self.clouds]

        self.images = [c[1] for c in cloud_image_list]

        self.camera_calib = camera_calib
        self.point_scale = 1

        self.R_ideal = np.array([[ 0, 0, 1],
                                 [-1, 0, 0],
                                 [ 0,-1, 0]])

        self.fn = fn
        if os.path.exists(self.fn):
            lidar_calib_info = np.load(self.fn)
            if not 'R_imu' in lidar_calib_info or not 'origin' in lidar_calib_info:
                self.rotvec = np.zeros(3)
                self.t = lidar_calib_info['t_c_l']
                self.R_imu = R_imu
            else:
                if lidar_calib_info['origin'] == self.fn:
                    self.R_imu = lidar_calib_info['R_imu']
                    self.rotvec = lidar_calib_info['r_c_l']
                    self.t = lidar_calib_info['t_c_l']
                else:
                    self.R_imu = R_imu
                    self.rotvec = np.zeros(3)
                    self.t = lidar_calib_info['t_c_l']
        else:
            self.rotvec = np.zeros(3)
            self.t = np.zeros(3)
            self.R_imu = R_imu

        print("Initializing")
        print(self.rotvec)
        print(self.t)
        print(self.R_imu)

        self.masks = [None] * len(self.clouds)

    def show_cloud(self):
        o3d.visualization.draw_geometries(self.clouds)

    def project_points(self, cloud_points, mask, cloud_colors=None,
                       confirm_only=False):
        T_c_l = np.eye(4)
        T_c_l[:3,:3] = (Rotation.from_matrix(self.R_imu) * Rotation.from_rotvec(self.rotvec)).as_matrix().T
        T_c_l[:3,3] = self.t

        if not confirm_only:
            np.savez(self.fn, T_c_l=T_c_l, r_c_l=self.rotvec, t_c_l=self.t, R_imu=self.R_imu, origin=self.fn)

        transformed_cloud = cloud_points @ (Rotation.from_matrix(self.R_imu) * Rotation.from_rotvec(self.rotvec)).as_matrix() + self.t

        rvecs = cv2.Rodrigues(np.eye(3))[0]
        tvecs = np.zeros((3,1))

        imgpts, jac = cv2.projectPoints(
                transformed_cloud.reshape(-1,1,3),
                rvecs,
                tvecs,
                self.camera_calib['camera_matrix'],
                self.camera_calib['distortion_coefficients'])

        if mask is None:
            mask = (imgpts[:,:,0] > 0) * (imgpts[:,:,0]<1280) #  * (imgpts[:,:,1] > 0) * (imgpts[:,:,1]<720)
            mask = mask.squeeze()

        imgpts = imgpts.squeeze()[mask]
        depth = transformed_cloud[mask][:,2]
        colors = np.log(depth) # self.cloud_colors[mask]
        colors = colors / colors.max()

        return imgpts, colors, mask

    def update_transform(self, val):
        self.t = np.array([self.slider_l_x.val, self.slider_l_y.val, self.slider_l_z.val])
        self.rotvec = np.array([self.slider_r_x.val, self.slider_r_y.val, self.slider_r_z.val])

        self.point_scale = self.slider_scale.val

        for i in range(len(self.clouds)):
            imgpts, colors, mask = self.project_points(self.cloud_points[i], self.masks[i])

            self.scatter_plts[i].set_offsets( imgpts )
            self.scatter_plts[i].set_sizes( np.ones((imgpts.shape[0],)) * self.point_scale)

        self.fig.canvas.draw_idle()

    def plot(self, confirm_only=False, figfn='tmp.pdf'):
        if confirm_only:
            import matplotlib as mpl
            mpl.use('Agg')
            import matplotlib.pyplot as plt
            imgpts, colors,_ = self.project_points(self.cloud_points[0], None,
                                                 confirm_only=confirm_only)
            self.fig, axes = plt.subplots(1,1)
            axes.imshow(self.image.squeeze(), cmap='gray')
            axes.scatter(x=imgpts[:,0], y=imgpts[:,1], c=colors, s=0.5)
            self.fig.savefig(figfn, dpi=400)
        else:
            import matplotlib.pyplot as plt

            self.fig, axes = plt.subplots(8,len(self.clouds),height_ratios=[25,1,1,1,1,1,1,1],width_ratios=[1]*len(self.clouds), squeeze=False)

            self.img_plts = []
            self.scatter_plts = []
            for i in range(len(self.clouds)):
                imgpts, colors, mask = self.project_points(self.cloud_points[i], self.masks[i])
                self.masks[i] = mask
                self.img_plts.append(axes[0,i].imshow(self.images[i], cmap='gray'))
                self.scatter_plts.append(axes[0,i].scatter(x=imgpts[:,0], y=imgpts[:,1], c=colors, s=self.point_scale))
                axes[0,i].set_xlim([-100,1380])
                axes[0,i].set_ylim([820,-100])

            center_axis_id = len(self.clouds) // 2
            from matplotlib.widgets import Slider, Button
            self.slider_l_x = Slider(axes[1, center_axis_id], "X", -0.40, 0.40, valinit=self.t[0])
            self.slider_l_y = Slider(axes[2, center_axis_id], "Y", -0.40, 0.40, valinit=self.t[1])
            self.slider_l_z = Slider(axes[3, center_axis_id], "Z", -0.40, 0.40, valinit=self.t[2])

            self.slider_r_x = Slider(axes[4, center_axis_id], "Rx", -0.05, 0.05, valinit=self.rotvec[0])
            self.slider_r_y = Slider(axes[5, center_axis_id], "Ry", -0.15, 0.07, valinit=self.rotvec[1])
            self.slider_r_z = Slider(axes[6, center_axis_id], "Rz", -0.15, 0.15, valinit=self.rotvec[2])

            self.slider_scale = Slider(axes[7, center_axis_id], "S", 0.5, 20, valinit=2)

            self.slider_l_x.on_changed(self.update_transform)
            self.slider_l_y.on_changed(self.update_transform)
            self.slider_l_z.on_changed(self.update_transform)
            self.slider_r_x.on_changed(self.update_transform)
            self.slider_r_y.on_changed(self.update_transform)
            self.slider_r_z.on_changed(self.update_transform)
            self.slider_scale.on_changed(self.update_transform)

            plt.show()

def get_images(h5f, idx):
    return {
            '/ovc/left': h5f['/ovc/left/data'][idx,...],
            '/ovc/right': h5f['/ovc/right/data'][idx,...],
            '/ovc/ts': h5f['/ovc/ts'][idx,...],
           }

def get_info_via_scans(hdf5_file, args, ts):
    lidar_start_ts = hdf5_file['/ouster/ts_start']
    lidar_end_ts = hdf5_file['/ouster/ts_end']

    lidar_mid_times = ( lidar_start_ts[...] + lidar_end_ts[...] ) /  2
    lidar_idx = np.argmin( np.abs(lidar_mid_times - ts) )
    lidar_mid_time = lidar_mid_times[lidar_idx]

    camera_ts = hdf5_file['/ovc/ts']

    cam_idx = np.argmin( np.abs(lidar_mid_time - camera_ts) )

    cloud = get_cloud(hdf5_file, lidar_idx)
    images = get_images(hdf5_file, cam_idx)
    return cloud, images

def get_info_via_fasterlio(hdf5_file, args, percentage):
    exp_name = os.path.basename(args.tmp_folder)

    import yaml
    with open(args.tmp_folder+"/"+exp_name+"_time_corrections.yaml", 'r') as f:
        timesync = yaml.safe_load(f)

    traj_file = args.tmp_folder+"/"+exp_name+".traj"
    traj = load_trajectory(traj_file, timesync, hdf5_file)
    orig_times = np.array([t['orig_ts'] for t in traj])
    orig_corrected_times = np.array([t['orig_corrected_ts'] for t in traj])

    lidar_times = np.array([t['timestamp'] for t in traj])
    # nominal_times = (np.arange(lidar_times.shape[0])+1)* 100000
    # offset = (lidar_times - nominal_times)[0]
    # offset = (offset/1000).round() * 1000
    # lidar_times = nominal_times + offset

    ovc_times = hdf5_file['/ovc/ts'][...]

    from scipy.spatial.distance import cdist
    all_pair_times = cdist(lidar_times[:,None], ovc_times[:,None])
    lidar_to_camera = np.argmin(np.abs(all_pair_times),axis=1)
    lidar_to_camera = np.searchsorted(ovc_times, lidar_times )
    lidar_to_camera[lidar_to_camera == 0] = -1
    lidar_to_camera[lidar_to_camera >= ovc_times.shape[0]] = -1

    # lidar_times = lidar_times[lidar_to_camera < ovc_times.shape[0]]
    # lidar_to_camera = lidar_to_camera[lidar_to_camera < ovc_times.shape[0]]

    closest_ovc_times = ovc_times[lidar_to_camera]
    closest_time_diffs = closest_ovc_times - lidar_times

    lidar_idx = int(percentage * lidar_to_camera.shape[0])
    cam_idx = lidar_to_camera[lidar_idx]
    lidar_idx += 0

    if np.abs(lidar_times[lidar_idx] - ovc_times[cam_idx]) > 20000:
        print("Foo")
        lidar_idx += 1
        cam_idx = lidar_to_camera[lidar_idx]

    traj_entry = traj[lidar_idx]
    lidar_time = traj_entry['timestamp']

    Lnm1_T_L0 = traj[lidar_idx-1]['Ln_T_L0'] # time = -100000
    Ln_T_L0 = traj_entry['Ln_T_L0'] # time = 0
    Lnp1_T_L0 = traj[lidar_idx+1]['Ln_T_L0'] # time = 100000

    R_T_R0 = Rotation.from_matrix([
            Lnm1_T_L0[:3,:3],
            Ln_T_L0[:3,:3],
            Lnp1_T_L0[:3,:3],
             ])
    slerp = Slerp([-100000,0,100000], R_T_R0) # setup slerp interpolation

    rel_cam_time = (ovc_times[cam_idx] - lidar_time)

    Lc_T_L0 = np.eye(4)
    Lc_T_L0[:3,:3] = slerp(rel_cam_time).as_matrix() # interpolate to current time

    if rel_cam_time < 0:
        a, b, c = (-rel_cam_time/100000), ((100000+rel_cam_time)/100000), 0.0
    elif rel_cam_time > 0:
        a, b, c = 0.0, ((100000-rel_cam_time)/100000), ((rel_cam_time)/100000)
    else:
        a, b, c = 0.0, 1.0, 0.0

    Lc_T_L0[:3,3:] = a * Lnm1_T_L0[:3,3:] + b * Ln_T_L0[:3,3:] + c * Lnp1_T_L0[:3,3:]

    cloud = o3d.geometry.PointCloud()
    points = [load_clouds(args.out_folder+"/local_scans", idx) for idx in range(traj_entry['idx'], traj_entry['idx']+4)]
    points = np.concatenate(points)

    points = (points @ Lc_T_L0[:3,:3].T) + Lc_T_L0[:3,3]

    cloud.points = o3d.utility.Vector3dVector(points)
    cloud.paint_uniform_color([1, 0.706, 0])

    mesh, pt_map = cloud.hidden_point_removal(np.zeros((3,)), 10000. ) # camera_location['Ln_T_L0'][:3,3], 1000000.0 )
    cloud = cloud.select_by_index(pt_map)

    images = get_images(hdf5_file, cam_idx)

    print("{ cam_idx:", cam_idx, ", lidar_idx: ", lidar_idx)
    print(" dt ", ovc_times[cam_idx] - lidar_time)

    return cloud, images['/ovc/left']

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--h5fn', help='HDF5 File')
    parser.add_argument('--percentage', nargs='+',type=float, help='Lidar index as percentage of sequence')
    parser.add_argument('--verbose', action='store_true', help='Set Verbose Mode')
    parser.add_argument('--confirm_only', action='store_true', help='Set to only confirm')
    parser.add_argument('--confirm_fn', help='Filename of figure to save out', default='tmp.pdf')
    parser.add_argument('--npz_fn', help='Save settings for the npz')
    parser.add_argument('--use_pcd', action='store_true', help="Use PCD from fasterlio")
    parser.add_argument('--tmp_folder', help='Tmp folder within M3ED_Build')
    parser.add_argument('--out_folder', help='Tmp folder within M3ED_Build')
    args = parser.parse_args()

    hdf5_file = h5py.File(args.h5fn, 'r')

    lidar_end_ts = hdf5_file['/ouster/ts_end']

    clouds_and_images = [get_info_via_fasterlio(hdf5_file, args, p) for p in args.percentage]

    left_h5_calib = hdf5_file['/ovc/left/calib']
    K = np.eye(3)
    np.fill_diagonal(K[:2,:2], left_h5_calib['intrinsics'][:2])
    K[:2,2] = left_h5_calib['intrinsics'][2:]
    D = np.zeros(5)
    D[:4] = left_h5_calib['distortion_coeffs']
    left_cam_calib = {
        "camera_matrix": K,
        "distortion_coefficients": D,
    }

    R_imu = load_imu_based_calib(hdf5_file)

    mc = ManualCalibrator(clouds_and_images, left_cam_calib, R_imu, args.npz_fn)
    mc.plot(args.confirm_only, args.confirm_fn)
