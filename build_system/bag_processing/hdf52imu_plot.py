import os
import pdb
import argparse

import numpy as np
import h5py
import matplotlib.pyplot as plt

from scipy.interpolate import splev, splrep, barycentric_interpolate
from scipy.spatial.transform import Rotation
from scipy import signal

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


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Creates vids and imgs from h5.")
    # input h5 file
    parser.add_argument("--h5fn", help="The h5 file to convert.",
                        required=True)
    args = parser.parse_args()

    if not os.path.isfile(args.h5fn):
        sys.exit("The input h5 file %s does not exist." % args.h5fn)

    h5f = h5py.File(args.h5fn, 'r')

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

    ouster_R_ovc = align_imus(ovc_omega, ovc_accel, ouster_omega, ouster_accel)

    print(ouster_R_ovc)
    print(np.rad2deg(Rotation.from_matrix(ouster_R_ovc).as_euler('xyz')))
    print(np.linalg.det(ouster_R_ovc))

    transformed_ovc_omega = (ouster_R_ovc @ ovc_omega.T).T
    transformed_ovc_accel = (ouster_R_ovc @ ovc_accel.T).T

    fig, axes = plt.subplots(3,2,sharex=True,sharey=True)

    axes[0,0].set_title('OVC gyro')
    axes[0,0].plot( ovc_ts, transformed_ovc_omega[:,0] )
    axes[1,0].plot( ovc_ts, transformed_ovc_omega[:,1] )
    axes[2,0].plot( ovc_ts, transformed_ovc_omega[:,2] )

    axes[0,1].set_title('Ouster gyro')
    axes[0,1].plot( ouster_ts[:-1], ouster_omega[:-1,0] )
    axes[1,1].plot( ouster_ts[:-1], ouster_omega[:-1,1] )
    axes[2,1].plot( ouster_ts[:-1], ouster_omega[:-1,2] )

    fig, axes = plt.subplots(3,2,sharex=True,sharey=True)

    axes[0,0].set_title('OVC accel')
    axes[0,0].plot( ovc_ts, transformed_ovc_accel[:,0] )
    axes[1,0].plot( ovc_ts, transformed_ovc_accel[:,1] )
    axes[2,0].plot( ovc_ts, transformed_ovc_accel[:,2] )

    axes[0,1].set_title('Ouster accel')
    axes[0,1].plot( ouster_ts[:-1], ouster_accel[:-1,0] )
    axes[1,1].plot( ouster_ts[:-1], ouster_accel[:-1,1] )
    axes[2,1].plot( ouster_ts[:-1], ouster_accel[:-1,2] )

    plt.figure()
    plt.plot(ovc_ts, transformed_ovc_omega[:,0] )
    plt.plot( ouster_ts, ouster_omega[:,0])

    plt.figure()
    plt.plot(ovc_ts, transformed_ovc_omega[:,1] )
    plt.plot( ouster_ts, ouster_omega[:,1])

    plt.figure()
    plt.plot(ovc_ts, transformed_ovc_omega[:,2] )
    plt.plot( ouster_ts, ouster_omega[:,2])

    plt.show()
