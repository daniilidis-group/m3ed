import os
import argparse

from ouster import client, pcap
from ouster.sdk.examples.open3d import colorize

import open3d as o3d

import h5py

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--data_h5', required=True, help="The h5 file to load")
    parser.add_argument('--out_file', help="The file to save the cloud into", default='/tmp/test.pcd')

    args = parser.parse_args()

    f = h5py.File(args.data_h5, 'r')

    ouster_sweep = f['/ouster/data'][0][...]
    ouster_metadata_str = f['/ouster/metadata'][...].tolist()

    metadata = client.SensorInfo(ouster_metadata_str)

    # precompute xyzlut to save computation in a loop
    xyzlut = client.XYZLut(metadata)

    packets = client.Packets([client.LidarPacket(opacket, metadata) for opacket in ouster_sweep], metadata)
    scan = next(iter(client.Scans(packets)))

    xyz = xyzlut(scan.field(client.ChanField.RANGE))
    color = colorize(scan.field(client.ChanField.SIGNAL)) # SIGNAL or REFLECTIVITY

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz.reshape(-1,3))
    pcd.colors = o3d.utility.Vector3dVector(color.reshape((-1,3)))

    o3d.io.write_point_cloud(args.out_file, pcd)

    o3d.visualization.draw_geometries([pcd])
