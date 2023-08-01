import roslib
import rospy
import rosbag
from sensor_msgs.msg import CompressedImage, Image

import argparse
import os
import shutil

import h5py
import numpy as np
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()
import pdb
from collections import defaultdict

from tqdm import tqdm

def get_ouster_info(bag, topic="/os_node/metadata"):
    for topic, msg, t in bag.read_messages(topics=[topic]):
        ouster_metadata = msg.data
    return ouster_metadata

def process_bag(filename, ouster_fn=None):
    # Open bag
    bag = rosbag.Bag(filename)
    metadata = get_ouster_info(bag)

    print("Saving metadata to -- %s" % ouster_fn)
    with open(ouster_fn, 'w') as f:
        f.write(metadata)

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', help='ROS bag name')
    parser.add_argument('--ouster_fn', default="/tmp/ouster_metadata.json", help='Ouster Metadata Name')
    args = parser.parse_args()

    process_bag(args.bag, args.ouster_fn)
