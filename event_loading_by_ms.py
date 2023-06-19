import h5py
import argparse

parser = argparse.ArgumentParser()

parser.add_argument("--data_h5", required=True, help="H5 file path with sensor data")
parser.add_argument("--ms", required=True, type=int, default=100, help="Millisecond to load from")

args = parser.parse_args()

# Start by loading a file through h5py. The compression used is available within the base h5py package.
f = h5py.File(args.data_h5,'r')

# Get the locations for starting at 1000 ms and stopping at 1001 ms
event_start_idx = f['/prophesee/left/ms_map_idx'][args.ms]
event_stop_idx = f['/prophesee/left/ms_map_idx'][args.ms+1]

# Load 200k events surrounding the central timestamp
left_events_x = f['/prophesee/left/x'][event_start_idx:event_stop_idx]
left_events_y = f['/prophesee/left/y'][event_start_idx:event_stop_idx]
left_events_t = f['/prophesee/left/t'][event_start_idx:event_stop_idx]
left_events_p = f['/prophesee/left/p'][event_start_idx:event_stop_idx]
