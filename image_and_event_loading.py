import h5py
import argparse

parser = argparse.ArgumentParser()

parser.add_argument("--data_h5", required=True, help="H5 file path with sensor data")
parser.add_argument("--idx", required=True, type=int, default=100, help="Image index to load")
parser.add_argument("--n_events", required=True, type=int, default=200000, help="Number of events to load (centered on the image)")

# Start by loading a file through h5py. The compression used is available within the base h5py package.
f = h5py.File(args.data_h5,'r')

# Grab the image itself
left_image = f['/ovc/left/data'][args.idx]

# Find the index in the event stream that correlates with the image time
left_event_idx = f['/ovc/ts_map_prophesee_left_t'][args.idx]

# Compute the start and stop index within the event stream
half_n_events = args.n_events // 2
start = left_event_idx - half_n_events
stop = left_event_idx + half_n_events

# Load 200k events surrounding the central timestamp
left_events_x = f['/prophesee/left/x'][start:stop]
left_events_y = f['/prophesee/left/y'][start:stop]
left_events_t = f['/prophesee/left/t'][start:stop]
left_events_p = f['/prophesee/left/p'][start:stop]
