![alt text](https://github.com/daniilidis-group/m3ed/blob/main/M3ED_banner.webp)

# M3ED

## Intro

M3ED provides high-quality synchronized and labeled data from multiple platforms, including wheeled ground vehicles, legged robots, and aerial robots, operating in challenging conditions such as driving along off-road trails, navigating through dense forests, and executing aggressive flight maneuvers.

M3ED processed data, raw data, and code are available to
[download](https://m3ed.io/download/). For more information about our dataset,
please visit [https://m3ed.io](https://m3ed.io/).

## Sample Loading

M3ED provides helper datasets within the HDF5 files to more easily load synchronized data.

For loading synchronized images:
```
python3 image_and_event_loading.py --data_h5 <sample_hdf5> --idx 500 --n_events 200000
```

For loading events in 1ms chunks:
```
python3 event_loading_by_ms.py --data_h5 <sample_hdf5> --ms 1000
```
