![alt text](https://github.com/daniilidis-group/m3ed/blob/main/M3ED_banner.webp)

# M3ED


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
