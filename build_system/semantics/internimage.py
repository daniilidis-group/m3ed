# Some of these code was written with the sample code for InternImage
# Copyright (c) OpenMMLab. All rights reserved.
# See https://github.com/OpenGVLab/InternImage/blob/master/segmentation/image_demo.py

import cv2
import os
import sys
import numpy as np
import h5py
import pdb
import argparse
import subprocess
from datetime import datetime

import matplotlib.pyplot as plt

from argparse import ArgumentParser


def intern_image_model_load(args):
    import mmcv

    import mmcv_custom   # noqa: F401,F403
    import mmseg_custom   # noqa: F401,F403
    from mmseg.apis import inference_segmentor, init_segmentor, show_result_pyplot
    from mmseg.core.evaluation import get_palette
    from mmcv.runner import load_checkpoint
    from mmseg.core import get_classes
    import cv2

    # build the model from a config file and a checkpoint file

    model = init_segmentor(args.config, checkpoint=None, device=args.device)
    checkpoint = load_checkpoint(model, args.checkpoint, map_location='cpu')
    if 'CLASSES' in checkpoint.get('meta', {}):
        model.CLASSES = checkpoint['meta']['CLASSES']
    else:
        model.CLASSES = get_classes(args.palette)

    return model

def intern_image_model_inference(args, model, img):
    from mmseg.apis import inference_segmentor, init_segmentor, show_result_pyplot
    result = inference_segmentor(model, img)
    return result

def intern_image_inference_save(args, model, img, result, idx):
    import mmcv
    from mmseg.core.evaluation import get_palette
    if hasattr(model, 'module'):
        model = model.module
    img = model.show_result(img, result, palette=get_palette(args.palette),
                            show=False, opacity=args.opacity)
    mmcv.mkdir_or_exist(args.out)
    outpath = os.path.join(args.out, "%05d.png" % idx)
    cv2.imwrite(outpath, img)

def invert_map(F):
    # shape is (h, w, 2), an "xymap"
    (h, w) = F.shape[:2]
    I = np.zeros_like(F)
    I[:,:,1], I[:,:,0] = np.indices((h, w)) # identity map
    P = np.copy(I)
    for i in range(10):
        correction = I - cv2.remap(F, P, None, interpolation=cv2.INTER_LINEAR)
        P += correction * 0.5
    return P

def load_remapping(target_group, source_group):
    target_T_to_prophesee_left = target_group['T_to_prophesee_left'][...]
    source_T_to_prophesee_left = source_group['T_to_prophesee_left'][...]

    source_T_target = source_T_to_prophesee_left @ np.linalg.inv( target_T_to_prophesee_left )
    target_T_source = np.linalg.inv(source_T_target)

    target_model = target_group['camera_model']
    target_dist_coeffs = target_group['distortion_coeffs'][...]
    target_dist_model = target_group['distortion_model']
    target_intrinsics = target_group['intrinsics'][...]
    target_res = target_group['resolution'][...]
    target_Size = target_res

    target_K = np.eye(3)
    target_K[0,0] = target_intrinsics[0]
    target_K[1,1] = target_intrinsics[1]
    target_K[0,2] = target_intrinsics[2]
    target_K[1,2] = target_intrinsics[3]

    target_P = np.zeros((3,4))
    target_P[:3,:3] = target_K

    source_model = source_group['camera_model']
    source_dist_coeffs = source_group['distortion_coeffs'][...]
    source_dist_model = source_group['distortion_model']
    source_intrinsics = source_group['intrinsics'][...]
    source_res = source_group['resolution'][...]
    source_width, source_height = source_res
    source_Size = source_res

    source_K = np.eye(3)
    source_K[0,0] = source_intrinsics[0]
    source_K[1,1] = source_intrinsics[1]
    source_K[0,2] = source_intrinsics[2]
    source_K[1,2] = source_intrinsics[3]

    source_P = np.zeros((3,4))
    source_P[:3,:3] = target_K
    source_P[0,3] =  target_K[0,0] * target_T_source[0,3]
    source_P[1,3] =  target_K[1,1] * target_T_source[1,3]

    target_R = target_T_source[:3,:3] # np.eye(3)
    source_R = np.eye(3) # target_T_source[:3,:3]

    print(target_R)
    print(source_R)
    print(target_P)
    print(source_P)

    map_target = np.stack(cv2.initUndistortRectifyMap(target_K, target_dist_coeffs, target_R, target_P, target_Size, cv2.CV_32FC1), axis=-1)
    map_source = np.stack(cv2.initUndistortRectifyMap(source_K, source_dist_coeffs, source_R, source_P, source_Size, cv2.CV_32FC1), axis=-1)
    inv_map_target = invert_map(map_target)
    inv_map_source = invert_map(map_source)

    return map_target, map_source, inv_map_target, inv_map_source

def create_full_map_and_mask(source_map, target_inv_map):
    source_to_target_map = cv2.remap(source_map, target_inv_map, None, interpolation=cv2.INTER_LINEAR )
    source_to_target_mask = cv2.remap(source_map, target_inv_map, None, interpolation=cv2.INTER_LINEAR, borderValue=-1 )
    source_to_target_mask = source_to_target_mask[:,:,0] == -1

    return source_to_target_map, source_to_target_mask

def remap_and_mask(remap_grid, remap_mask, img):
    img_remap = cv2.remap(img, remap_grid[:,:,0], remap_grid[:,:,1], cv2.INTER_LINEAR)
    img_remap[remap_mask] = 0

    if img_remap.ndim == 2:
        img_remap = img_remap[:,:,None]
    return img_remap

def remap_mask(remap_grid, remap_mask, orig_resolution=None):
    if orig_resolution is None:
        orig_resolution = remap_grid[:,:,0].shape
    mask = np.ones( orig_resolution, dtype=np.uint8 ) * 255
    mask_remapped = remap_and_mask(remap_grid, remap_mask, mask)
    mask_remapped = cv2.medianBlur(mask_remapped,3)
    return mask_remapped

def view_remap_topics(remap_grid, remap_mask, image_group, time_lookup, event_group, img_idx=2190):
    img = image_group['data'][img_idx]

    img_remap = remap_and_mask(remap_grid, remap_mask, img)

    start_idx = int(time_lookup[img_idx] - 200000)
    end_idx = int(time_lookup[img_idx] + 0)

    x = event_group['x'][start_idx:end_idx]
    y = event_group['y'][start_idx:end_idx]
    p = event_group['p'][start_idx:end_idx]
    eimg = np.zeros((720,1280,3)).astype(np.uint8)
    eimg[y.astype(int),x.astype(int),0] = 255

    fig, axes = plt.subplots(2,2,sharex=True,sharey=True)

    axes[0,0].imshow(img)
    axes[0,1].imshow(img_remap)
    axes[1,0].imshow(eimg)

    overlay = eimg.copy()
    if img_remap.shape[-1] == 3:
        overlay[:,:,2] = np.linalg.norm(img_remap, axis=-1)
    else:
        overlay[:,:,2] = img_remap.squeeze()
    axes[1,1].imshow(overlay)

    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("--h5fn", help="H5 file to load")
    parser.add_argument("--out_h5fn", help="H5 file to save to")
    parser.add_argument("--target", default="prophesee", help="Camera to warp to")
    parser.add_argument("--img_idx", type=int, default=2000, help="Index to load")
    parser.add_argument('--config', help='Config file')
    parser.add_argument('--checkpoint', help='Checkpoint file')
    parser.add_argument('--out', type=str, default="demo", help='out dir')
    parser.add_argument("--verbose", action="store_true",
                                    help="Verbose output")
    parser.add_argument(
        '--device', default='cuda:0', help='Device used for inference')
    parser.add_argument(
        '--palette',
        default='ade20k',
        choices=['ade20k', 'cityscapes', 'cocostuff'],
        help='Color palette used for segmentation map')
    parser.add_argument(
        '--opacity',
        type=float,
        default=0.5,
        help='Opacity of painted segmentation map. In (0, 1] range.')

    args = parser.parse_args()

    h5f = h5py.File(args.h5fn, 'r')

    target_key = '/' + args.target + '/left'

    ovc_key = '/ovc/rgb'
    target_map, source_map, target_inv_map, source_inv_map = load_remapping( h5f[target_key]['calib'], h5f[ovc_key]['calib'] )

    source_to_target_map, source_to_target_mask = create_full_map_and_mask(source_map, target_inv_map)

    # view_remap_topics(source_to_target_map, source_to_target_mask, h5f[ovc_key], h5f['/ovc/ts_map_prophesee_left_t'], h5f[target_key], img_idx=350)

    model = intern_image_model_load(args)
    h5f_out = h5py.File(args.out_h5fn, 'w')

    total_images = h5f[ovc_key]['data'].shape[0]
    remap_shape = source_to_target_map[:,:,0].shape

    semantics_shape = (total_images, remap_shape[0], remap_shape[1], 1)
    semantics_chunk = (1, remap_shape[0], remap_shape[1], 1)
    if "rgb" in ovc_key:
        warped_shape = (total_images, remap_shape[0], remap_shape[1], 3)
        warped_chunk = (1, remap_shape[0], remap_shape[1], 3)
    else:
        warped_shape = (total_images, remap_shape[0], remap_shape[1], 1)
        warped_chunk = (1, remap_shape[0], remap_shape[1], 1)

    h5f_out.attrs["origin_camera"] = ovc_key
    h5f_out.attrs["destination_camera"] = target_key

    version = subprocess.check_output(["git", "describe", "--tags", "--long"]).decode("utf-8").strip()
    h5f_out.attrs["version"] = version
    h5f_out.attrs["creation_date"] = str(datetime.now())


    h5f_out.create_dataset("ts", data=h5f['/ovc/ts'])
    h5f_out.create_dataset("ts_map_prophesee_left_t", data=h5f['/ovc/ts_map_prophesee_left_t'])

    h5f_out.create_dataset("warped_image", warped_shape, dtype='u1', chunks=warped_chunk, compression='lzf')
    h5f_out.create_dataset("predictions", semantics_shape, dtype='u1', chunks=semantics_chunk, compression='lzf')

    mask = remap_mask(source_to_target_map, source_to_target_mask)
    mask = (255.0 - mask).astype(np.uint8)
    mask = mask > 0

    from tqdm import tqdm
    for idx in tqdm(range(total_images), total=total_images, disable=not args.verbose):
        ovc_image = h5f[ovc_key]['data'][idx]
        ovc_image_remapped = remap_and_mask(source_to_target_map, source_to_target_mask, ovc_image)
        if ovc_image_remapped.shape[-1] == 1:
            input_image = np.tile(ovc_image_remapped, (1,1,3))
        else:
            input_image = ovc_image_remapped

        input_image[mask] = 0

        result = intern_image_model_inference(args, model, input_image)
        result = result[0][:,:,None]
        masked_result = result
        masked_result[mask] = 255

        if "rgb" in ovc_key:
            h5f_out['warped_image'][idx] = ovc_image_remapped
        else:
            h5f_out['warped_image'][idx] = ovc_image_remapped

        h5f_out['predictions'][idx] = masked_result
