"""Convert a RealSense-style ROS2 mcap (RGB + unaligned depth + CameraInfo) into
SplaTAM's NeRFCapture dataset layout (rgb/*.png, depth/*.png, transforms.json).

Usage:
    python scripts/mcap2dataset.py \
        --mcap D:/SplaTAM/test_bag_0.mcap \
        --out  D:/SplaTAM/data/mcap_capture/seq0 \
        --rgb-topic   /camera/camera/color/image_raw \
        --depth-topic /camera/camera/depth/image_rect_raw \
        --rgb-info    /camera/camera/color/camera_info \
        --depth-info  /camera/camera/depth/camera_info

This bag has no /tf_static, so the depth->color extrinsic is unknown. We assume
identity (true within ~15 mm for RealSense D4xx; the cx/cy difference between
the two K matrices is preserved when we reproject depth into the color frame).
"""
import argparse
import json
import os
from pathlib import Path

import cv2
import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

# SplaTAM's NeRFCapture loader hardcodes this at nerfcapture.py:49 - depth_uint16 / 6553.5 = meters.
PNG_DEPTH_SCALE = 6553.5
MAX_DEPTH_M = 65535.0 / PNG_DEPTH_SCALE  # ~10 m
RGB_DEPTH_SYNC_NS = 30_000_000  # 30 ms


def decode_image(msg):
    """sensor_msgs/Image -> numpy array."""
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    h, w = msg.height, msg.width
    enc = msg.encoding
    if enc == "rgb8":
        return buf.reshape(h, w, 3)
    if enc == "bgr8":
        return cv2.cvtColor(buf.reshape(h, w, 3), cv2.COLOR_BGR2RGB)
    if enc == "16UC1" or enc == "mono16":
        return np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
    if enc == "32FC1":
        return np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
    raise ValueError(f"unsupported image encoding: {enc}")


def k_from_camera_info(msg):
    K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
    D = np.array(msg.d, dtype=np.float64).reshape(-1)
    return K, D, int(msg.width), int(msg.height)


def stamp_ns(msg):
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def reproject_depth_into_color(depth_mm, K_d, K_c, out_hw, max_depth_mm=8000):
    """Back-project depth_mm (uint16, mm) using K_d, then project into the color
    image plane with K_c. Identity extrinsic between cameras. Returns uint16 mm.
    Z-min collision policy."""
    h_d, w_d = depth_mm.shape
    h_c, w_c = out_hw
    # RealSense uses 65535 as out-of-range sentinel; also drop unreliable far depths.
    valid = (depth_mm > 0) & (depth_mm < max_depth_mm)
    if not valid.any():
        return np.zeros(out_hw, dtype=np.uint16)
    vs, us = np.nonzero(valid)
    z_m = depth_mm[vs, us].astype(np.float32) / 1000.0  # meters
    fx_d, fy_d, cx_d, cy_d = K_d[0, 0], K_d[1, 1], K_d[0, 2], K_d[1, 2]
    x = (us - cx_d) * z_m / fx_d
    y = (vs - cy_d) * z_m / fy_d
    # identity extrinsic -> same XYZ in color frame
    fx_c, fy_c, cx_c, cy_c = K_c[0, 0], K_c[1, 1], K_c[0, 2], K_c[1, 2]
    u_c = np.round(fx_c * x / z_m + cx_c).astype(np.int32)
    v_c = np.round(fy_c * y / z_m + cy_c).astype(np.int32)
    inside = (u_c >= 0) & (u_c < w_c) & (v_c >= 0) & (v_c < h_c)
    u_c, v_c, z_m = u_c[inside], v_c[inside], z_m[inside]

    out = np.full(out_hw, np.iinfo(np.uint16).max, dtype=np.uint16)
    z_mm = np.clip(z_m * 1000.0, 0, np.iinfo(np.uint16).max).astype(np.uint16)
    # Z-min: pick the nearest point per output pixel. np.minimum.at is unbuffered.
    flat_idx = v_c * w_c + u_c
    flat_out = out.reshape(-1)
    np.minimum.at(flat_out, flat_idx, z_mm)
    # Pixels that were never written stay at uint16 max -> reset to 0 (invalid).
    out[out == np.iinfo(np.uint16).max] = 0
    return out


def mm_to_splatam_uint16(depth_mm):
    """RealSense raw mm -> SplaTAM-scaled uint16 (so loader / 6553.5 gives meters)."""
    depth_m = depth_mm.astype(np.float32) / 1000.0
    depth_m = np.clip(depth_m, 0, MAX_DEPTH_M)
    return (depth_m * PNG_DEPTH_SCALE).astype(np.uint16)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mcap", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--rgb-topic", required=True)
    ap.add_argument("--depth-topic", required=True)
    ap.add_argument("--rgb-info", required=True)
    ap.add_argument("--depth-info", required=True)
    ap.add_argument("--stride", type=int, default=1, help="keep every Nth synced pair")
    ap.add_argument("--max-frames", type=int, default=None)
    args = ap.parse_args()

    out_root = Path(args.out)
    (out_root / "rgb").mkdir(parents=True, exist_ok=True)
    (out_root / "depth").mkdir(parents=True, exist_ok=True)

    # Pass 1: read CameraInfo for both cameras (just need one each).
    K_c = K_d = D_c = D_d = None
    w_c = h_c = w_d = h_d = None
    print("[pass 1] reading CameraInfo...")
    with open(args.mcap, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for schema, channel, _, ros_msg in reader.iter_decoded_messages(
            topics=[args.rgb_info, args.depth_info]
        ):
            if channel.topic == args.rgb_info and K_c is None:
                K_c, D_c, w_c, h_c = k_from_camera_info(ros_msg)
            elif channel.topic == args.depth_info and K_d is None:
                K_d, D_d, w_d, h_d = k_from_camera_info(ros_msg)
            if K_c is not None and K_d is not None:
                break
    assert K_c is not None and K_d is not None, "missing CameraInfo on one stream"
    print(f"  color: {w_c}x{h_c} K=\n{K_c}\n  D={D_c}")
    print(f"  depth: {w_d}x{h_d} K=\n{K_d}\n  D={D_d}")

    # Pre-compute optimal undistorted K and remap maps for color stream.
    new_K_c, _ = cv2.getOptimalNewCameraMatrix(K_c, D_c, (w_c, h_c), alpha=0.0, newImgSize=(w_c, h_c))
    map1, map2 = cv2.initUndistortRectifyMap(K_c, D_c, None, new_K_c, (w_c, h_c), cv2.CV_16SC2)
    print(f"  undistorted color K=\n{new_K_c}")

    # Pass 2: load all RGB + depth into memory (~700 MB for 733 VGA frames - fine
    # for a one-shot converter) so we can sync correctly with np.searchsorted.
    print("[pass 2] loading images...")
    rgb_data = []
    depth_data = []
    with open(args.mcap, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for schema, channel, message, ros_msg in reader.iter_decoded_messages(
            topics=[args.rgb_topic, args.depth_topic]
        ):
            t = stamp_ns(ros_msg)
            img = decode_image(ros_msg)
            if channel.topic == args.rgb_topic:
                rgb_data.append((t, img))
            else:
                depth_data.append((t, img))
    print(f"  loaded {len(rgb_data)} rgb / {len(depth_data)} depth messages")

    rgb_data.sort(key=lambda x: x[0])
    depth_data.sort(key=lambda x: x[0])
    depth_stamps = np.array([d[0] for d in depth_data], dtype=np.int64)

    frames_meta = []
    kept = 0
    print("[pass 3] syncing + saving pairs...")
    for idx, (t_r, rgb) in enumerate(rgb_data):
        if idx % args.stride != 0:
            continue
        j = np.searchsorted(depth_stamps, t_r)
        candidates = [k for k in (j - 1, j) if 0 <= k < len(depth_data)]
        if not candidates:
            continue
        best = min(candidates, key=lambda k: abs(depth_stamps[k] - t_r))
        if abs(int(depth_stamps[best]) - t_r) > RGB_DEPTH_SYNC_NS:
            continue
        depth = depth_data[best][1]

        rgb_rect = cv2.remap(rgb, map1, map2, interpolation=cv2.INTER_LINEAR)
        depth_aligned_mm = reproject_depth_into_color(depth, K_d, new_K_c, (h_c, w_c))
        depth_save = mm_to_splatam_uint16(depth_aligned_mm)
        cv2.imwrite(str(out_root / "rgb" / f"{kept}.png"), cv2.cvtColor(rgb_rect, cv2.COLOR_RGB2BGR))
        cv2.imwrite(str(out_root / "depth" / f"{kept}.png"), depth_save)
        frames_meta.append({
            "file_path": f"rgb/{kept}.png",
            "depth_path": f"depth/{kept}.png",
            "transform_matrix": np.eye(4).tolist(),
        })
        if kept % 50 == 0:
            valid_pct = (depth_aligned_mm > 0).mean() * 100
            print(f"  saved frame {kept} (depth valid {valid_pct:.1f}%)")
        kept += 1
        if args.max_frames is not None and kept >= args.max_frames:
            break

    # Write transforms.json.
    transforms = {
        "fl_x": float(new_K_c[0, 0]),
        "fl_y": float(new_K_c[1, 1]),
        "cx": float(new_K_c[0, 2]),
        "cy": float(new_K_c[1, 2]),
        "w": int(w_c),
        "h": int(h_c),
        "integer_depth_scale": float(PNG_DEPTH_SCALE),
        "frames": frames_meta,
    }
    with open(out_root / "transforms.json", "w") as f:
        json.dump(transforms, f, indent=2)
    print(f"\nWrote {kept} frame pairs to {out_root}")
    print(f"transforms.json intrinsics: fx={transforms['fl_x']:.2f} fy={transforms['fl_y']:.2f} "
          f"cx={transforms['cx']:.2f} cy={transforms['cy']:.2f}  {w_c}x{h_c}")


if __name__ == "__main__":
    main()
