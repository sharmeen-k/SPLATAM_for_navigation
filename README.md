# SPLATAM_for_navigation

Testing [**SplaTAM**](https://github.com/spla-tam/SplaTAM) (*Splat, Track & Map 3D Gaussians for Dense RGB-D SLAM*, Keetha et al., CVPR 2024) on custom Intel RealSense footage captured for indoor robot navigation.

This repository is the experiment harness for an EE243 course project. It does **not** re-implement SplaTAM — it clones the upstream repo at runtime and drives it. The contribution here is the tooling and configuration needed to:

1. Reproduce the SplaTAM **Replica** baseline (`room0`, `office0`) to validate the setup.
2. Convert RealSense ROS 2 **`.mcap`** recordings into the **NeRFCapture** dataset layout SplaTAM expects.
3. Run full RGB-D SLAM (tracking + mapping, **no ground-truth poses**) on four custom sequences and evaluate rendering / depth quality.

The reference paper is included as [`splatam_paper.pdf`](splatam_paper.pdf).

## Repository layout

| Path | Description |
| --- | --- |
| `main_SPLATAM_testing.ipynb` | Main Google Colab notebook. Installs SplaTAM and the CUDA `diff-gaussian-rasterization` rasterizer, runs the Replica baselines, builds the custom datasets, and launches SplaTAM on each sequence. |
| `mcap2nerfcapturedataset_results/mcap2dataset.py` | Converter: RealSense ROS 2 `.mcap` (RGB + depth + `CameraInfo`) → SplaTAM NeRFCapture dataset (`rgb/*.png`, `depth/*.png`, `transforms.json`). |
| `mcap2nerfcapturedataset_results/Diagnostics.ipynb` | Diagnostic notebook: raw bag extraction, Gaussian-count growth tracking, depth-scale / sparsity sanity checks, RGB-D visualization. |
| `turtlebot_seq1.py` … `turtlebot_seq4.py` | SplaTAM config files (one per custom sequence). Identical apart from `run_name` / `sequence`. |
| `outputs/` | Saved results: per-frame eval metrics (`psnr/ssim/lpips/rmse/l1.txt`, `metrics.png`), run logs, optimized Gaussian params (`params.npz`), and the frozen `config.py` for each run. |
| `splatam_paper.pdf` | The SplaTAM paper, for reference. |

> Note: per `.gitignore`, the actual datasets (`data/`), point clouds (`*.ply`), checkpoints (`*.pt`/`*.pth`), and zip archives are **not** committed — only configs, code, logs, and evaluation metrics.

## The data pipeline

The custom footage was recorded on an Intel RealSense D4xx camera (640×480 color + depth) and saved as ROS 2 `.mcap` bags. `mcap2dataset.py` turns a bag into a SplaTAM-ready dataset:

1. **Read intrinsics** for both the color and depth streams from their `CameraInfo` topics.
2. **Undistort** the color image using `cv2.getOptimalNewCameraMatrix` / `initUndistortRectifyMap`.
3. **Align depth to color** — back-project the depth map with `K_depth`, then reproject into the color frame with `K_color` (identity extrinsic; valid to ~15 mm for RealSense D4xx). A Z-min collision policy keeps the nearest point per output pixel.
4. **Rescale depth** to the fixed scale SplaTAM's NeRFCapture loader hardcodes — `depth_uint16 / 6553.5 = meters` (so the usable range is ~10 m).
5. **Time-sync** RGB and depth by nearest timestamp within a 30 ms window.
6. **Write** `rgb/{i}.png`, `depth/{i}.png`, and a `transforms.json` carrying the undistorted intrinsics and per-frame `transform_matrix` (identity — SplaTAM estimates the poses itself).

### Example

```bash
python mcap2nerfcapturedataset_results/mcap2dataset.py \
    --mcap   path/to/recording_0.mcap \
    --out    path/to/dataset/seq1 \
    --rgb-topic   /camera/camera/color/image_raw \
    --depth-topic /camera/camera/aligned_depth_to_color/image_raw \
    --rgb-info    /camera/camera/color/camera_info \
    --depth-info  /camera/camera/color/camera_info
```

Requires `mcap`, `mcap-ros2-support`, `opencv-python`, and `numpy`. Optional flags: `--stride N` (keep every Nth pair), `--max-frames N`.

## Custom sequences

| Sequence | Footage | Frames |
| --- | --- | --- |
| `seq1` | Handheld, room | 873 |
| `seq2` | Handheld, desk | 876 |
| `seq3` | Robot-mounted, lab | 963 |
| `seq4` | Robot-mounted, desk | 958 |

Each is run with the matching `turtlebot_seqN.py` config: full SLAM (`use_gt_poses=False`), 40 tracking iterations and 60 mapping iterations per keyframe, `mapping_window_size=24`, isotropic Gaussians, silhouette-based tracking loss. Because the runs are long and Colab runtimes are volatile, the notebook supports **resuming from a saved checkpoint** (`load_checkpoint` / `checkpoint_time_idx`).

## Results

### Custom sequences (full SLAM, estimated poses)

| Sequence | PSNR ↑ | MS-SSIM ↑ | LPIPS ↓ | Depth RMSE ↓ | ATE RMSE ↓ |
| --- | --- | --- | --- | --- | --- |
| seq1 (room, handheld)  | 20.43 | 0.812 | 0.220 | 15.48 cm | 209.74 cm |
| seq2 (desk, handheld)  | 26.18 | 0.906 | 0.109 |  4.80 cm | 121.26 cm |
| seq3 (lab, robot)      | 15.80 | 0.731 | 0.295 |  5.07 cm |  85.52 cm |
| seq4 (desk, robot)     | 23.52 | 0.897 | 0.135 | 10.50 cm |  26.50 cm |

Rendering quality (PSNR / SSIM / LPIPS) is solid on the desk and robot sequences, but trajectory error (ATE) is high on the handheld sequences — fast hand motion and sparse / noisy RealSense depth make camera tracking hard without ground-truth poses.

### Replica baseline (sanity check)

| Scene | PSNR | MS-SSIM | LPIPS | Depth RMSE | ATE RMSE |
| --- | --- | --- | --- | --- | --- |
| `room0`   | 32.82 | 0.977 | 0.072 | 0.49 cm | — |
| `office0` | 38.71 | 0.984 | 0.082 | 0.37 cm | 0.46 cm |

For `room0` the reproduction matches the paper closely (paper PSNR 33.18 / ours 32.82, paper SSIM 0.972 / ours 0.977, paper LPIPS 0.074 / ours 0.072), confirming the harness is faithful before moving to custom data.

## Reproducing

1. Open `main_SPLATAM_testing.ipynb` in Google Colab with a GPU runtime (developed on a Tesla T4 / PyTorch + CUDA 12.8).
2. Run the setup cell — it clones SplaTAM, installs dependencies (`tqdm`, `kornia`, `lpips`, `open3d`, `pytorch-msssim`, `diff-gaussian-rasterization-w-depth`, …), and patches a couple of upstream config / `__init__.py` issues.
3. **Replica baseline:** upload `Replica.zip` to your Drive, unzip into the runtime, and run the `room0` / `office0` cells.
4. **Custom data:** point the `mcap2dataset.py` cells at your `.mcap` bags to build `seq1`–`seq4`, then launch SplaTAM with the `turtlebot_seqN.py` configs.

Paths in the notebook and configs assume a Google Drive working directory (`/content/drive/MyDrive/EE243 Project/...`); adjust `workdir`, `basedir`, and the dataset paths to your own setup.

## Acknowledgements

Built on top of [SplaTAM](https://github.com/spla-tam/SplaTAM) by Keetha et al. The Gaussian rasterizer is [diff-gaussian-rasterization-w-depth](https://github.com/JonathonLuiten/diff-gaussian-rasterization-w-depth). Replica scenes follow the dataset conventions used by the SplaTAM authors.
