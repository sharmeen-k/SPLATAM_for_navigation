# SplaTAM Baseline — Replica room0

## Metrics vs Paper Table 1
| Metric      | Our Run  | Paper    |
|-------------|----------|----------|
| ATE RMSE    | 0.28 cm  | 0.36 cm  |
| PSNR        | 32.82 dB | 33.86 dB |
| MS-SSIM     | 0.977    | 0.970    |
| LPIPS       | 0.072    | 0.230    |
| Depth RMSE  | 0.49 cm  | —        |

## Run details
- Hardware: NVIDIA A100-SXM4-40GB
- Tracking: 2.52s/frame, Mapping: 4.10s/frame
- Total: ~2000 frames, seed=0, default paper config
- Outputs: /outputs/Replica/room0_0/
