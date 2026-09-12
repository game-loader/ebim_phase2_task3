# ZED factory intrinsics

`SN17064700.conf` is the factory calibration exported read-only from the
inspected ZED Mini's SDK settings. It is distinct from the policy's camera
extrinsics in `../zed_pnp_calibration.json`.

The base image includes this file for the original serial. For a replacement
camera, provide its matching `SN<serial>.conf` in `camera.sdk_settings_dir` on
the base host (any data directory; no host SDK installation is required).
Obtain it from the camera's existing SDK cache or Stereolabs before offline
deployment. The container copies only that serial's file into its settings
volume. A different camera's calibration is never substituted silently.
