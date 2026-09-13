"""Validate the live rectified camera contract independently of mount calibration."""

import numpy as np

from .zed_pnp_calib import intrinsics_from_camera_info


def camera_matrix(image, info, config, calibration, *, tolerance_px=1e-3):
    matrix, _, size = intrinsics_from_camera_info(info)
    if size != (image.width, image.height):
        raise ValueError("camera image and CameraInfo dimensions differ")
    if not image.header.frame_id or image.header.frame_id != info.header.frame_id:
        raise ValueError("camera image and CameraInfo frames differ")
    if config.get("camera_intrinsics") == "live_rectified":
        # P describes rectified pixels; K is the fallback for open-capture nodes
        # that leave P unset. Neither path reads the reference camera's K.
        projection = np.asarray(getattr(info, "p", [0.0] * 12), dtype=np.float64)
        if projection.shape != (12,) or not np.isfinite(projection).all():
            raise ValueError("camera_info.p must contain twelve finite values")
        if np.any(projection):
            matrix = projection.reshape(3, 4)[:, :3]
            if matrix[0, 0] <= 0 or matrix[1, 1] <= 0 or not np.allclose(matrix[2], [0, 0, 1]):
                raise ValueError("camera_info.p is not a valid rectified projection")
        return matrix
    reference = np.asarray(calibration["camera"]["K"], dtype=np.float64)
    if not np.allclose(matrix, reference, atol=tolerance_px, rtol=0):
        raise ValueError("live intrinsics differ from the calibration")
    return matrix


def require_saved_image_intrinsics(config):
    if config.get("camera_intrinsics") == "live_rectified":
        raise ValueError("external camera requires live Image and CameraInfo; saved-image intrinsics are unavailable")
