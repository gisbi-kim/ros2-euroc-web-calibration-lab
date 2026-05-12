"""Camera calibration helpers for the EuRoC web lab.

The functions in this module are deliberately side-effect free, except for the
explicit load/save helpers. They do not mutate images or metadata dictionaries
passed by the caller.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any, Iterable, Mapping

import cv2
import numpy as np


DEFAULT_WIDTH = 752
DEFAULT_HEIGHT = 480

# EuRoC ASL mav0/cam0/sensor.yaml values for the VI-Sensor cam0 (MT9M034).
# The original model is radial-tangential with [k1, k2, p1, p2].
# OpenCV plumb_bob accepts [k1, k2, p1, p2, k3]; we use k3=0.
EUROC_CAM0_K = np.array(
    [
        [458.654, 0.0, 367.215],
        [0.0, 457.296, 248.375],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)
EUROC_CAM0_D = np.array([-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05, 0.0], dtype=np.float64)


@dataclass(frozen=True)
class CameraCalibration:
    """Immutable camera calibration bundle."""

    width: int
    height: int
    k: np.ndarray
    d: np.ndarray
    distortion_model: str = "plumb_bob"

    def as_json_dict(self) -> dict[str, Any]:
        return {
            "width": int(self.width),
            "height": int(self.height),
            "K": self.k.astype(float).reshape(3, 3).tolist(),
            "D": self.d.astype(float).reshape(-1).tolist(),
            "distortion_model": self.distortion_model,
        }


def default_euroc_cam0_calibration() -> CameraCalibration:
    """Return a fresh immutable EuRoC cam0 calibration object."""

    return CameraCalibration(
        width=DEFAULT_WIDTH,
        height=DEFAULT_HEIGHT,
        k=EUROC_CAM0_K.copy(),
        d=EUROC_CAM0_D.copy(),
        distortion_model="plumb_bob",
    )


def _coerce_float_array(values: Iterable[float], *, expected: int | None = None) -> np.ndarray:
    arr = np.asarray(list(values), dtype=np.float64).reshape(-1)
    if expected is not None and arr.size != expected:
        raise ValueError(f"Expected {expected} values, got {arr.size}")
    return arr


def normalize_distortion_coeffs(values: Iterable[float]) -> np.ndarray:
    """Return OpenCV-compatible distortion coefficients.

    EuRoC radial-tangential calibration commonly has four values
    [k1, k2, p1, p2]. OpenCV's plumb_bob model often uses five values
    [k1, k2, p1, p2, k3]. Missing k3 is set to zero.
    """

    d = _coerce_float_array(values)
    if d.size == 4:
        return np.concatenate([d, np.zeros(1, dtype=np.float64)])
    if d.size >= 5:
        return d[:5].copy()
    if d.size == 0:
        return np.zeros(5, dtype=np.float64)
    raise ValueError("Distortion coefficient vector must have length 0, 4, or at least 5")


def calibration_from_metadata(metadata: Mapping[str, Any]) -> CameraCalibration:
    """Create a calibration object from saved image metadata."""

    k_raw = metadata.get("K") or metadata.get("camera_matrix")
    d_raw = metadata.get("D") or metadata.get("distortion_coefficients")
    if k_raw is None:
        k = EUROC_CAM0_K.copy()
    else:
        k = _coerce_float_array(np.asarray(k_raw, dtype=np.float64).reshape(-1), expected=9).reshape(3, 3)
    if d_raw is None:
        d = EUROC_CAM0_D.copy()
    else:
        d = normalize_distortion_coeffs(np.asarray(d_raw, dtype=np.float64).reshape(-1))

    width = int(metadata.get("width", DEFAULT_WIDTH))
    height = int(metadata.get("height", DEFAULT_HEIGHT))
    model = str(metadata.get("distortion_model", "plumb_bob"))
    return CameraCalibration(width=width, height=height, k=k, d=d, distortion_model=model)


def camera_info_to_metadata(msg: Any) -> dict[str, Any]:
    """Convert a ROS2 sensor_msgs/CameraInfo message to JSON metadata."""

    k = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
    d = normalize_distortion_coeffs(msg.d)
    return {
        "width": int(msg.width),
        "height": int(msg.height),
        "K": k.tolist(),
        "D": d.tolist(),
        "R": np.asarray(msg.r, dtype=np.float64).reshape(3, 3).tolist(),
        "P": np.asarray(msg.p, dtype=np.float64).reshape(3, 4).tolist(),
        "distortion_model": str(msg.distortion_model or "plumb_bob"),
    }


def undistort_image(
    image: np.ndarray,
    calibration: CameraCalibration,
    *,
    alpha: float = 0.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Undistort an image using OpenCV remapping.

    Returns:
        A tuple ``(undistorted_image, new_camera_matrix)``.
    """

    if image.ndim not in (2, 3):
        raise ValueError("image must be a grayscale or color image")

    height, width = image.shape[:2]
    image_size = (int(width), int(height))
    new_k, _roi = cv2.getOptimalNewCameraMatrix(calibration.k, calibration.d, image_size, alpha, image_size)
    map_x, map_y = cv2.initUndistortRectifyMap(
        calibration.k,
        calibration.d,
        None,
        new_k,
        image_size,
        cv2.CV_32FC1,
    )
    result = cv2.remap(image.copy(), map_x, map_y, interpolation=cv2.INTER_LINEAR)
    return result, new_k


def load_json(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError(f"Expected object in JSON file: {path}")
    return data


def save_json(path: str | Path, value: Mapping[str, Any]) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("w", encoding="utf-8") as f:
        json.dump(dict(value), f, indent=2, sort_keys=True)


def create_metadata(
    *,
    image_id: str,
    topic: str,
    timestamp_ns: int,
    encoding: str,
    width: int,
    height: int,
    calibration: CameraCalibration,
    source: str,
) -> dict[str, Any]:
    """Create metadata for a captured frame."""

    base = calibration.as_json_dict()
    base.update(
        {
            "image_id": image_id,
            "topic": topic,
            "timestamp_ros_ns": int(timestamp_ns),
            "encoding": encoding,
            "width": int(width),
            "height": int(height),
            "camera": "cam0",
            "source": source,
        }
    )
    return base
