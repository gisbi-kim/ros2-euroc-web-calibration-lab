"""ROS2 image topic to web UI bridge.

This node subscribes to a camera image and CameraInfo topic, serves a browser UI,
stores user-selected frames, and runs calibration-based undistortion on demand.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from datetime import datetime, timezone
import os
from pathlib import Path
import signal
import subprocess
from threading import Lock, Thread
from typing import Any

import cv2
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
import numpy as np
from pydantic import BaseModel, Field
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image
import uvicorn
import yaml

from .calibration import (
    CameraCalibration,
    calibration_from_metadata,
    camera_info_to_metadata,
    create_metadata,
    default_euroc_cam0_calibration,
    load_json,
    normalize_distortion_coeffs,
    save_json,
    undistort_image,
)


MAX_SAVED_IMAGES = 6


@dataclass(frozen=True)
class FrameSnapshot:
    image: np.ndarray
    timestamp_ns: int
    encoding: str
    width: int
    height: int
    topic: str
    calibration: CameraCalibration
    source: str


class BagSeekRequest(BaseModel):
    offset_sec: float = Field(ge=0.0)


def _ros_time_to_ns(sec: int, nanosec: int) -> int:
    return int(sec) * 1_000_000_000 + int(nanosec)


def _image_msg_to_cv2(msg: Image) -> np.ndarray:
    """Convert common ROS2 sensor_msgs/Image encodings to OpenCV arrays."""

    height = int(msg.height)
    width = int(msg.width)
    step = int(msg.step)
    encoding = str(msg.encoding).lower()

    if encoding in {"mono8", "8uc1"}:
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, step)
        return raw[:, :width].copy()

    if encoding in {"bgr8", "rgb8"}:
        channels = 3
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, step)
        img = raw[:, : width * channels].reshape(height, width, channels).copy()
        if encoding == "rgb8":
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img

    if encoding in {"bgra8", "rgba8"}:
        channels = 4
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, step)
        img = raw[:, : width * channels].reshape(height, width, channels).copy()
        if encoding == "rgba8":
            return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

    raise ValueError(f"Unsupported image encoding: {msg.encoding}")


def _encode_jpeg(image: np.ndarray, quality: int) -> bytes:
    params = [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
    ok, encoded = cv2.imencode(".jpg", image, params)
    if not ok:
        raise RuntimeError("Failed to JPEG-encode image")
    return encoded.tobytes()


def _display_image_for_web(image: np.ndarray) -> tuple[np.ndarray, str]:
    if image.ndim == 2:
        return image, "grayscale mono8"
    if image.ndim == 3 and image.shape[2] == 3:
        return image, "native RGB/BGR"
    return image, "native image"


def _safe_image_id_from_path(path: Path) -> str:
    return path.stem


def _read_bag_metadata(bag_path: Path, image_topic: str) -> dict[str, Any]:
    metadata_path = bag_path / "metadata.yaml"
    result: dict[str, Any] = {
        "path": str(bag_path),
        "metadata_path": str(metadata_path),
        "start_time_ns": 0,
        "duration_ns": 0,
        "message_count": 0,
        "image_message_count": 0,
        "topics": [],
        "available": metadata_path.exists(),
    }
    if not metadata_path.exists():
        return result

    data = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
    info = data.get("rosbag2_bagfile_information") or {}
    result["start_time_ns"] = int((info.get("starting_time") or {}).get("nanoseconds_since_epoch") or 0)
    result["duration_ns"] = int((info.get("duration") or {}).get("nanoseconds") or 0)
    result["message_count"] = int(info.get("message_count") or 0)

    topics: list[dict[str, Any]] = []
    for item in info.get("topics_with_message_count") or []:
        topic_metadata = item.get("topic_metadata") or {}
        topic_name = str(topic_metadata.get("name") or "")
        count = int(item.get("message_count") or 0)
        topics.append({"name": topic_name, "type": topic_metadata.get("type"), "message_count": count})
        if topic_name == image_topic:
            result["image_message_count"] = count
    result["topics"] = topics
    return result


class ImageWebBridge(Node):
    """Bridge ROS2 image topics into a web application."""

    def __init__(self) -> None:
        super().__init__("euroc_image_web_bridge")
        self.declare_parameter("image_topic", "/cam0/image_raw")
        self.declare_parameter("camera_info_topic", "/cam0/camera_info")
        self.declare_parameter("save_dir", "/data/saved")
        self.declare_parameter("jpeg_quality", 85)
        self.declare_parameter("stream_fps", 12.0)
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("bag_path", os.environ.get("BAG_PATH", "/external/euroc/MH_01_easy_ros2_ready"))
        self.declare_parameter("bag_rate", float(os.environ.get("BAG_RATE", "0.5")))
        self.declare_parameter("manage_bag_play", False)
        self.declare_parameter("loop_bag", True)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.save_dir = Path(str(self.get_parameter("save_dir").value))
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.stream_fps = float(self.get_parameter("stream_fps").value)
        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)
        self.bag_path = Path(str(self.get_parameter("bag_path").value))
        self.bag_rate = float(self.get_parameter("bag_rate").value)
        self.manage_bag_play = bool(self.get_parameter("manage_bag_play").value)
        self.loop_bag = bool(self.get_parameter("loop_bag").value)

        self.raw_dir = self.save_dir / "raw"
        self.undistorted_dir = self.save_dir / "undistorted"
        self.meta_dir = self.save_dir / "metadata"
        for directory in [self.raw_dir, self.undistorted_dir, self.meta_dir]:
            directory.mkdir(parents=True, exist_ok=True)

        self._lock = Lock()
        self._latest_frame: np.ndarray | None = None
        self._latest_timestamp_ns: int = 0
        self._latest_encoding = "unknown"
        self._latest_width = 0
        self._latest_height = 0
        self._latest_jpeg: bytes | None = None
        self._latest_display_mode = "waiting"
        self._latest_camera_info: dict[str, Any] | None = None
        self._frame_count = 0
        self._capture_count = self._latest_capture_index()
        self._startup_time = datetime.now(timezone.utc)
        self._last_frame_wall_time: datetime | None = None
        self._last_bag_timestamp_ns = 0
        self._bag_loop_count = 0
        self._bag_requested_offset_sec = 0.0
        self._bag_start_wall_time: datetime | None = None
        self._bag_process_lock = Lock()
        self._bag_process: subprocess.Popen[bytes] | None = None
        self._bag_log_handle: Any | None = None
        self._bag_metadata = _read_bag_metadata(self.bag_path, self.image_topic)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, self.image_topic, self._image_callback, qos)
        self.create_subscription(CameraInfo, self.camera_info_topic, self._camera_info_callback, qos)

        self.get_logger().info(f"Subscribing to image topic: {self.image_topic}")
        self.get_logger().info(f"Subscribing to camera info topic: {self.camera_info_topic}")
        self.get_logger().info(f"Saving captures under: {self.save_dir}")
        if self.manage_bag_play:
            self.start_bag_play(0.0)

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        metadata = camera_info_to_metadata(msg)
        with self._lock:
            self._latest_camera_info = metadata

    def _image_callback(self, msg: Image) -> None:
        try:
            image = _image_msg_to_cv2(msg)
            display_image, display_mode = _display_image_for_web(image)
            timestamp_ns = _ros_time_to_ns(msg.header.stamp.sec, msg.header.stamp.nanosec)
            jpeg = _encode_jpeg(display_image, self.jpeg_quality)
        except Exception as exc:  # pragma: no cover - ROS runtime logging path
            self.get_logger().error(f"Failed to process image message: {exc}")
            return

        with self._lock:
            if self._last_bag_timestamp_ns and timestamp_ns + 1_000_000 < self._last_bag_timestamp_ns:
                self._bag_loop_count += 1
            self._last_bag_timestamp_ns = timestamp_ns
            self._latest_frame = image
            self._latest_timestamp_ns = timestamp_ns
            self._latest_encoding = str(msg.encoding)
            self._latest_width = int(msg.width)
            self._latest_height = int(msg.height)
            self._latest_jpeg = jpeg
            self._latest_display_mode = display_mode
            self._frame_count += 1
            self._last_frame_wall_time = datetime.now(timezone.utc)

    def _clamp_bag_offset(self, offset_sec: float) -> float:
        duration_sec = self._bag_metadata.get("duration_ns", 0) / 1_000_000_000
        if duration_sec <= 0:
            return max(float(offset_sec), 0.0)
        return min(max(float(offset_sec), 0.0), max(duration_sec - 0.001, 0.0))

    def _clear_playback_frame_state(self, offset_sec: float) -> None:
        start_ns = int(self._bag_metadata.get("start_time_ns") or 0)
        with self._lock:
            self._latest_frame = None
            self._latest_jpeg = None
            self._latest_display_mode = "waiting"
            self._latest_timestamp_ns = start_ns + int(offset_sec * 1_000_000_000) if start_ns else 0
            self._last_bag_timestamp_ns = 0
            self._last_frame_wall_time = None
            self._frame_count = 0
            self._bag_loop_count = 0
            self._bag_requested_offset_sec = offset_sec

    def _bag_process_running_unlocked(self) -> bool:
        return self._bag_process is not None and self._bag_process.poll() is None

    def _stop_bag_play_unlocked(self) -> None:
        process = self._bag_process
        self._bag_process = None
        if process is not None and process.poll() is None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            except Exception:
                process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except Exception:
                    process.kill()
                process.wait(timeout=3.0)

        if self._bag_log_handle is not None:
            self._bag_log_handle.close()
            self._bag_log_handle = None

    def stop_bag_play(self) -> dict[str, Any]:
        if not self.manage_bag_play:
            raise RuntimeError("This web bridge is not configured to manage ros2 bag play.")
        with self._bag_process_lock:
            self._stop_bag_play_unlocked()
        return self.bag_play_status()

    def start_bag_play(self, offset_sec: float = 0.0) -> dict[str, Any]:
        if not self.manage_bag_play:
            raise RuntimeError("This web bridge is not configured to manage ros2 bag play.")
        if not self.bag_path.exists():
            raise RuntimeError(f"Bag path does not exist: {self.bag_path}")

        offset_sec = self._clamp_bag_offset(offset_sec)
        command = [
            "ros2",
            "bag",
            "play",
            str(self.bag_path),
            "--rate",
            str(self.bag_rate),
            "--disable-keyboard-controls",
        ]
        if self.loop_bag:
            command.append("--loop")
        if offset_sec > 0.0:
            command.extend(["--start-offset", f"{offset_sec:.3f}"])

        with self._bag_process_lock:
            self._stop_bag_play_unlocked()
            self._clear_playback_frame_state(offset_sec)
            log_path = Path("/tmp/ros2_bag_play.log")
            self._bag_log_handle = log_path.open("ab")
            self._bag_log_handle.write(
                f"\n[{datetime.now(timezone.utc).isoformat()}] {' '.join(command)}\n".encode("utf-8")
            )
            self._bag_log_handle.flush()
            self._bag_process = subprocess.Popen(
                command,
                stdout=self._bag_log_handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            self._bag_start_wall_time = datetime.now(timezone.utc)
            self.get_logger().info(
                f"Started ros2 bag play from {self.bag_path} at offset {offset_sec:.3f}s, rate {self.bag_rate}x"
            )
        return self.bag_play_status()

    def bag_play_status(self) -> dict[str, Any]:
        with self._bag_process_lock:
            process = self._bag_process
            running = process is not None and process.poll() is None
            pid = process.pid if process is not None else None
            return_code = None if process is None or running else process.returncode

        with self._lock:
            start_ns = int(self._bag_metadata.get("start_time_ns") or 0)
            duration_ns = int(self._bag_metadata.get("duration_ns") or 0)
            latest_ns = int(self._latest_timestamp_ns)
            if start_ns and latest_ns:
                position_ns = max(0, latest_ns - start_ns)
                if duration_ns > 0:
                    position_ns = min(position_ns, duration_ns)
            else:
                position_ns = int(self._bag_requested_offset_sec * 1_000_000_000)
            duration_sec = duration_ns / 1_000_000_000 if duration_ns else 0.0
            position_sec = position_ns / 1_000_000_000
            progress = min(max(position_sec / duration_sec, 0.0), 1.0) if duration_sec else 0.0
            last_frame_age_sec = None
            if self._last_frame_wall_time is not None:
                last_frame_age_sec = (datetime.now(timezone.utc) - self._last_frame_wall_time).total_seconds()

            return {
                "enabled": self.manage_bag_play,
                "path": str(self.bag_path),
                "metadata_available": bool(self._bag_metadata.get("available")),
                "running": running,
                "pid": pid,
                "return_code": return_code,
                "rate": self.bag_rate,
                "loop_enabled": self.loop_bag,
                "loop_count": int(self._bag_loop_count),
                "start_time_ns": start_ns,
                "duration_ns": duration_ns,
                "duration_sec": duration_sec,
                "position_ns": position_ns,
                "position_sec": position_sec,
                "progress": progress,
                "progress_pct": progress * 100.0,
                "requested_offset_sec": float(self._bag_requested_offset_sec),
                "last_frame_age_sec": last_frame_age_sec,
                "image_message_count": int(self._bag_metadata.get("image_message_count") or 0),
                "message_count": int(self._bag_metadata.get("message_count") or 0),
                "topics": self._bag_metadata.get("topics") or [],
            }

    def _current_calibration_unlocked(self) -> CameraCalibration:
        if self._latest_camera_info is None:
            return default_euroc_cam0_calibration()
        return calibration_from_metadata(self._latest_camera_info)

    def _calibration_source_unlocked(self) -> str:
        if self._latest_camera_info is None:
            return "fallback: EuRoC ASL mav0/cam0/sensor.yaml defaults"
        return f"camera_info topic: {self.camera_info_topic}"

    def _latest_capture_index(self) -> int:
        latest = 0
        for raw_path in self.raw_dir.glob("frame_*.png"):
            try:
                latest = max(latest, int(raw_path.stem.rsplit("_", 1)[1]))
            except (IndexError, ValueError):
                continue
        return latest

    def _saved_raw_paths(self) -> list[Path]:
        return sorted(self.raw_dir.glob("*.png"), key=lambda path: path.stem, reverse=True)

    def _prune_saved_images(self) -> None:
        for raw_path in self._saved_raw_paths()[MAX_SAVED_IMAGES:]:
            image_id = _safe_image_id_from_path(raw_path)
            for path in [
                raw_path,
                self.meta_dir / f"{image_id}.json",
                self.undistorted_dir / f"{image_id}_undistorted.png",
            ]:
                try:
                    path.unlink(missing_ok=True)
                except OSError as exc:
                    self.get_logger().warning(f"Could not remove old saved frame {path}: {exc}")

    def snapshot(self) -> FrameSnapshot | None:
        with self._lock:
            if self._latest_frame is None:
                return None
            return FrameSnapshot(
                image=self._latest_frame.copy(),
                timestamp_ns=int(self._latest_timestamp_ns),
                encoding=self._latest_encoding,
                width=int(self._latest_width),
                height=int(self._latest_height),
                topic=self.image_topic,
                calibration=self._current_calibration_unlocked(),
                source="ros2_bag_or_live_topic",
            )

    def latest_jpeg(self) -> bytes | None:
        with self._lock:
            return None if self._latest_jpeg is None else bytes(self._latest_jpeg)

    def status(self) -> dict[str, Any]:
        with self._lock:
            calib = self._current_calibration_unlocked()
            status = {
                "node": self.get_name(),
                "uptime_sec": (datetime.now(timezone.utc) - self._startup_time).total_seconds(),
                "image_topic": self.image_topic,
                "camera_info_topic": self.camera_info_topic,
                "has_frame": self._latest_frame is not None,
                "frame_count": int(self._frame_count),
                "last_timestamp_ns": int(self._latest_timestamp_ns),
                "encoding": self._latest_encoding,
                "display_mode": self._latest_display_mode,
                "width": int(self._latest_width),
                "height": int(self._latest_height),
                "calibration": calib.as_json_dict(),
                "calibration_source": self._calibration_source_unlocked(),
                "has_camera_info": self._latest_camera_info is not None,
            }
        status["bag"] = self.bag_play_status()
        return status

    def capture_current_frame(self) -> dict[str, Any]:
        snap = self.snapshot()
        if snap is None:
            raise RuntimeError("No image frame has arrived yet")

        self._capture_count += 1
        image_id = f"frame_{self._capture_count:06d}"
        raw_path = self.raw_dir / f"{image_id}.png"
        meta_path = self.meta_dir / f"{image_id}.json"
        if not cv2.imwrite(str(raw_path), snap.image):
            raise RuntimeError(f"Failed to save raw image: {raw_path}")

        metadata = create_metadata(
            image_id=image_id,
            topic=snap.topic,
            timestamp_ns=snap.timestamp_ns,
            encoding=snap.encoding,
            width=snap.width,
            height=snap.height,
            calibration=snap.calibration,
            source=snap.source,
        )
        save_json(meta_path, metadata)
        self._prune_saved_images()
        return {
            "image_id": image_id,
            "raw_url": f"/saved/raw/{image_id}.png",
            "metadata_url": f"/saved/metadata/{image_id}.json",
            "metadata": metadata,
        }

    def undistort_saved_image(self, image_id: str) -> dict[str, Any]:
        raw_path = self.raw_dir / f"{image_id}.png"
        meta_path = self.meta_dir / f"{image_id}.json"
        if not raw_path.exists():
            raise FileNotFoundError(f"Raw image not found: {raw_path}")
        if not meta_path.exists():
            raise FileNotFoundError(f"Metadata not found: {meta_path}")

        raw = cv2.imread(str(raw_path), cv2.IMREAD_UNCHANGED)
        if raw is None:
            raise RuntimeError(f"OpenCV could not read raw image: {raw_path}")

        metadata = load_json(meta_path)
        calib = calibration_from_metadata(metadata)
        undistorted, new_k = undistort_image(raw, calib, alpha=0.0)

        output_path = self.undistorted_dir / f"{image_id}_undistorted.png"
        if not cv2.imwrite(str(output_path), undistorted):
            raise RuntimeError(f"Failed to save undistorted image: {output_path}")

        updated_metadata = dict(metadata)
        updated_metadata["undistorted_url"] = f"/saved/undistorted/{image_id}_undistorted.png"
        updated_metadata["new_K"] = new_k.astype(float).reshape(3, 3).tolist()
        save_json(meta_path, updated_metadata)

        return {
            "image_id": image_id,
            "raw_url": f"/saved/raw/{image_id}.png",
            "undistorted_url": f"/saved/undistorted/{image_id}_undistorted.png",
            "metadata_url": f"/saved/metadata/{image_id}.json",
            "metadata": updated_metadata,
        }

    def saved_images(self) -> list[dict[str, Any]]:
        self._prune_saved_images()
        items: list[dict[str, Any]] = []
        for raw_path in self._saved_raw_paths()[:MAX_SAVED_IMAGES]:
            image_id = _safe_image_id_from_path(raw_path)
            metadata_path = self.meta_dir / f"{image_id}.json"
            undistorted_path = self.undistorted_dir / f"{image_id}_undistorted.png"
            metadata = load_json(metadata_path) if metadata_path.exists() else {}
            items.append(
                {
                    "image_id": image_id,
                    "raw_url": f"/saved/raw/{image_id}.png",
                    "metadata_url": f"/saved/metadata/{image_id}.json",
                    "undistorted_url": f"/saved/undistorted/{image_id}_undistorted.png" if undistorted_path.exists() else None,
                    "timestamp_ros_ns": metadata.get("timestamp_ros_ns"),
                    "width": metadata.get("width"),
                    "height": metadata.get("height"),
                }
            )
        return items


def build_app(node: ImageWebBridge) -> FastAPI:
    app = FastAPI(title="EuRoC ROS2 Web Calibration Lab", version="0.1.0")

    package_dir = Path(__file__).resolve().parent
    static_dir = package_dir / "static"
    app.mount("/static", StaticFiles(directory=str(static_dir)), name="static")
    app.mount("/saved", StaticFiles(directory=str(node.save_dir)), name="saved")

    @app.get("/")
    async def index() -> FileResponse:
        return FileResponse(static_dir / "index.html")

    @app.get("/health")
    async def health() -> dict[str, str]:
        return {"status": "ok"}

    @app.get("/api/status")
    async def status() -> dict[str, Any]:
        return node.status()

    @app.get("/api/bag")
    async def bag_status() -> dict[str, Any]:
        return node.bag_play_status()

    @app.post("/api/bag/restart")
    async def restart_bag() -> JSONResponse:
        try:
            return JSONResponse(node.start_bag_play(0.0))
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.post("/api/bag/seek")
    async def seek_bag(request: BagSeekRequest) -> JSONResponse:
        try:
            return JSONResponse(node.start_bag_play(request.offset_sec))
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.get("/api/saved")
    async def saved() -> dict[str, Any]:
        return {"images": node.saved_images()}

    @app.post("/api/capture")
    async def capture() -> JSONResponse:
        try:
            return JSONResponse(node.capture_current_frame())
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.post("/api/undistort/{image_id}")
    async def undistort(image_id: str) -> JSONResponse:
        try:
            return JSONResponse(node.undistort_saved_image(image_id))
        except FileNotFoundError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        except RuntimeError as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc

    @app.get("/stream.mjpg")
    async def stream() -> StreamingResponse:
        async def frames():
            delay = max(1.0 / max(node.stream_fps, 1.0), 0.02)
            while True:
                jpeg = node.latest_jpeg()
                if jpeg is not None:
                    yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"
                await asyncio.sleep(delay)

        return StreamingResponse(frames(), media_type="multipart/x-mixed-replace; boundary=frame")

    return app


def main() -> None:
    rclpy.init()
    node = ImageWebBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        uvicorn.run(build_app(node), host=node.host, port=node.port, log_level="info")
    finally:
        if node.manage_bag_play:
            node.stop_bag_play()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
