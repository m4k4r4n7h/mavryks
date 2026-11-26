from __future__ import annotations

import argparse
import json
import logging
import math
import os
import queue
import signal
import threading
import time
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np
import requests
from ultralytics import YOLO

from pose import MarkerLocalizer
from proximity import ProximityMonitor
from quality import FrameQuality

logging.basicConfig(
    level=os.getenv("LOG_LEVEL", "INFO").upper(),
    format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
)
logger = logging.getLogger("mavryks.tracker")

LABELS = ["Top-Left", "Top-Right", "Bottom-Right", "Bottom-Left"]
WINDOW = "MAVRYKS Live"


@dataclass
class TrackerConfig:
    source: str = os.getenv("CAMERA_SOURCE", "0")
    backend_url: str = os.getenv("BACKEND_URL", "http://127.0.0.1:8000").rstrip("/")
    model_path: str = os.getenv("MODEL_PATH", "yolov8n.pt")
    device: str | None = os.getenv("YOLO_DEVICE") or None
    grid_size: int = int(os.getenv("GRID_SIZE", "12"))
    frame_w: int = 640
    frame_h: int = 480
    conf: float = 0.30
    diff_thresh: int = 20
    min_contour_area: float = 90.0
    occ_alpha: float = 0.45
    enter_thresh: float = 0.55
    exit_thresh: float = 0.25
    adaptive_bg: bool = True
    bg_alpha: float = 0.004
    sync_timeout: float = 0.6
    headless: bool = False
    cal_dir: str = "."
    marker_dictionary: int = cv2.aruco.DICT_4X4_50
    min_marker_area: float = 36.0
    cell_size_m: float = 1.0
    proximity_warning_cells: float = 2.5
    proximity_critical_cells: float = 1.2


def project_points(matrix: np.ndarray, pts: np.ndarray | list[Any]) -> np.ndarray:
    a = np.asarray(pts, dtype=np.float64).reshape(-1, 2)
    if len(a) == 0:
        return np.empty((0, 2), dtype=np.float64)
    homog = np.column_stack((a, np.ones(len(a), dtype=np.float64)))
    q = homog @ matrix.T
    z = q[:, 2:3]
    z = np.where(np.abs(z) < 1e-6, np.where(z < 0, -1e-6, 1e-6), z)
    out = q[:, :2] / z
    return np.nan_to_num(out, nan=0.0, posinf=1e5, neginf=-1e5)


def safe_int_pts(pts: np.ndarray, limit: int = 10000) -> np.ndarray:
    return np.clip(np.round(pts), -limit, limit).astype(np.int32)


class CalibrationManager:
    def __init__(self, cfg: TrackerConfig) -> None:
        self.cfg = cfg
        self.h_path = os.path.join(cfg.cal_dir, "H.npy")
        self.inv_path = os.path.join(cfg.cal_dir, "inv_H.npy")
        self.ref_path = os.path.join(cfg.cal_dir, "floor_ref.npy")
        self.meta_path = os.path.join(cfg.cal_dir, "calibration_meta.json")
        self.pts: list[tuple[float, float]] = []
        self.H: np.ndarray | None = None
        self.inv_H: np.ndarray | None = None
        self.ref_gray: np.ndarray | None = None
        self.floor_mask: np.ndarray | None = None
        self.locked = False
        self.need_ref = False
        self.status_msg = ""

    def floor_corners(self) -> np.ndarray:
        n = self.cfg.grid_size
        return np.array([[0, 0], [n - 1, 0], [n - 1, n - 1], [0, n - 1]], dtype=np.float64)

    def cell_corners(self, x: int, y: int) -> np.ndarray:
        return np.array(
            [[x - 0.5, y - 0.5], [x + 0.5, y - 0.5], [x + 0.5, y + 0.5], [x - 0.5, y + 0.5]],
            dtype=np.float64,
        )

    def _build_floor_mask(self) -> None:
        if self.inv_H is None:
            self.floor_mask = None
            return
        n = self.cfg.grid_size
        outer = np.array(
            [[-0.5, -0.5], [n - 0.5, -0.5], [n - 0.5, n - 0.5], [-0.5, n - 0.5]],
            dtype=np.float64,
        )
        poly = safe_int_pts(project_points(self.inv_H, outer))
        mask = np.zeros((self.cfg.frame_h, self.cfg.frame_w), dtype=np.uint8)
        cv2.fillPoly(mask, [poly], 255)
        self.floor_mask = mask

    def _frame_to_ref_gray(self, frame: np.ndarray) -> np.ndarray:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return cv2.GaussianBlur(gray, (5, 5), 0).astype(np.float32)

    def save_calibration(self, frame: np.ndarray) -> bool:
        if len(self.pts) != 4:
            return False
        src = np.asarray(self.pts, dtype=np.float32)
        area = cv2.contourArea(src)
        if area < 400.0 or not cv2.isContourConvex(src):
            self.status_msg = "INVALID QUAD: Click 4 convex corners clockwise"
            logger.warning("Rejected degenerate calibration quad (area=%.1f)", area)
            return False
        dst = self.floor_corners().astype(np.float32)
        h_mat = cv2.getPerspectiveTransform(src, dst)
        if abs(float(np.linalg.det(h_mat))) < 1e-9:
            self.status_msg = "SINGULAR MATRIX: Re-pick corners"
            return False
        try:
            inv_mat = np.linalg.inv(h_mat)
        except np.linalg.LinAlgError:
            self.status_msg = "MATRIX INVERSION FAILED: Re-pick corners"
            return False
        self.H = h_mat
        self.inv_H = inv_mat
        self.ref_gray = self._frame_to_ref_gray(frame)
        self._build_floor_mask()
        np.save(self.h_path, self.H)
        np.save(self.inv_path, self.inv_H)
        np.save(self.ref_path, frame)
        with open(self.meta_path, "w", encoding="utf-8") as fh:
            json.dump(
                {
                    "grid_size": self.cfg.grid_size,
                    "frame_w": self.cfg.frame_w,
                    "frame_h": self.cfg.frame_h,
                    "saved_at": time.time(),
                },
                fh,
            )
        self.locked = True
        self.need_ref = False
        self.status_msg = "CALIBRATION LOCKED"
        logger.info("Saved perspective calibration to %s", self.cfg.cal_dir)
        return True

    def capture_reference(self, frame: np.ndarray) -> None:
        self.ref_gray = self._frame_to_ref_gray(frame)
        np.save(self.ref_path, frame)
        self.need_ref = False
        self.status_msg = "FLOOR REFERENCE UPDATED"
        logger.info("Recaptured empty floor reference")

    def load(self) -> None:
        if not (os.path.exists(self.h_path) and os.path.exists(self.inv_path)):
            return
        try:
            h_mat = np.load(self.h_path)
            inv_mat = np.load(self.inv_path)
            if h_mat.shape != (3, 3) or inv_mat.shape != (3, 3):
                return
            self.H = h_mat
            self.inv_H = inv_mat
            self.locked = True
            self._build_floor_mask()
            if os.path.exists(self.ref_path):
                ref_img = np.load(self.ref_path)
                if ref_img.ndim == 3:
                    if ref_img.shape[:2] != (self.cfg.frame_h, self.cfg.frame_w):
                        ref_img = cv2.resize(ref_img, (self.cfg.frame_w, self.cfg.frame_h))
                    self.ref_gray = self._frame_to_ref_gray(ref_img)
                    self.need_ref = False
                else:
                    self.need_ref = True
            else:
                self.need_ref = True
            logger.info("Loaded saved calibration (need_ref=%s)", self.need_ref)
        except Exception as exc:
            logger.warning("Failed to load calibration files: %s", exc)
            self.clear(remove_files=False)

    def clear(self, remove_files: bool = True) -> None:
        self.pts = []
        self.H = None
        self.inv_H = None
        self.ref_gray = None
        self.floor_mask = None
        self.locked = False
        self.need_ref = False
        self.status_msg = "CALIBRATION CLEARED"
        if remove_files:
            for fp in (self.h_path, self.inv_path, self.ref_path, self.meta_path):
                if os.path.exists(fp):
                    try:
                        os.remove(fp)
                    except OSError:
                        pass

    def cell_from_grid_pt(self, p: np.ndarray) -> tuple[int, int] | None:
        x = int(round(float(p[0])))
        y = int(round(float(p[1])))
        if 0 <= x < self.cfg.grid_size and 0 <= y < self.cfg.grid_size:
            return (x, y)
        return None


class WorkerTracker:
    def __init__(self, max_dist: float = 2.5, alpha: float = 0.55, max_missed: int = 8) -> None:
        self.max_dist = max_dist
        self.alpha = alpha
        self.max_missed = max_missed
        self._next_id = 1
        self._tracks: dict[int, dict[str, Any]] = {}

    def update(self, detections: list[tuple[float, float]], grid_size: int) -> list[tuple[int, int, int]]:
        unmatched_tracks = set(self._tracks.keys())
        unmatched_dets = list(range(len(detections)))
        pairs: list[tuple[float, int, int]] = []
        for tid, tr in self._tracks.items():
            tx, ty = tr["pos"]
            for di, (dx, dy) in enumerate(detections):
                dist = math.hypot(dx - tx, dy - ty)
                if dist <= self.max_dist:
                    pairs.append((dist, tid, di))
        pairs.sort(key=lambda x: x[0])
        matched_dets: set[int] = set()
        for _, tid, di in pairs:
            if tid not in unmatched_tracks or di in matched_dets:
                continue
            dx, dy = detections[di]
            tx, ty = self._tracks[tid]["pos"]
            self._tracks[tid]["pos"] = (
                self.alpha * dx + (1.0 - self.alpha) * tx,
                self.alpha * dy + (1.0 - self.alpha) * ty,
            )
            self._tracks[tid]["missed"] = 0
            unmatched_tracks.remove(tid)
            matched_dets.add(di)
        for tid in list(unmatched_tracks):
            self._tracks[tid]["missed"] += 1
            if self._tracks[tid]["missed"] > self.max_missed:
                del self._tracks[tid]
        for di in unmatched_dets:
            if di not in matched_dets:
                self._tracks[self._next_id] = {"pos": detections[di], "missed": 0}
                self._next_id += 1
        out: list[tuple[int, int, int]] = []
        for tid in sorted(self._tracks.keys()):
            px, py = self._tracks[tid]["pos"]
            cx = max(0, min(grid_size - 1, int(round(px))))
            cy = max(0, min(grid_size - 1, int(round(py))))
            out.append((tid, cx, cy))
        return out


class OccupancyFilter:
    def __init__(self, grid_size: int, alpha: float, enter: float, exit_t: float) -> None:
        self.n = grid_size
        self.alpha = alpha
        self.enter = enter
        self.exit_t = exit_t
        self.ema = np.zeros((grid_size, grid_size), dtype=np.float32)
        self.state = np.zeros((grid_size, grid_size), dtype=np.bool_)

    def update(self, raw_mask: np.ndarray) -> tuple[set[tuple[int, int]], np.ndarray]:
        obs_float = (raw_mask > 0).astype(np.float32)
        self.ema = self.alpha * obs_float + (1.0 - self.alpha) * self.ema
        self.state = np.where(
            self.state,
            self.ema >= self.exit_t,
            self.ema >= self.enter,
        )
        ys, xs = np.nonzero(self.state)
        obstacles = {(int(x), int(y)) for x, y in zip(xs, ys)}
        a = self.state.astype(np.float32)
        rho = cv2.GaussianBlur(a, (5, 5), 1.0)
        rho = np.clip(rho * 1.6, 0.0, 1.0)
        rho[self.state] = np.maximum(rho[self.state], 0.95)
        return obstacles, rho

    def reset(self) -> None:
        self.ema.fill(0.0)
        self.state.fill(False)


class BackendSyncWorker:
    def __init__(self, backend_url: str, timeout: float = 0.6) -> None:
        self.backend_url = backend_url
        self.timeout = timeout
        self._lock = threading.Lock()
        self._event = threading.Event()
        self._stop = threading.Event()
        self._latest_payload: dict[str, Any] | None = None
        self._command_queue: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=32)
        self.path: list[list[int]] = []
        self.fleet_paths: list[list[list[int]]] = []
        self.targets: list[list[int]] = []
        self.static_obstacles: list[list[int]] = []
        self.tasks: list[dict[str, Any]] = []
        self.task_stats: dict[str, int] = {}
        self.commands: list[dict[str, Any]] = []
        self.safety: dict[str, Any] = {}
        self.eta_sec: float = 0.0
        self.distance: float = 0.0
        self.online: bool = False
        self.latency_ms: float = 0.0
        self.estop: bool = False
        self._session = requests.Session()
        self._thread = threading.Thread(target=self._run, name="BackendSync", daemon=True)
        self._thread.start()

    def publish_vision(self, payload: dict[str, Any]) -> None:
        with self._lock:
            self._latest_payload = payload
        self._event.set()

    def send_command(self, method: str, path: str, payload: dict[str, Any] | None = None) -> None:
        try:
            self._command_queue.put_nowait({"method": method, "path": path, "payload": payload or {}})
            self._event.set()
        except queue.Full:
            pass

    def send_target(self, x: int, y: int, append: bool = False) -> None:
        self.send_command("POST", "/api/target", {"x": x, "y": y, "append": append})

    def send_task(self, x: int, y: int, kind: str = "pick") -> None:
        self.send_command("POST", "/api/tasks", {"x": x, "y": y, "kind": kind, "priority": 0})

    def complete_task(self, task_id: str) -> None:
        self.send_command("POST", f"/api/tasks/{task_id}/complete", {})

    def send_estop(self, active: bool) -> None:
        self.send_command("POST", "/api/estop", {"active": active})

    def get_snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "path": list(self.path),
                "fleet_paths": [list(p) for p in self.fleet_paths],
                "targets": [list(t) for t in self.targets],
                "static_obstacles": [list(o) for o in self.static_obstacles],
                "tasks": [dict(t) for t in self.tasks],
                "task_stats": dict(self.task_stats),
                "commands": [dict(c) for c in self.commands],
                "safety": dict(self.safety),
                "eta_sec": self.eta_sec,
                "distance": self.distance,
                "online": self.online,
                "latency_ms": self.latency_ms,
                "estop": self.estop,
            }

    def _apply_response(self, data: dict[str, Any], latency_ms: float) -> None:
        with self._lock:
            self.path = [[int(p[0]), int(p[1])] for p in data.get("path", []) if len(p) >= 2]
            self.fleet_paths = [
                [[int(p[0]), int(p[1])] for p in r.get("path", []) if len(p) >= 2]
                for r in data.get("fleet_routes", [])
            ]
            self.targets = [[int(t[0]), int(t[1])] for t in data.get("targets", []) if len(t) >= 2]
            self.static_obstacles = [
                [int(o[0]), int(o[1])] for o in data.get("static_obstacles", []) if len(o) >= 2
            ]
            self.tasks = [dict(t) for t in data.get("tasks", []) if isinstance(t, dict)]
            self.task_stats = dict(data.get("task_stats", {}))
            self.commands = [dict(c) for c in data.get("commands", []) if isinstance(c, dict)]
            self.safety = dict(data.get("safety", {}))
            self.eta_sec = float(data.get("estimated_time_sec", 0.0))
            self.distance = float(data.get("distance", 0.0))
            self.estop = bool(data.get("estop", False))
            self.online = True
            self.latency_ms = latency_ms

    def _run(self) -> None:
        backoff = 0.2
        while not self._stop.is_set():
            self._event.wait(timeout=0.5)
            self._event.clear()
            while not self._command_queue.empty():
                cmd = self._command_queue.get_nowait()
                try:
                    t0 = time.perf_counter()
                    r = self._session.request(
                        cmd["method"],
                        f"{self.backend_url}{cmd['path']}",
                        json=cmd["payload"],
                        timeout=self.timeout,
                    )
                    should_apply = bool(r.ok) and (
                        cmd["path"] in ("/api/target", "/api/estop")
                        or cmd["path"].startswith("/api/tasks")
                    )
                    if should_apply:
                        self._apply_response(r.json().get("state", r.json()), (time.perf_counter() - t0) * 1000.0)
                except requests.RequestException:
                    with self._lock:
                        self.online = False
            with self._lock:
                payload = self._latest_payload
                self._latest_payload = None
            if payload is None:
                continue
            try:
                t0 = time.perf_counter()
                r = self._session.post(
                    f"{self.backend_url}/api/vision/update",
                    json=payload,
                    timeout=self.timeout,
                )
                if r.ok:
                    self._apply_response(r.json(), (time.perf_counter() - t0) * 1000.0)
                    backoff = 0.2
                else:
                    with self._lock:
                        self.online = False
            except requests.RequestException:
                with self._lock:
                    self.online = False
                time.sleep(backoff)
                backoff = min(2.0, backoff * 1.5)

    def close(self) -> None:
        self._stop.set()
        self._event.set()
        self._thread.join(timeout=1.5)
        self._session.close()


class MavryksTracker:
    def __init__(self, cfg: TrackerConfig) -> None:
        self.cfg = cfg
        self.cal = CalibrationManager(cfg)
        self.cal.load()
        self.occ_filter = OccupancyFilter(
            cfg.grid_size, cfg.occ_alpha, cfg.enter_thresh, cfg.exit_thresh
        )
        self.worker_tracker = WorkerTracker()
        self.sync = BackendSyncWorker(cfg.backend_url, cfg.sync_timeout)
        self.marker_localizer = MarkerLocalizer(cfg.marker_dictionary, cfg.min_marker_area)
        self.quality_monitor = FrameQuality()
        self.proximity_monitor = ProximityMonitor(
            cfg.proximity_warning_cells, cfg.proximity_critical_cells
        )
        logger.info("Loading YOLO model: %s", cfg.model_path)
        self.model = YOLO(cfg.model_path)
        self.morph_kernel = np.ones((3, 3), dtype=np.uint8)
        self.manual_start: list[int] | None = None
        self.paused = False
        self.show_debug = False
        self.running = True
        self.estop = False
        self.fps = 0.0
        self.last_boxes: list[tuple[int, int, int, int, str, float]] = []
        self.last_person_points: list[tuple[float, float]] = []
        self.last_markers: list[dict[str, Any]] = []
        self.last_quality: dict[str, Any] = {}

    def _open_capture(self) -> cv2.VideoCapture:
        try:
            index = int(str(self.cfg.source).strip())
        except (TypeError, ValueError):
            index = 0
        cap = cv2.VideoCapture(max(0, index))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cfg.frame_w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cfg.frame_h)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def _window_visible(self) -> bool:
        if self.cfg.headless:
            return False
        try:
            return cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) >= 1.0
        except cv2.error:
            return False

    def _rasterize_poly(self, poly_grid: np.ndarray, canvas: np.ndarray) -> None:
        if len(poly_grid) < 3:
            return
        clamped = np.clip(poly_grid, -50.0, self.cfg.grid_size + 50.0)
        pts_fixed = np.round(clamped * 16.0).astype(np.int32).reshape(-1, 1, 2)
        cv2.fillPoly(canvas, [pts_fixed], 1, shift=4)

    def detect(
        self, frame: np.ndarray
    ) -> tuple[
        list[tuple[int, int, int]],
        set[tuple[int, int]],
        np.ndarray,
        list[dict[str, Any]],
        dict[str, Any],
    ]:
        n = self.cfg.grid_size
        if self.cal.H is None or self.cal.inv_H is None or self.cal.ref_gray is None:
            return [], set(), np.zeros((n, n), dtype=np.float32), [], {}
        raw_occ = np.zeros((n, n), dtype=np.uint8)
        raw_workers: list[tuple[float, float]] = []
        boxes_debug: list[tuple[int, int, int, int, str, float]] = []
        predict_kwargs: dict[str, Any] = {"verbose": False, "conf": self.cfg.conf}
        if self.cfg.device:
            predict_kwargs["device"] = self.cfg.device
        names = self.model.names
        results = self.model.predict(frame, **predict_kwargs)
        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                cls_id = int(box.cls[0].item())
                conf = float(box.conf[0].item())
                name = names.get(cls_id, str(cls_id))
                x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
                boxes_debug.append((int(x1), int(y1), int(x2), int(y2), name, conf))
                if name == "person":
                    foot = project_points(self.cal.H, [[(x1 + x2) * 0.5, y2]])[0]
                    if -0.5 <= foot[0] <= n - 0.5 and -0.5 <= foot[1] <= n - 0.5:
                        raw_workers.append((float(foot[0]), float(foot[1])))
                else:
                    y_top = y1 + (y2 - y1) * 0.55
                    quad = np.array([[x1, y_top], [x2, y_top], [x2, y2], [x1, y2]], dtype=np.float64)
                    self._rasterize_poly(project_points(self.cal.H, quad), raw_occ)
        self.last_boxes = boxes_debug
        self.last_person_points = list(raw_workers)

        gray = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (5, 5), 0).astype(np.float32)
        diff = cv2.absdiff(gray, self.cal.ref_gray)
        _, mask = cv2.threshold(diff.astype(np.uint8), self.cfg.diff_thresh, 255, cv2.THRESH_BINARY)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morph_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morph_kernel)
        if self.cal.floor_mask is not None:
            mask = cv2.bitwise_and(mask, mask, mask=self.cal.floor_mask)
        if self.cfg.adaptive_bg:
            quiet_mask = (mask == 0).astype(np.float32)
            a = self.cfg.bg_alpha
            self.cal.ref_gray = self.cal.ref_gray * (1.0 - a * quiet_mask) + gray * (a * quiet_mask)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) < self.cfg.min_contour_area:
                continue
            approx = cv2.approxPolyDP(contour, 3.0, True).reshape(-1, 2)
            if len(approx) >= 3:
                self._rasterize_poly(project_points(self.cal.H, approx.astype(np.float64)), raw_occ)

        markers = self.marker_localizer.detect(frame, self.cal.H, n)
        self.last_markers = markers
        if markers:
            workers = [(1000 + m["robot_id"], int(m["cell"][0]), int(m["cell"][1])) for m in markers]
        else:
            workers = self.worker_tracker.update(raw_workers, n)
        for _, wx, wy in workers:
            if 0 <= wy < n and 0 <= wx < n:
                raw_occ[wy, wx] = 0
        obstacles, rho = self.occ_filter.update(raw_occ)
        obstacles -= {(wx, wy) for _, wx, wy in workers}
        self.last_quality = self.quality_monitor.assess(frame, self.cal.ref_gray)
        return workers, obstacles, rho, markers, self.last_quality

    def on_mouse(self, event: int, x: int, y: int, flags: int, _param: Any) -> None:
        w = self.cfg.frame_w
        h = self.cfg.frame_h
        n = self.cfg.grid_size
        if x >= w:
            gx = int(((x - w) / float(h)) * n)
            gy = int((y / float(h)) * n)
            if 0 <= gx < n and 0 <= gy < n:
                if event == cv2.EVENT_LBUTTONDOWN:
                    if flags & cv2.EVENT_FLAG_CTRLKEY:
                        self.sync.send_task(gx, gy)
                    else:
                        self.sync.send_target(gx, gy, append=bool(flags & cv2.EVENT_FLAG_SHIFTKEY))
                elif event == cv2.EVENT_RBUTTONDOWN:
                    self.manual_start = [gx, gy]
            return
        if event == cv2.EVENT_LBUTTONDOWN:
            if not self.cal.locked and len(self.cal.pts) < 4:
                self.cal.pts.append((float(x), float(y)))
            elif self.cal.locked and self.cal.H is not None:
                p = project_points(self.cal.H, [(float(x), float(y))])[0]
                c = self.cal.cell_from_grid_pt(p)
                if c is not None:
                    if flags & cv2.EVENT_FLAG_CTRLKEY:
                        self.sync.send_task(c[0], c[1])
                    elif flags & cv2.EVENT_FLAG_SHIFTKEY:
                        self.sync.send_target(c[0], c[1], append=False)
        elif event == cv2.EVENT_RBUTTONDOWN and self.cal.locked and self.cal.H is not None:
            p = project_points(self.cal.H, [(float(x), float(y))])[0]
            c = self.cal.cell_from_grid_pt(p)
            if c is not None:
                self.manual_start = [c[0], c[1]]

    def _draw_banner(self, img: np.ndarray, text: str, color: tuple[int, int, int] = (255, 255, 255)) -> None:
        cv2.rectangle(img, (0, 0), (img.shape[1], 30), (15, 23, 42), -1)
        cv2.putText(img, text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 1, cv2.LINE_AA)

    def draw_camera(
        self,
        frame: np.ndarray,
        obstacles: set[tuple[int, int]],
        workers: list[tuple[int, int, int]],
        snap: dict[str, Any],
    ) -> np.ndarray:
        out = frame.copy()
        n = self.cfg.grid_size
        if self.show_debug:
            for x1, y1, x2, y2, name, conf in self.last_boxes:
                col = (0, 220, 0) if name == "person" else (0, 140, 255)
                cv2.rectangle(out, (x1, y1), (x2, y2), col, 1)
                cv2.putText(out, f"{name} {conf:.2f}", (x1, max(14, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1)
        if self.cal.locked and self.cal.inv_H is not None:
            inv_h = self.cal.inv_H
            for i in range(n + 1):
                a = i - 0.5
                vline = safe_int_pts(project_points(inv_h, [(a, -0.5), (a, n - 0.5)]))
                hline = safe_int_pts(project_points(inv_h, [(-0.5, a), (n - 0.5, a)]))
                cv2.polylines(out, [vline], False, (150, 150, 150), 1, cv2.LINE_AA)
                cv2.polylines(out, [hline], False, (150, 150, 150), 1, cv2.LINE_AA)
            overlay = out.copy()
            for x, y in obstacles:
                poly = safe_int_pts(project_points(inv_h, self.cal.cell_corners(x, y)))
                cv2.fillPoly(overlay, [poly], (0, 0, 210))
                cv2.polylines(out, [poly], True, (0, 0, 255), 1, cv2.LINE_AA)
            for x, y in snap["static_obstacles"]:
                poly = safe_int_pts(project_points(inv_h, self.cal.cell_corners(x, y)))
                cv2.fillPoly(overlay, [poly], (70, 70, 90))
            cv2.addWeighted(overlay, 0.45, out, 0.55, 0.0, out)
            for r_path in snap["fleet_paths"] or ([snap["path"]] if snap["path"] else []):
                if len(r_path) >= 2:
                    pp = safe_int_pts(project_points(inv_h, np.array(r_path, dtype=np.float64)))
                    cv2.polylines(out, [pp], False, (0, 255, 255), 3, cv2.LINE_AA)
                    for i in range(1, len(pp)):
                        cv2.arrowedLine(out, tuple(pp[i - 1]), tuple(pp[i]), (0, 255, 255), 2, cv2.LINE_AA, tipLength=0.35)
            for tx, ty in snap["targets"]:
                tp = safe_int_pts(project_points(inv_h, [(float(tx), float(ty))]))[0]
                cv2.circle(out, (int(tp[0]), int(tp[1])), 8, (0, 140, 255), -1)
            for tid, wx, wy in workers:
                wp = safe_int_pts(project_points(inv_h, [(float(wx), float(wy))]))[0]
                cv2.circle(out, (int(wp[0]), int(wp[1])), 8, (0, 0, 0), -1)
                label = f"R{tid - 1000}" if tid >= 1000 else f"W{tid}"
                cv2.putText(out, label, (int(wp[0]) + 8, int(wp[1]) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            if self.manual_start is not None and not workers:
                mp = safe_int_pts(project_points(inv_h, [(float(self.manual_start[0]), float(self.manual_start[1]))]))[0]
                cv2.circle(out, (int(mp[0]), int(mp[1])), 8, (0, 0, 0), -1)
        if not self.cal.locked:
            for i, (px, py) in enumerate(self.cal.pts):
                cv2.circle(out, (int(px), int(py)), 6, (0, 255, 255), -1)
                cv2.putText(out, str(i + 1), (int(px) + 8, int(py) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            banner = f"CLICK {LABELS[len(self.cal.pts)]}" if len(self.cal.pts) < 4 else "PRESS SPACE TO LOCK CALIBRATION"
            if self.cal.status_msg and len(self.cal.pts) == 4:
                banner = self.cal.status_msg
            self._draw_banner(out, banner, (0, 255, 255))
        elif self.cal.need_ref:
            self._draw_banner(out, "PRESS SPACE TO CAPTURE EMPTY FLOOR REFERENCE", (0, 200, 255))
        else:
            net_status = f"ONLINE {snap['latency_ms']:.0f}ms" if snap["online"] else "BACKEND OFFLINE"
            col = (120, 255, 140) if snap["online"] else (100, 100, 255)
            pause_tag = " [PAUSED]" if self.paused else ""
            estop_tag = " [ESTOP]" if self.estop or snap["estop"] else ""
            q = self.last_quality.get("score", 1.0)
            self._draw_banner(
                out,
                f"FPS:{self.fps:4.1f} | {net_status} | W:{len(workers)} Obs:{len(obstacles)} Q:{q:.2f} ETA:{snap['eta_sec']:.1f}s{pause_tag}{estop_tag}",
                col,
            )
        cv2.putText(
            out,
            "[SPACE]Lock [R]Ref [C]Clear [D]Debug [P]Pause [E]Estop [N]Next [Q]Quit",
            (8, self.cfg.frame_h - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (220, 220, 220),
            1,
            cv2.LINE_AA,
        )
        return out

    def draw_top(
        self,
        obstacles: set[tuple[int, int]],
        workers: list[tuple[int, int, int]],
        rho: np.ndarray,
        snap: dict[str, Any],
    ) -> np.ndarray:
        side = self.cfg.frame_h
        n = self.cfg.grid_size
        cell = side / float(n)
        img = np.full((side, side, 3), 255, dtype=np.uint8)
        static_set = {(p[0], p[1]) for p in snap["static_obstacles"]}
        for y in range(n):
            y0, y1 = int(round(y * cell)), int(round((y + 1) * cell))
            for x in range(n):
                x0, x1 = int(round(x * cell)), int(round((x + 1) * cell))
                v = float(rho[y, x])
                shade = int(round(255.0 - 75.0 * max(0.0, min(1.0, v))))
                cv2.rectangle(img, (x0, y0), (x1, y1), (shade, shade, 255), -1)
                if (x, y) in static_set:
                    cv2.rectangle(img, (x0 + 2, y0 + 2), (x1 - 2, y1 - 2), (80, 65, 50), -1)
                elif (x, y) in obstacles:
                    cv2.rectangle(img, (x0 + 2, y0 + 2), (x1 - 2, y1 - 2), (0, 0, 220), -1)
                cv2.rectangle(img, (x0, y0), (x1, y1), (180, 180, 180), 1)
        route_colors = [(220, 110, 0), (30, 160, 40), (180, 40, 160), (0, 130, 230)]
        paths = snap["fleet_paths"] or ([snap["path"]] if snap["path"] else [])
        for idx, r_path in enumerate(paths):
            if len(r_path) >= 2:
                pp = np.array([[(p[0] + 0.5) * cell, (p[1] + 0.5) * cell] for p in r_path], dtype=np.int32)
                cv2.polylines(img, [pp], False, route_colors[idx % len(route_colors)], 3, cv2.LINE_AA)
        for idx, (tx, ty) in enumerate(snap["targets"]):
            cx, cy = int((tx + 0.5) * cell), int((ty + 0.5) * cell)
            cv2.circle(img, (cx, cy), max(6, int(cell * 0.25)), (0, 140, 255), -1)
            cv2.putText(img, str(idx + 1), (cx - 4, cy + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
        for tid, wx, wy in workers:
            cx, cy = int((wx + 0.5) * cell), int((wy + 0.5) * cell)
            cv2.circle(img, (cx, cy), max(6, int(cell * 0.24)), (0, 0, 0), -1)
            label = f"R{tid - 1000}" if tid >= 1000 else f"W{tid}"
            cv2.putText(img, label, (cx - 12, cy - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
        if self.manual_start is not None and not workers:
            cx, cy = int((self.manual_start[0] + 0.5) * cell), int((self.manual_start[1] + 0.5) * cell)
            cv2.circle(img, (cx, cy), max(6, int(cell * 0.24)), (0, 0, 0), -1)
        if self.estop or snap["estop"]:
            cv2.rectangle(img, (0, 0), (side, 30), (0, 0, 200), -1)
            cv2.putText(img, "SAFETY STOP ACTIVE", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1, cv2.LINE_AA)
        else:
            cv2.putText(
                img,
                "TOP-DOWN GRID (L:Target | SHIFT:Append | CTRL:Task | R:Start)",
                (8, 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.43,
                (20, 20, 20),
                1,
                cv2.LINE_AA,
            )
        return img

    def run(self) -> None:
        def handle_sig(_sig: int, _frame: Any) -> None:
            self.running = False

        signal.signal(signal.SIGINT, handle_sig)
        signal.signal(signal.SIGTERM, handle_sig)
        cap = self._open_capture()
        if not self.cfg.headless:
            cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
            cv2.setMouseCallback(WINDOW, self.on_mouse)
        last_t = time.perf_counter()
        reconnect_delay = 1.0
        try:
            while self.running:
                if not self.cfg.headless and not self._window_visible():
                    logger.info("OpenCV window closed by operator")
                    self.running = False
                    break
                if not cap.isOpened():
                    logger.warning("Camera unavailable; retrying in %.1fs...", reconnect_delay)
                    time.sleep(reconnect_delay)
                    reconnect_delay = min(5.0, reconnect_delay * 1.5)
                    cap = self._open_capture()
                    continue
                ok, frame = cap.read()
                if not ok or frame is None:
                    logger.warning("Frame grab failed; attempting stream reconnect")
                    cap.release()
                    time.sleep(0.5)
                    cap = self._open_capture()
                    continue
                reconnect_delay = 1.0
                frame = cv2.resize(frame, (self.cfg.frame_w, self.cfg.frame_h))
                now = time.perf_counter()
                dt = max(1e-4, now - last_t)
                last_t = now
                self.fps = 0.85 * self.fps + 0.15 * (1.0 / dt)
                if not self.paused and self.cal.locked and self.cal.ref_gray is not None:
                    workers, obstacles, rho, robots, quality = self.detect(frame)
                    worker_coords = (
                        [[wx, wy] for _, wx, wy in workers]
                        if workers
                        else [self.manual_start or [0, 0]]
                    )
                    snap = self.sync.get_snapshot()
                    proximity = self.proximity_monitor.assess(
                        self.last_person_points,
                        robots,
                        snap["path"],
                        self.cfg.cell_size_m,
                    )
                    self.sync.publish_vision(
                        {
                            "obstacles": [[x, y] for x, y in sorted(obstacles)],
                            "rho": np.round(rho, 4).tolist(),
                            "workers": worker_coords,
                            "robots": robots,
                            "quality": quality,
                            "proximity": proximity,
                            "fps": round(self.fps, 1),
                            "estop": self.estop,
                        }
                    )
                else:
                    workers = []
                    obstacles = set()
                    rho = np.zeros((self.cfg.grid_size, self.cfg.grid_size), dtype=np.float32)
                if not self.cfg.headless:
                    snap = self.sync.get_snapshot()
                    cam_img = self.draw_camera(frame, obstacles, workers, snap)
                    top_img = self.draw_top(obstacles, workers, rho, snap)
                    cv2.imshow(WINDOW, np.hstack([cam_img, top_img]))
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        logger.info("Quit key pressed")
                        self.running = False
                        break
                    if not self._window_visible():
                        logger.info("OpenCV window closed by operator")
                        self.running = False
                        break
                    if key == ord("c"):
                        self.cal.clear()
                        self.occ_filter.reset()
                        self.manual_start = None
                    elif key == ord("r") and self.cal.locked:
                        self.cal.capture_reference(frame)
                        self.occ_filter.reset()
                    elif key == ord("p"):
                        self.paused = not self.paused
                    elif key == ord("d"):
                        self.show_debug = not self.show_debug
                    elif key == ord("e"):
                        self.estop = not self.estop
                        self.sync.send_estop(self.estop)
                    elif key == ord("n"):
                        active = [t for t in snap["tasks"] if t.get("status") in ("pending", "assigned", "active")]
                        if active:
                            self.sync.complete_task(str(active[0].get("id", "")))
                    elif key == 32:
                        if not self.cal.locked:
                            self.cal.save_calibration(frame)
                        elif self.cal.need_ref:
                            self.cal.capture_reference(frame)
        finally:
            self.sync.close()
            cap.release()
            if not self.cfg.headless:
                cv2.destroyAllWindows()
            logger.info("MAVRYKS Tracker shut down cleanly")


def parse_args() -> TrackerConfig:
    p = argparse.ArgumentParser(description="MAVRYKS Physical AI Vision Tracker")
    p.add_argument("--source", default=os.getenv("CAMERA_SOURCE", "0"), help="Live webcam index (default 0)")
    p.add_argument("--backend", default=os.getenv("BACKEND_URL", "http://127.0.0.1:8000"), help="Backend API base URL")
    p.add_argument("--model", default=os.getenv("MODEL_PATH", "yolov8n.pt"), help="YOLO weights path")
    p.add_argument("--device", default=os.getenv("YOLO_DEVICE") or None, help="Inference device")
    p.add_argument("--grid-size", type=int, default=int(os.getenv("GRID_SIZE", "12")), help="Grid dimension")
    p.add_argument("--conf", type=float, default=0.30, help="YOLO confidence threshold")
    p.add_argument("--cell-size-m", type=float, default=1.0, help="Physical size of one grid cell")
    p.add_argument("--no-adaptive-bg", action="store_true", help="Disable gradual floor illumination adaptation")
    p.add_argument("--headless", action="store_true", help="Run without OpenCV GUI windows")
    args = p.parse_args()
    return TrackerConfig(
        source=args.source,
        backend_url=args.backend.rstrip("/"),
        model_path=args.model,
        device=args.device,
        grid_size=max(4, args.grid_size),
        conf=args.conf,
        cell_size_m=max(0.01, args.cell_size_m),
        adaptive_bg=not args.no_adaptive_bg,
        headless=args.headless,
    )


def main() -> None:
    cfg = parse_args()
    MavryksTracker(cfg).run()


if __name__ == "__main__":
    main()
