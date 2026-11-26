from __future__ import annotations

import math
from typing import Any

import cv2
import numpy as np


class MarkerLocalizer:
    def __init__(self, dictionary_id: int = cv2.aruco.DICT_4X4_50, min_area_px: float = 36.0) -> None:
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.params)
        self.min_area_px = float(min_area_px)

    @staticmethod
    def project(matrix: np.ndarray, pts: np.ndarray) -> np.ndarray:
        a = np.asarray(pts, dtype=np.float64).reshape(-1, 2)
        homog = np.column_stack((a, np.ones(len(a), dtype=np.float64)))
        q = homog @ matrix.T
        z = q[:, 2:3]
        z = np.where(np.abs(z) < 1e-6, np.where(z < 0, -1e-6, 1e-6), z)
        out = q[:, :2] / z
        return np.nan_to_num(out, nan=0.0, posinf=1e5, neginf=-1e5)

    def detect(self, frame: np.ndarray, h_mat: np.ndarray, grid_size: int) -> list[dict[str, Any]]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        out: list[dict[str, Any]] = []
        if ids is None:
            return out
        n = max(1, int(grid_size))
        for corner_set, marker_id in zip(corners, ids.flatten()):
            quad = np.asarray(corner_set, dtype=np.float32).reshape(4, 2)
            area = abs(float(cv2.contourArea(quad)))
            if area < self.min_area_px:
                continue
            bottom_left = quad[3]
            bottom_right = quad[2]
            foot_px = np.array([(bottom_left[0] + bottom_right[0]) * 0.5, (bottom_left[1] + bottom_right[1]) * 0.5])
            foot_grid = self.project(h_mat, foot_px)[0]
            bl_grid = self.project(h_mat, bottom_left)[0]
            br_grid = self.project(h_mat, bottom_right)[0]
            gx = int(round(float(foot_grid[0])))
            gy = int(round(float(foot_grid[1])))
            if not (0 <= gx < n and 0 <= gy < n):
                continue
            dx = float(br_grid[0] - bl_grid[0])
            dy = float(br_grid[1] - bl_grid[1])
            heading = math.degrees(math.atan2(dy, dx)) % 360.0
            out.append(
                {
                    "robot_id": int(marker_id),
                    "cell": [gx, gy],
                    "pos": [round(float(foot_grid[0]), 4), round(float(foot_grid[1]), 4)],
                    "heading_deg": round(heading, 2),
                    "marker_area_px": round(area, 2),
                    "speed_mps": 0.0,
                    "battery_pct": 100.0,
                    "source": "aruco",
                }
            )
        out.sort(key=lambda item: item["robot_id"])
        return out
