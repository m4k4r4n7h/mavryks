from __future__ import annotations

from typing import Any

import cv2
import numpy as np


class FrameQuality:
    def __init__(self, blur_floor: float = 60.0, exposure_low: float = 35.0, exposure_high: float = 225.0) -> None:
        self.blur_floor = float(blur_floor)
        self.exposure_low = float(exposure_low)
        self.exposure_high = float(exposure_high)

    def assess(self, frame: np.ndarray, ref_gray: np.ndarray | None = None) -> dict[str, Any]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        mean = float(np.mean(gray))
        glare = float(np.mean(gray > 245))
        shadow = float(np.mean(gray < 12))
        drift = 0.0
        if ref_gray is not None and ref_gray.shape == gray.shape:
            drift = float(np.mean(cv2.absdiff(gray, ref_gray.astype(np.uint8))))

        blur_score = min(1.0, blur / self.blur_floor)
        exposure_score = 1.0 - min(1.0, max(0.0, self.exposure_low - mean) / self.exposure_low)
        exposure_score *= 1.0 - min(1.0, max(0.0, mean - self.exposure_high) / (255.0 - self.exposure_high))
        glare_score = 1.0 - min(1.0, glare * 8.0)
        drift_score = 1.0 - min(1.0, drift / 90.0)
        score = max(0.0, min(1.0, blur_score * exposure_score * glare_score * drift_score))
        alerts: list[str] = []
        if blur_score < 0.5:
            alerts.append("BLUR")
        if exposure_score < 0.5:
            alerts.append("EXPOSURE")
        if glare_score < 0.5:
            alerts.append("GLARE")
        if drift_score < 0.5:
            alerts.append("LIGHT_DRIFT")
        if shadow > 0.25:
            alerts.append("SHADOW")
        return {
            "score": round(score, 4),
            "blur": round(blur, 2),
            "exposure": round(mean, 2),
            "glare": round(glare, 4),
            "shadow": round(shadow, 4),
            "drift": round(drift, 2),
            "alerts": alerts,
        }
