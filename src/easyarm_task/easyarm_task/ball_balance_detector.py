# Copyright 2026 linx
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Grayscale edge based detector for the EasyArm ball balance task."""

from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class DetectionConfig:
    """Thresholds used by the plate and ball detector."""

    plate_min_radius: float
    plate_max_radius: float
    plate_roi_x_min: float
    plate_roi_x_max: float
    plate_roi_y_min: float
    plate_roi_y_max: float
    plate_expected_x: float
    plate_expected_y: float
    plate_expected_radius: float
    ball_min_area: float
    ball_max_area: float
    ball_min_radius: float
    ball_max_radius: float
    ball_max_value: int
    ball_min_circularity: float
    ball_plate_inner_scale: float


@dataclass(frozen=True)
class CircleDetection:
    """A detected circle in image coordinates."""

    center: tuple[float, float]
    radius: float
    area: float
    circularity: float
    bbox: tuple[int, int, int, int]


@dataclass(frozen=True)
class BallBalanceDetection:
    """Combined detection result for one frame."""

    plate: CircleDetection | None
    ball: CircleDetection | None
    offset: tuple[float, float] | None
    plate_mask: np.ndarray | None
    ball_mask: np.ndarray | None
    debug: "DetectionDebug | None" = None


@dataclass(frozen=True)
class PlateCandidateDebug:
    """Debug data for a plate candidate."""

    detection: CircleDetection
    score: float | None
    accepted: bool
    reason: str
    edge_support: float | None = None
    color_ratio: float | None = None


@dataclass(frozen=True)
class DetectionDebug:
    """Intermediate images and candidate data for detector tuning."""

    gray: np.ndarray
    roi_mask: np.ndarray
    edge_mask: np.ndarray
    plate_mask: np.ndarray
    ball_mask: np.ndarray | None
    plate_candidates: list[PlateCandidateDebug]


def detect_objects(
    frame: np.ndarray,
    config: DetectionConfig,
) -> BallBalanceDetection:
    """Detect the plate and dark ball in a BGR frame."""
    plate, plate_mask, plate_debug = detect_plate(frame, config)
    if plate is None:
        debug = DetectionDebug(
            gray=plate_debug.gray,
            roi_mask=plate_debug.roi_mask,
            edge_mask=plate_debug.edge_mask,
            plate_mask=plate_mask,
            ball_mask=None,
            plate_candidates=plate_debug.plate_candidates,
        )
        return BallBalanceDetection(None, None, None, plate_mask, None, debug)

    ball, ball_mask = detect_ball(frame, plate, config)
    offset = None
    if ball is not None and plate.radius > 0.0:
        offset = (
            (ball.center[0] - plate.center[0]) / plate.radius,
            (ball.center[1] - plate.center[1]) / plate.radius,
        )
    debug = DetectionDebug(
        gray=plate_debug.gray,
        roi_mask=plate_debug.roi_mask,
        edge_mask=plate_debug.edge_mask,
        plate_mask=plate_mask,
        ball_mask=ball_mask,
        plate_candidates=plate_debug.plate_candidates,
    )
    return BallBalanceDetection(plate, ball, offset, plate_mask, ball_mask,
                                debug)


def detect_plate(
    frame: np.ndarray,
    config: DetectionConfig,
) -> tuple[CircleDetection | None, np.ndarray, DetectionDebug]:
    """Detect the white plate with grayscale Hough circle detection."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (9, 9), 0)
    roi_mask = normalized_roi_mask(
        gray.shape[:2],
        config.plate_roi_x_min,
        config.plate_roi_x_max,
        config.plate_roi_y_min,
        config.plate_roi_y_max,
    )
    edges = cv2.Canny(gray, 45, 120)
    edge_mask = cv2.bitwise_and(edges, roi_mask)
    candidates = detect_plate_candidates(gray, frame, edge_mask, config)
    accepted = [candidate for candidate in candidates if candidate.accepted]
    best = max(
        accepted,
        key=lambda candidate: candidate.score or float("-inf"),
        default=None,
    )
    plate = best.detection if best is not None else None
    plate_mask = make_plate_mask(gray.shape[:2], plate)
    debug = DetectionDebug(
        gray=gray,
        roi_mask=roi_mask,
        edge_mask=edge_mask,
        plate_mask=plate_mask,
        ball_mask=None,
        plate_candidates=candidates,
    )
    return plate, plate_mask, debug


def detect_plate_candidates(
    gray: np.ndarray,
    frame: np.ndarray,
    edge_mask: np.ndarray,
    config: DetectionConfig,
) -> list[PlateCandidateDebug]:
    """Detect and score plate circle candidates."""
    x0, y0, x1, y1 = roi_bounds(gray.shape[:2], config)
    roi = gray[y0:y1, x0:x1]
    circles = cv2.HoughCircles(
        roi,
        cv2.HOUGH_GRADIENT,
        dp=1.0,
        minDist=120,
        param1=100,
        param2=26,
        minRadius=int(config.plate_min_radius),
        maxRadius=int(config.plate_max_radius),
    )
    if circles is None:
        return []

    candidates = []
    for cx, cy, radius in circles[0, :]:
        circle = circle_from_hough(cx + x0, cy + y0, radius, gray.shape[:2])
        if not circle_inside_roi(circle, (x0, y0, x1, y1)):
            candidates.append(rejected_plate_candidate(circle, "roi"))
            continue
        circle = refine_circle_from_edges(
            edge_mask,
            circle,
            (x0, y0, x1, y1),
            config,
        )
        edge_support = circular_edge_support(edge_mask, circle)
        color_ratio = circle_plate_color_ratio(frame, circle, 0.82)
        if edge_support < 0.18:
            candidates.append(rejected_plate_candidate(
                circle,
                "edge",
                edge_support=edge_support,
                color_ratio=color_ratio,
            ))
            continue
        if color_ratio < 0.40:
            candidates.append(rejected_plate_candidate(
                circle,
                "color",
                edge_support=edge_support,
                color_ratio=color_ratio,
            ))
            continue
        score = plate_score(frame, circle, edge_support, color_ratio, config)
        candidates.append(accepted_plate_candidate(
            circle,
            score,
            edge_support=edge_support,
            color_ratio=color_ratio,
        ))
    return candidates


def refine_circle_from_edges(
    edge_mask: np.ndarray,
    circle: CircleDetection,
    roi: tuple[int, int, int, int],
    config: DetectionConfig,
) -> CircleDetection:
    """Refine a Hough circle with nearby Canny edge points."""
    band_width = 3.0
    max_center_shift = 6.0
    max_radius_shift = 6.0
    min_points = 80
    height, width = edge_mask.shape[:2]
    search_radius = circle.radius + band_width + 2.0
    x0 = max(roi[0], int(math.floor(circle.center[0] - search_radius)))
    x1 = min(roi[2], int(math.ceil(circle.center[0] + search_radius)) + 1)
    y0 = max(roi[1], int(math.floor(circle.center[1] - search_radius)))
    y1 = min(roi[3], int(math.ceil(circle.center[1] + search_radius)) + 1)
    if x1 <= x0 or y1 <= y0:
        return circle

    patch = edge_mask[y0:y1, x0:x1]
    ys, xs = np.nonzero(patch)
    if len(xs) < min_points:
        return circle

    xs = xs.astype(np.float64) + x0
    ys = ys.astype(np.float64) + y0
    distances = np.hypot(xs - circle.center[0], ys - circle.center[1])
    near_perimeter = np.abs(distances - circle.radius) <= band_width
    xs = xs[near_perimeter]
    ys = ys[near_perimeter]
    if len(xs) < min_points:
        return circle

    refined = fit_circle_to_points(xs, ys, (height, width))
    if refined is None:
        return circle
    if math.dist(refined.center, circle.center) > max_center_shift:
        return circle
    if abs(refined.radius - circle.radius) > max_radius_shift:
        return circle
    if refined.radius < config.plate_min_radius:
        return circle
    if refined.radius > config.plate_max_radius:
        return circle
    if not circle_inside_roi(refined, roi):
        return circle
    return refined


def fit_circle_to_points(
    xs: np.ndarray,
    ys: np.ndarray,
    image_shape: tuple[int, int],
) -> CircleDetection | None:
    """Fit a circle to edge points with a centered least-squares solve."""
    x_mean = float(np.mean(xs))
    y_mean = float(np.mean(ys))
    us = xs - x_mean
    vs = ys - y_mean
    system = np.column_stack((2.0 * us, 2.0 * vs, np.ones_like(us)))
    rhs = us * us + vs * vs
    try:
        a, b, c = np.linalg.lstsq(system, rhs, rcond=None)[0]
    except np.linalg.LinAlgError:
        return None
    radius_squared = c + a * a + b * b
    if radius_squared <= 0.0:
        return None

    center = (x_mean + float(a), y_mean + float(b))
    radius = math.sqrt(float(radius_squared))
    if not math.isfinite(center[0] + center[1] + radius):
        return None
    area = math.pi * radius * radius
    bbox = circle_bbox(center, radius, image_shape)
    return CircleDetection(center, radius, area, 1.0, bbox)


def circle_from_hough(
    x: float,
    y: float,
    radius: float,
    image_shape: tuple[int, int],
) -> CircleDetection:
    """Create a circle detection from Hough output."""
    radius_float = float(radius)
    area = math.pi * radius_float * radius_float
    bbox = circle_bbox((float(x), float(y)), radius_float, image_shape)
    return CircleDetection(
        (float(x), float(y)),
        radius_float,
        area,
        1.0,
        bbox,
    )


def plate_score(
    frame: np.ndarray,
    circle: CircleDetection,
    edge_support: float,
    color_ratio: float,
    config: DetectionConfig,
) -> float:
    """Score a plate circle candidate."""
    expected = (
        frame.shape[1] * config.plate_expected_x,
        frame.shape[0] * config.plate_expected_y,
    )
    center_penalty = math.dist(circle.center, expected) / max(
        frame.shape[1],
        1,
    )
    expected_radius = max(config.plate_expected_radius, 1.0)
    radius_penalty = abs(circle.radius - expected_radius) / expected_radius
    return (
        edge_support * 10000.0
        + color_ratio * 6000.0
        - center_penalty * 3000.0
        - radius_penalty * 2500.0
    )


def detect_ball(
    frame: np.ndarray,
    plate: CircleDetection,
    config: DetectionConfig,
) -> tuple[CircleDetection | None, np.ndarray]:
    """Detect the dark circular object inside the detected plate."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    dark = cv2.inRange(hsv, (0, 0, 0), (179, 255, config.ball_max_value))
    plate_inner_mask = make_plate_mask(
        frame.shape[:2],
        plate,
        scale=config.ball_plate_inner_scale,
    )
    mask = cv2.bitwise_and(dark, plate_inner_mask)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)

    best = None
    best_score = float("-inf")
    for contour in contours:
        candidate = contour_to_circle(contour, frame.shape[:2])
        if candidate is None:
            continue
        if candidate.area < config.ball_min_area:
            continue
        if candidate.area > config.ball_max_area:
            continue
        if candidate.radius < config.ball_min_radius:
            continue
        if candidate.radius > config.ball_max_radius:
            continue
        if candidate.circularity < config.ball_min_circularity:
            continue
        distance = math.dist(candidate.center, plate.center)
        score = candidate.area + candidate.circularity * 1000.0
        score -= distance * 2.0
        if score > best_score:
            best = candidate
            best_score = score
    return best, mask


def accepted_plate_candidate(
    detection: CircleDetection,
    score: float,
    edge_support: float,
    color_ratio: float,
) -> PlateCandidateDebug:
    """Build an accepted plate candidate record."""
    return PlateCandidateDebug(
        detection=detection,
        score=score,
        accepted=True,
        reason="accepted",
        edge_support=edge_support,
        color_ratio=color_ratio,
    )


def rejected_plate_candidate(
    detection: CircleDetection,
    reason: str,
    edge_support: float | None = None,
    color_ratio: float | None = None,
) -> PlateCandidateDebug:
    """Build a rejected plate candidate record."""
    return PlateCandidateDebug(
        detection=detection,
        score=None,
        accepted=False,
        reason=reason,
        edge_support=edge_support,
        color_ratio=color_ratio,
    )


def normalized_roi_mask(
    shape: tuple[int, int],
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
) -> np.ndarray:
    """Create a binary mask from normalized ROI bounds."""
    x0, y0, x1, y1 = normalized_roi_bounds(shape, x_min, x_max, y_min, y_max)
    mask = np.zeros(shape, dtype=np.uint8)
    if x1 > x0 and y1 > y0:
        mask[y0:y1, x0:x1] = 255
    return mask


def roi_bounds(
    shape: tuple[int, int],
    config: DetectionConfig,
) -> tuple[int, int, int, int]:
    """Return detector ROI bounds."""
    return normalized_roi_bounds(
        shape,
        config.plate_roi_x_min,
        config.plate_roi_x_max,
        config.plate_roi_y_min,
        config.plate_roi_y_max,
    )


def normalized_roi_bounds(
    shape: tuple[int, int],
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
) -> tuple[int, int, int, int]:
    """Convert normalized ROI bounds to integer pixel bounds."""
    height, width = shape
    x0 = int(round(clamp(x_min, 0.0, 1.0) * width))
    x1 = int(round(clamp(x_max, 0.0, 1.0) * width))
    y0 = int(round(clamp(y_min, 0.0, 1.0) * height))
    y1 = int(round(clamp(y_max, 0.0, 1.0) * height))
    x0, x1 = sorted((max(0, x0), min(width, x1)))
    y0, y1 = sorted((max(0, y0), min(height, y1)))
    return x0, y0, x1, y1


def circle_inside_roi(
    circle: CircleDetection,
    roi: tuple[int, int, int, int],
) -> bool:
    """Return whether a whole circle lies inside ROI bounds."""
    x0, y0, x1, y1 = roi
    return (
        circle.center[0] - circle.radius >= x0
        and circle.center[0] + circle.radius <= x1
        and circle.center[1] - circle.radius >= y0
        and circle.center[1] + circle.radius <= y1
    )


def circular_edge_support(
    edge_mask: np.ndarray,
    circle: CircleDetection,
) -> float:
    """Measure edge support around a circle perimeter."""
    samples = max(120, int(round(2.0 * math.pi * circle.radius)))
    hits = 0
    height, width = edge_mask.shape[:2]
    for index in range(samples):
        theta = 2.0 * math.pi * index / samples
        x = int(round(circle.center[0] + math.cos(theta) * circle.radius))
        y = int(round(circle.center[1] + math.sin(theta) * circle.radius))
        if x < 0 or y < 0 or x >= width or y >= height:
            continue
        patch = edge_mask[max(0, y - 1):min(height, y + 2),
                          max(0, x - 1):min(width, x + 2)]
        if np.any(patch > 0):
            hits += 1
    return hits / samples if samples > 0 else 0.0


def circle_plate_color_ratio(
    frame: np.ndarray,
    circle: CircleDetection,
    scale: float,
) -> float:
    """Return ratio of configured plate-color pixels inside a circle."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = make_plate_mask(frame.shape[:2], circle, scale=scale)
    green = cv2.inRange(hsv, (35, 60, 60), (95, 255, 255))
    white = cv2.inRange(hsv, (0, 0, 135), (179, 80, 255))
    plate_color = cv2.bitwise_or(green, white)
    total = cv2.countNonZero(mask)
    if total <= 0:
        return 0.0
    selected = cv2.countNonZero(cv2.bitwise_and(plate_color, mask))
    return selected / total


def make_plate_mask(
    shape: tuple[int, int],
    plate: CircleDetection | None,
    scale: float = 1.0,
) -> np.ndarray:
    """Create a circular mask for the plate."""
    mask = np.zeros(shape, dtype=np.uint8)
    if plate is None:
        return mask
    cv2.circle(
        mask,
        round_point(plate.center),
        max(1, int(round(plate.radius * scale))),
        255,
        -1,
    )
    return mask


def contour_to_circle(
    contour: np.ndarray,
    image_shape: tuple[int, int],
) -> CircleDetection | None:
    """Convert a contour to a circular detection candidate."""
    area = float(cv2.contourArea(contour))
    if area <= 0.0:
        return None
    (cx, cy), radius = cv2.minEnclosingCircle(contour)
    if radius <= 0.0:
        return None
    perimeter = cv2.arcLength(contour, True)
    if perimeter <= 0.0:
        return None
    circularity = 4.0 * math.pi * area / (perimeter * perimeter)
    bbox = circle_bbox((cx, cy), radius, image_shape)
    return CircleDetection((float(cx), float(cy)), float(radius), area,
                           circularity, bbox)


def circle_bbox(
    center: tuple[float, float],
    radius: float,
    image_shape: tuple[int, int],
) -> tuple[int, int, int, int]:
    """Convert a circle to a clamped x/y/width/height bounding box."""
    height, width = image_shape
    x0 = max(0, int(round(center[0] - radius)))
    y0 = max(0, int(round(center[1] - radius)))
    x1 = min(width - 1, int(round(center[0] + radius)))
    y1 = min(height - 1, int(round(center[1] + radius)))
    return x0, y0, max(1, x1 - x0), max(1, y1 - y0)


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp a number into a closed interval."""
    return max(lower, min(upper, value))


def round_point(point: tuple[float, float]) -> tuple[int, int]:
    """Round a floating-point image coordinate to integer pixels."""
    return int(round(point[0])), int(round(point[1]))
