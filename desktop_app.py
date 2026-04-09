from __future__ import annotations

import csv
from dataclasses import dataclass
from datetime import datetime, timedelta
import importlib.util
import itertools
import math
import os
from pathlib import Path
import queue
import re
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageTk

try:
    from openpyxl import Workbook
except Exception:
    Workbook = None


VIDEO_EXTENSIONS = {".mp4", ".avi", ".mov", ".mkv", ".wmv", ".m4v"}
CATEGORY_ORDER = ("person", "bicycle", "motorcycle", "car", "truck")
CATEGORY_LABELS = {
    "person": "보행자",
    "bicycle": "자전거/킥보드",
    "motorcycle": "오토바이",
    "car": "승용차",
    "truck": "트럭",
}
CATEGORY_TABLE_LABELS = {
    "person": "보행",
    "bicycle": "자전거/킥보드",
    "motorcycle": "오토바이",
    "car": "승용",
    "truck": "트럭",
}
YOLOX_CLASS_TO_CATEGORY = {
    0: "person",
    1: "bicycle",
    2: "car",
    3: "motorcycle",
    5: "truck",
    7: "truck",
}


@dataclass
class CrossingTrack:
    track_id: int
    category: str
    bbox: tuple[int, int, int, int]
    centroid: tuple[float, float]
    previous_centroid: tuple[float, float]
    last_seen: float
    last_cross_time: float = -1e9
    seen_count: int = 1
    stable_side: int = 0
    velocity: tuple[float, float] = (0.0, 0.0)
    high_conf_hits: int = 1


class LineCrossingTracker:
    def __init__(self) -> None:
        self.tracks: list[CrossingTrack] = []
        self.next_id = 1
        self.high_confidence_threshold = 0.30
        self.low_confidence_threshold = 0.10

    def reset(self) -> None:
        self.tracks.clear()
        self.next_id = 1

    def update(
        self,
        detections: list[dict],
        current_time: float,
        match_distance: float,
        cooldown_seconds: float,
        line_start: tuple[int, int],
        line_end: tuple[int, int],
        timestamp_value: datetime | None,
        timestamp_text: str,
        video_name: str,
    ) -> list[dict]:
        events: list[dict] = []
        track_max_age_seconds = max(1.0, cooldown_seconds * 0.25)
        min_confirmations = 3
        line_margin_px = 12.0
        self.tracks = [
            track for track in self.tracks if current_time - track.last_seen <= track_max_age_seconds
        ]

        high_conf_detections = [d for d in detections if float(d.get("score", 0.0)) >= self.high_confidence_threshold]
        low_conf_detections = [
            d for d in detections
            if self.low_confidence_threshold <= float(d.get("score", 0.0)) < self.high_confidence_threshold
        ]

        matched_track_ids, unmatched_high = self._match_tracks(
            detections=high_conf_detections,
            current_time=current_time,
            match_distance=match_distance,
            high_confidence=True,
        )
        matched_track_ids, _unmatched_low = self._match_tracks(
            detections=low_conf_detections,
            current_time=current_time,
            match_distance=match_distance,
            high_confidence=False,
            candidate_track_ids=set(self._track_ids()) - matched_track_ids,
        )

        for detection in unmatched_high:
            new_track = CrossingTrack(
                track_id=self.next_id,
                category=detection.get("category", "person"),
                bbox=detection["bbox"],
                centroid=detection["centroid"],
                previous_centroid=detection["centroid"],
                last_seen=current_time,
                stable_side=classify_point_side(detection["centroid"], line_start, line_end, line_margin_px),
                high_conf_hits=1,
            )
            detection["track_id"] = new_track.track_id
            detection["direction"] = None
            self.next_id += 1
            self.tracks.append(new_track)

        for detection in detections:
            track_id = detection.get("track_id")
            if track_id is None:
                continue
            track = self._find_track(track_id)
            if track is None:
                continue

            direction = None
            current_side = classify_point_side(track.centroid, line_start, line_end, line_margin_px)
            previous_side = track.stable_side
            if current_side != 0:
                if previous_side == 0:
                    track.stable_side = current_side
                elif current_side != previous_side and track.high_conf_hits >= min_confirmations:
                    direction = "IN" if previous_side < 0 < current_side else "OUT"
                    track.stable_side = current_side

            detection["direction"] = direction
            if direction and current_time - track.last_cross_time >= cooldown_seconds:
                track.last_cross_time = current_time
                events.append(
                    {
                        "track_id": track.track_id,
                        "category": track.category,
                        "direction": direction,
                        "elapsed_seconds": current_time,
                        "timestamp": timestamp_value,
                        "timestamp_text": timestamp_text,
                        "video_name": video_name,
                    }
                )

        return events

    def _track_ids(self) -> list[int]:
        return [track.track_id for track in self.tracks]

    def _find_track(self, track_id: int) -> CrossingTrack | None:
        for track in self.tracks:
            if track.track_id == track_id:
                return track
        return None

    def _match_tracks(
        self,
        detections: list[dict],
        current_time: float,
        match_distance: float,
        high_confidence: bool,
        candidate_track_ids: set[int] | None = None,
    ) -> tuple[set[int], list[dict]]:
        if not detections:
            return set(), []

        track_lookup = {
            track.track_id: track
            for track in self.tracks
            if candidate_track_ids is None or track.track_id in candidate_track_ids
        }
        candidates: list[tuple[float, int, int]] = []
        indexed_detections = list(enumerate(detections))
        for track_id, track in track_lookup.items():
            predicted = predict_track_centroid(track, current_time)
            for detection_index, detection in indexed_detections:
                if track.category != detection.get("category", "person"):
                    continue
                distance = math.hypot(predicted[0] - detection["centroid"][0], predicted[1] - detection["centroid"][1])
                iou = bbox_iou(track.bbox, detection["bbox"])
                dynamic_match_distance = max(match_distance * 0.65, min(track.bbox[2], detection["bbox"][2]) * 0.8)
                if iou < 0.01 and distance > dynamic_match_distance:
                    continue
                score = distance - (iou * 140.0) - (float(detection.get("score", 0.0)) * 25.0)
                candidates.append((score, track_id, detection_index))

        candidates.sort(key=lambda item: item[0])
        matched_tracks: set[int] = set()
        matched_detection_indices: set[int] = set()
        for _score, track_id, detection_index in candidates:
            if track_id in matched_tracks or detection_index in matched_detection_indices:
                continue
            track = track_lookup[track_id]
            detection = detections[detection_index]
            update_track_from_detection(track, detection, current_time, high_confidence)
            detection["track_id"] = track.track_id
            matched_tracks.add(track_id)
            matched_detection_indices.add(detection_index)

        unmatched = [
            detection
            for detection_index, detection in indexed_detections
            if detection_index not in matched_detection_indices
        ]
        return matched_tracks, unmatched


class GlyphTemplateOCR:
    def __init__(self) -> None:
        self.canvas_size = (28, 44)
        self.threshold_width_ratio = 0.38
        self.threshold_height_ratio = 0.12
        self.threshold_margin_ratio = 0.015
        self.right_threshold_width_ratio = 0.28
        self.right_threshold_height_ratio = 0.09
        self.right_threshold_side_margin_ratio = 0.0
        self.right_threshold_bottom_margin_ratio = 0.0
        self.overlay_corners = ("bottom_left", "bottom_right")
        self.char_templates = self._build_templates()

    def _build_templates(self) -> dict[str, list[np.ndarray]]:
        charset = "0123456789:-/."
        font_candidates = [
            Path("C:/Windows/Fonts/consola.ttf"),
            Path("C:/Windows/Fonts/arial.ttf"),
            Path("C:/Windows/Fonts/arialbd.ttf"),
            Path("C:/Windows/Fonts/tahoma.ttf"),
            Path("C:/Windows/Fonts/segoeui.ttf"),
            Path("C:/Windows/Fonts/lucon.ttf"),
            Path("C:/Windows/Fonts/malgun.ttf"),
        ]
        available_fonts = [path for path in font_candidates if path.exists()]
        if not available_fonts:
            return {char: [self._render_char(char, None, 28)] for char in charset}

        templates: dict[str, list[np.ndarray]] = {char: [] for char in charset}
        for font_path in available_fonts[:5]:
            for font_size in (24, 28, 32, 36):
                for char in charset:
                    templates[char].append(self._render_char(char, font_path, font_size))
        return templates

    def _render_char(self, char: str, font_path: Path | None, font_size: int) -> np.ndarray:
        width, height = self.canvas_size
        image = Image.new("L", (width, height), 0)
        draw = ImageDraw.Draw(image)
        if font_path is None:
            font = ImageFont.load_default()
        else:
            font = ImageFont.truetype(str(font_path), font_size)
        bbox = draw.textbbox((0, 0), char, font=font)
        text_width = bbox[2] - bbox[0]
        text_height = bbox[3] - bbox[1]
        x = (width - text_width) // 2 - bbox[0]
        y = (height - text_height) // 2 - bbox[1]
        draw.text((x, y), char, fill=255, font=font)
        return self._normalize_char(np.array(image, dtype=np.uint8))

    def get_overlay_bounds(self, frame_shape: tuple[int, ...], corner: str) -> tuple[int, int, int, int]:
        height, width = frame_shape[:2]
        if corner == "bottom_right":
            crop_width = max(120, int(width * self.right_threshold_width_ratio))
            crop_height = max(42, int(height * self.right_threshold_height_ratio))
            side_margin = int(min(width, height) * self.right_threshold_side_margin_ratio)
            bottom_margin = int(min(width, height) * self.right_threshold_bottom_margin_ratio)
            x2 = min(width, width - side_margin)
            x1 = max(0, x2 - crop_width)
            y2 = min(height, height - bottom_margin)
            y1 = max(0, y2 - crop_height)
        else:
            crop_width = max(80, int(width * self.threshold_width_ratio))
            crop_height = max(28, int(height * self.threshold_height_ratio))
            margin = int(min(width, height) * self.threshold_margin_ratio)
            x1 = max(0, margin)
            x2 = min(width, crop_width + margin)
            y1 = max(0, height - crop_height - margin)
            y2 = min(height, height - margin)
        return x1, y1, x2, y2

    def extract_overlay(self, frame: np.ndarray, corner: str = "bottom_left") -> np.ndarray:
        x1, y1, x2, y2 = self.get_overlay_bounds(frame.shape, corner)
        return frame[y1:y2, x1:x2].copy()

    def read_timestamp_candidates(self, frame: np.ndarray) -> list[dict]:
        candidates: list[dict] = []
        for corner in self.overlay_corners:
            crop = self.extract_overlay(frame, corner)
            text, confidence = self._read_timestamp_from_crop(crop, corner)
            candidates.append(
                {
                    "corner": corner,
                    "text": text,
                    "confidence": confidence,
                }
            )
        return candidates

    def read_timestamp(self, frame: np.ndarray) -> tuple[str, float]:
        best = max(
            self.read_timestamp_candidates(frame),
            key=lambda item: (bool(item["text"]), item["confidence"], len(item["text"])),
            default={"text": "", "confidence": 0.0},
        )
        return best["text"], best["confidence"]

    def _read_timestamp_from_crop(self, crop: np.ndarray, corner: str) -> tuple[str, float]:
        focused_crop = self._focus_time_region(crop, corner)
        chars = self._segment_characters(focused_crop, corner)
        if not chars:
            return "", 0.0

        recognized: list[str] = []
        confidences: list[float] = []
        for char_image in chars:
            normalized = self._normalize_char(char_image)
            char, score = self._classify_char(normalized)
            if char:
                recognized.append(char)
                confidences.append(score)

        text = "".join(recognized).strip()
        confidence = float(np.mean(confidences)) if confidences else 0.0
        text = clean_timestamp_text(text)
        return text, confidence

    def _focus_time_region(self, crop: np.ndarray, corner: str) -> np.ndarray:
        if corner != "bottom_right":
            return crop
        height, width = crop.shape[:2]
        x1 = max(0, int(width * 0.48))
        y1 = max(0, int(height * 0.08))
        return crop[y1:height, x1:width].copy()

    def _segment_characters(self, crop: np.ndarray, corner: str) -> list[np.ndarray]:
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        if corner == "bottom_right":
            enlarged = cv2.resize(gray, None, fx=4.0, fy=4.0, interpolation=cv2.INTER_CUBIC)
            blurred = cv2.GaussianBlur(enlarged, (3, 3), 0)
            bright_binary = cv2.threshold(blurred, 185, 255, cv2.THRESH_BINARY)[1]
            top_hat = cv2.morphologyEx(blurred, cv2.MORPH_TOPHAT, np.ones((7, 7), dtype=np.uint8))
            hat_binary = cv2.threshold(top_hat, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
            adaptive_binary = cv2.adaptiveThreshold(
                blurred,
                255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY,
                31,
                -6,
            )
            candidates = [bright_binary, hat_binary, adaptive_binary]
        else:
            enlarged = cv2.resize(gray, None, fx=2.6, fy=2.6, interpolation=cv2.INTER_CUBIC)
            blurred = cv2.GaussianBlur(enlarged, (3, 3), 0)
            candidates = [cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]]

        best_boxes: list[tuple[int, int, int, int]] = []
        best_binary: np.ndarray | None = None
        for binary in candidates:
            if np.mean(binary) > 127:
                binary = cv2.bitwise_not(binary)
            kernel = np.ones((2, 2), dtype=np.uint8)
            binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=1)
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            boxes = self._collect_character_boxes(binary, contours, corner)
            if len(boxes) > len(best_boxes):
                best_boxes = boxes
                best_binary = binary

        if not best_boxes or best_binary is None:
            return []

        segmented: list[np.ndarray] = []
        for x, y, w, h in best_boxes:
            segmented.append(best_binary[y : y + h, x : x + w])
        return segmented

    def _collect_character_boxes(
        self,
        binary: np.ndarray,
        contours: list[np.ndarray],
        corner: str,
    ) -> list[tuple[int, int, int, int]]:
        boxes: list[tuple[int, int, int, int]] = []
        min_h = max(8, int(binary.shape[0] * (0.08 if corner == "bottom_right" else 0.28)))
        min_w = 2 if corner == "bottom_right" else 3
        max_h = int(binary.shape[0] * 0.92)
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if h < min_h or w < min_w or h > max_h:
                continue
            boxes.append((x, y, w, h))
        boxes.sort(key=lambda item: item[0])
        return boxes

    def _normalize_char(self, image: np.ndarray) -> np.ndarray:
        if image.ndim == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, image = cv2.threshold(image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        if np.mean(image) > 127:
            image = cv2.bitwise_not(image)

        coords = cv2.findNonZero(image)
        width, height = self.canvas_size
        canvas = np.zeros((height, width), dtype=np.uint8)
        if coords is None:
            return canvas

        x, y, w, h = cv2.boundingRect(coords)
        roi = image[y : y + h, x : x + w]
        scale = min((width - 6) / max(1, w), (height - 6) / max(1, h))
        resized = cv2.resize(roi, (max(1, int(round(w * scale))), max(1, int(round(h * scale)))), interpolation=cv2.INTER_AREA)
        off_x = (width - resized.shape[1]) // 2
        off_y = (height - resized.shape[0]) // 2
        canvas[off_y : off_y + resized.shape[0], off_x : off_x + resized.shape[1]] = resized
        return canvas

    def _classify_char(self, char_image: np.ndarray) -> tuple[str, float]:
        best_char = ""
        best_score = -1.0
        for char, templates in self.char_templates.items():
            for template in templates:
                score = compare_binary_images(char_image, template)
                if score > best_score:
                    best_char = char
                    best_score = score
        if best_score < 0.35:
            return "", 0.0
        return best_char, best_score


class VideoTimestampReader:
    def __init__(self) -> None:
        self.ocr = GlyphTemplateOCR()
        self.previous_timestamp: datetime | None = None
        self.previous_elapsed_seconds: float = 0.0
        self.last_text = ""
        self.confidence = 0.0

    def reset(self) -> None:
        self.previous_timestamp = None
        self.previous_elapsed_seconds = 0.0
        self.last_text = ""
        self.confidence = 0.0

    def has_absolute_anchor(self) -> bool:
        return self.previous_timestamp is not None

    def read_candidates(self, frame: np.ndarray, reference_timestamp: datetime | None = None) -> list[dict]:
        reference = reference_timestamp if reference_timestamp is not None else self.previous_timestamp
        parsed_candidates: list[dict] = []
        for candidate in self.ocr.read_timestamp_candidates(frame):
            parsed = parse_timestamp_text(candidate["text"], reference)
            if parsed is None:
                continue
            parsed_candidates.append(
                {
                    "parsed": parsed,
                    "text": candidate["text"],
                    "confidence": candidate["confidence"],
                    "corner": candidate["corner"],
                }
            )
        return parsed_candidates

    def read(self, frame: np.ndarray, elapsed_seconds: float) -> tuple[datetime | None, str, float]:
        candidates = self.ocr.read_timestamp_candidates(frame)
        parsed_candidates = self.read_candidates(frame)
        expected_timestamp = None
        if self.previous_timestamp is not None:
            delta = elapsed_seconds - self.previous_elapsed_seconds
            expected_timestamp = self.previous_timestamp + timedelta(seconds=delta)
            tolerance_seconds = max(20.0, abs(delta) * 2.0 + 10.0)
            consistent_candidates = [
                candidate
                for candidate in parsed_candidates
                if abs((candidate["parsed"] - expected_timestamp).total_seconds()) <= tolerance_seconds
            ]
            if consistent_candidates:
                parsed_candidates = consistent_candidates
            else:
                parsed_candidates = []

        if parsed_candidates:
            best_candidate = max(
                parsed_candidates,
                key=lambda item: (item["confidence"], len(item["text"])),
            )
            parsed = best_candidate["parsed"]
            confidence = best_candidate["confidence"]
            self.previous_timestamp = parsed
            self.previous_elapsed_seconds = elapsed_seconds
            self.last_text = format_display_timestamp(parsed)
            self.confidence = confidence
            return parsed, self.last_text, confidence

        best_candidate = max(
            candidates,
            key=lambda item: (bool(item["text"]), item["confidence"], len(item["text"])),
            default={"text": "", "confidence": 0.0},
        )
        text = best_candidate["text"]
        confidence = best_candidate["confidence"]

        if expected_timestamp is not None:
            estimated = expected_timestamp
            return estimated, format_display_timestamp(estimated), max(0.1, self.confidence * 0.8)

        return None, text, confidence


class PersonCounterApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("보행 통행량 카운터 - Python Desktop")
        self.geometry("1560x960")
        self.minsize(1320, 780)
        self.configure(bg="#ece7db")
        self.base_dir = Path(__file__).resolve().parent
        os.chdir(self.base_dir)

        self.input_folder: Path | None = None
        self.available_video_files: list[Path] = []
        self.video_files: list[Path] = []
        self.preview_video_path: Path | None = None
        self.preview_frame: np.ndarray | None = None
        self.preview_photo: ImageTk.PhotoImage | None = None
        self.frame_size = (0, 0)
        self.display_box = (0, 0, 1.0)
        self.line_start_frame: tuple[int, int] | None = None
        self.line_end_frame: tuple[int, int] | None = None
        self.drag_line_end_frame: tuple[int, int] | None = None
        self.normalized_line: tuple[tuple[float, float], tuple[float, float]] | None = None
        self.hourly_counts: list[dict] = []
        self.events: list[dict] = []
        self.debug_log_path: Path | None = None
        self.processing_thread: threading.Thread | None = None
        self.processing_queue: queue.Queue = queue.Queue()
        self.processing_active = False
        self.models_dir = Path("models")
        self.yolox_detector = None
        self.person_detector = None
        self.detector_runtime_name = "OpenCV DNN"
        self.dnn_backend_id = getattr(cv2.dnn, "DNN_BACKEND_OPENCV", 3)
        self.dnn_target_id = getattr(cv2.dnn, "DNN_TARGET_CPU", 0)
        self.dnn_device_label = "CPU"
        self.category_enabled_vars = {category: tk.BooleanVar(value=True) for category in CATEGORY_ORDER}
        self.active_categories = tuple(CATEGORY_ORDER)
        self.category_checkbuttons: list[ttk.Checkbutton] = []

        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        self.folder_label_var = tk.StringVar(value="선택된 폴더가 없습니다.")
        self.preview_video_var = tk.StringVar(value="미리보기 영상이 없습니다.")
        self.status_var = tk.StringVar(value="영상 폴더를 선택해 주세요.")
        self.progress_var = tk.StringVar(value="0%")
        self.line_var = tk.StringVar(value="카운팅 선이 아직 지정되지 않았습니다.")
        self.total_in_var = tk.StringVar(value="0")
        self.total_out_var = tk.StringVar(value="0")
        self.person_total_var = tk.StringVar(value="0")
        self.bicycle_total_var = tk.StringVar(value="0")
        self.motorcycle_total_var = tk.StringVar(value="0")
        self.car_total_var = tk.StringVar(value="0")
        self.truck_total_var = tk.StringVar(value="0")
        self.duration_var = tk.StringVar(value="0분")
        self.detector_status_var = tk.StringVar(value="감지기 준비 중")
        self.ocr_status_var = tk.StringVar(value="OCR 준비 중")

        self.sample_interval_var = tk.DoubleVar(value=0.2)
        self.match_distance_var = tk.IntVar(value=70)
        self.cooldown_var = tk.IntVar(value=8)
        self.score_threshold_var = tk.DoubleVar(value=0.30)
        self.hog_scale_var = tk.DoubleVar(value=0.6)
        self.speed_mode_var = tk.StringVar(value="1x")
        self.speed_accuracy_var = tk.StringVar(value="정확도: 매우 높음")
        self.sample_interval_var.trace_add("write", lambda *_args: self._update_speed_accuracy_label())

        self._build_ui()
        self._update_speed_accuracy_label()
        self._set_video_selection_controls_state("disabled")
        self._initialize_detector()
        self._bind_events()
        self.after(120, self._poll_processing_queue)

    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=16)
        root.pack(fill="both", expand=True)

        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("TFrame", background="#ece7db")
        style.configure("Card.TFrame", background="#faf7f0")
        style.configure("TLabel", background="#ece7db", foreground="#22180f")
        style.configure("Card.TLabel", background="#faf7f0", foreground="#22180f")
        style.configure("Header.TLabel", background="#ece7db", font=("Malgun Gothic", 18, "bold"))
        style.configure("Muted.TLabel", background="#ece7db", foreground="#5f554c")
        style.configure("CardTitle.TLabel", background="#faf7f0", font=("Malgun Gothic", 11, "bold"))
        style.configure("HeaderInfo.TLabel", background="#faf7f0", foreground="#6d6257", font=("Malgun Gothic", 10, "bold"))
        style.configure("HeaderValue.TLabel", background="#faf7f0", foreground="#22180f", font=("Malgun Gothic", 10))

        header = ttk.Frame(root)
        header.pack(fill="x", pady=(0, 12))
        ttk.Label(header, text="폴더 일괄 보행량 집계 데스크톱 앱", style="Header.TLabel").pack(anchor="w")
        ttk.Label(
            header,
            text="폴더 안 모든 영상을 읽고, 화면 하단 시간 오버레이를 기준으로 시간대별 IN / OUT 집계를 만듭니다.",
            style="Muted.TLabel",
        ).pack(anchor="w", pady=(4, 0))

        header_status = ttk.Frame(header, style="Card.TFrame", padding=(12, 10))
        header_status.pack(fill="x", pady=(10, 0))
        header_status.columnconfigure(1, weight=1)
        header_status.columnconfigure(3, weight=1)
        ttk.Label(header_status, text="감지기", style="HeaderInfo.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(
            header_status,
            textvariable=self.detector_status_var,
            style="HeaderValue.TLabel",
            wraplength=520,
        ).grid(row=0, column=1, sticky="we", padx=(8, 18))
        ttk.Label(header_status, text="시간 OCR", style="HeaderInfo.TLabel").grid(row=0, column=2, sticky="w")
        ttk.Label(
            header_status,
            textvariable=self.ocr_status_var,
            style="HeaderValue.TLabel",
            wraplength=420,
        ).grid(row=0, column=3, sticky="we", padx=(8, 0))

        ttk.Label(
            root,
            text="패널 사이 경계선을 드래그해서 각 영역 크기를 조절할 수 있습니다.",
            style="Muted.TLabel",
        ).pack(anchor="w", pady=(0, 8))

        content = tk.PanedWindow(
            root,
            orient=tk.HORIZONTAL,
            sashrelief=tk.RAISED,
            sashcursor="sb_h_double_arrow",
            sashwidth=10,
            sashpad=4,
            showhandle=False,
            opaqueresize=True,
            bg="#d8d1c2",
            bd=0,
        )
        content.pack(fill="both", expand=True)

        self.left_panel_container = ttk.Frame(content, style="Card.TFrame", width=380)
        self.center_panel = ttk.Frame(content, style="Card.TFrame", padding=14, width=920)
        self.right_panel = ttk.Frame(content, style="Card.TFrame", padding=14, width=460)

        self.left_panel_canvas = tk.Canvas(
            self.left_panel_container,
            bg="#faf7f0",
            bd=0,
            highlightthickness=0,
            relief="flat",
        )
        self.left_panel_scrollbar = ttk.Scrollbar(
            self.left_panel_container,
            orient="vertical",
            command=self.left_panel_canvas.yview,
        )
        self.left_panel_canvas.configure(yscrollcommand=self.left_panel_scrollbar.set)
        self.left_panel_scrollbar.pack(side="right", fill="y")
        self.left_panel_canvas.pack(side="left", fill="both", expand=True)
        self.left_panel = ttk.Frame(self.left_panel_canvas, style="Card.TFrame", padding=14)
        self.left_panel_window = self.left_panel_canvas.create_window((0, 0), window=self.left_panel, anchor="nw")
        self.left_panel.bind("<Configure>", self._update_left_panel_scrollregion)
        self.left_panel_canvas.bind("<Configure>", self._resize_left_panel_inner)

        content.add(self.left_panel_container, minsize=320, padx=0, pady=0)
        content.add(self.center_panel, minsize=560, padx=0, pady=0)
        content.add(self.right_panel, minsize=360, padx=0, pady=0)

        self._build_controls()
        self._build_viewer()
        self._build_results()

    def _build_controls(self) -> None:
        ttk.Label(self.left_panel, text="입력과 옵션", style="CardTitle.TLabel").pack(anchor="w")
        self.open_folder_button = ttk.Button(self.left_panel, text="영상 폴더 선택", command=self.open_folder)
        self.open_folder_button.pack(fill="x", pady=(12, 6))
        ttk.Label(self.left_panel, textvariable=self.folder_label_var, style="Card.TLabel", wraplength=320).pack(anchor="w")
        ttk.Label(self.left_panel, textvariable=self.preview_video_var, style="Card.TLabel", wraplength=320).pack(anchor="w", pady=(8, 0))

        ttk.Label(self.left_panel, text="분석할 영상 선택", style="CardTitle.TLabel").pack(anchor="w", pady=(14, 0))
        video_select_buttons = ttk.Frame(self.left_panel, style="Card.TFrame")
        video_select_buttons.pack(fill="x", pady=(8, 6))
        self.select_all_videos_button = ttk.Button(video_select_buttons, text="전체 선택", command=self.select_all_videos)
        self.select_all_videos_button.pack(side="left", fill="x", expand=True)
        self.clear_video_selection_button = ttk.Button(video_select_buttons, text="선택 해제", command=self.clear_video_selection)
        self.clear_video_selection_button.pack(side="left", fill="x", expand=True, padx=(8, 0))

        video_list_frame = ttk.Frame(self.left_panel, style="Card.TFrame")
        video_list_frame.pack(fill="x", pady=(0, 12))
        self.video_listbox = tk.Listbox(
            video_list_frame,
            height=8,
            exportselection=False,
            selectmode=tk.EXTENDED,
            font=("Consolas", 10),
        )
        self.video_listbox.pack(side="left", fill="both", expand=True)
        video_list_scrollbar = ttk.Scrollbar(video_list_frame, orient="vertical", command=self.video_listbox.yview)
        video_list_scrollbar.pack(side="right", fill="y")
        self.video_listbox.configure(yscrollcommand=video_list_scrollbar.set)

        ttk.Label(self.left_panel, text="카운팅 대상 선택", style="CardTitle.TLabel").pack(anchor="w", pady=(8, 0))
        category_box = ttk.Frame(self.left_panel, style="Card.TFrame")
        category_box.pack(fill="x", pady=(8, 12))
        for index, category in enumerate(CATEGORY_ORDER):
            check = ttk.Checkbutton(
                category_box,
                text=CATEGORY_LABELS.get(category, category),
                variable=self.category_enabled_vars[category],
                command=self.on_category_filter_change,
            )
            check.grid(row=index // 2, column=index % 2, sticky="w", padx=(0, 12), pady=(0, 6))
            self.category_checkbuttons.append(check)

        form = ttk.Frame(self.left_panel, style="Card.TFrame")
        form.pack(fill="x", pady=(18, 12))
        speed_row = ttk.Frame(form, style="Card.TFrame")
        speed_row.pack(fill="x", pady=(0, 10))
        ttk.Label(speed_row, text="분석 속도", style="Card.TLabel").pack(anchor="w")
        self.speed_combo = ttk.Combobox(
            speed_row,
            textvariable=self.speed_mode_var,
            values=("1x", "1.5x", "2x", "3x"),
            state="readonly",
        )
        self.speed_combo.pack(fill="x", pady=(4, 0))
        ttk.Label(speed_row, textvariable=self.speed_accuracy_var, style="Card.TLabel", wraplength=320).pack(anchor="w", pady=(4, 0))
        self._make_labeled_entry(form, "샘플링 간격(초)", self.sample_interval_var)
        self._make_labeled_entry(form, "트랙 연결 거리(px)", self.match_distance_var)
        self._make_labeled_entry(form, "재카운트 방지 시간(초)", self.cooldown_var)
        self._make_labeled_entry(form, "감지 신뢰도(0.1~0.95)", self.score_threshold_var)
        self._make_labeled_entry(form, "HOG 보조 축소 비율(0.3~1.0)", self.hog_scale_var)

        buttons = ttk.Frame(self.left_panel, style="Card.TFrame")
        buttons.pack(fill="x", pady=(8, 8))
        self.process_button = ttk.Button(buttons, text="폴더 분석 시작", command=self.start_processing)
        self.process_button.pack(fill="x")
        self.export_button = ttk.Button(buttons, text="결과 저장", command=self.save_csv)
        self.export_button.pack(fill="x", pady=(8, 0))
        self.export_button.configure(state="disabled")
        self.clear_line_button = ttk.Button(buttons, text="카운팅 선 초기화", command=self.clear_line)
        self.clear_line_button.pack(fill="x", pady=(8, 0))

        info = ttk.Frame(self.left_panel, style="Card.TFrame")
        info.pack(fill="x", pady=(12, 0))
        self._make_metric(info, "상태", self.status_var)
        self._make_metric(info, "감지기", self.detector_status_var)
        self._make_metric(info, "시간 OCR", self.ocr_status_var)
        self._make_metric(info, "진행률", self.progress_var)
        self._make_metric(info, "총 IN", self.total_in_var)
        self._make_metric(info, "총 OUT", self.total_out_var)
        self._make_metric(info, "보행자 합계", self.person_total_var)
        self._make_metric(info, "자전거/킥보드 합계", self.bicycle_total_var)
        self._make_metric(info, "오토바이 합계", self.motorcycle_total_var)
        self._make_metric(info, "승용차 합계", self.car_total_var)
        self._make_metric(info, "트럭 합계", self.truck_total_var)
        self._make_metric(info, "전체 길이", self.duration_var)

    def _build_viewer(self) -> None:
        title = ttk.Frame(self.center_panel, style="Card.TFrame")
        title.pack(fill="x")
        ttk.Label(title, text="카운팅 선 지정", style="CardTitle.TLabel").pack(anchor="w")
        ttk.Label(
            title,
            text=(
                "영상 위에서 마우스로 직접 드래그해 카운팅 선을 그리세요. "
                "첫 클릭이 시작점, 드래그 끝점이 종료점입니다. "
                "선의 시작점에서 끝점 방향으로 볼 때 오른쪽에서 왼쪽으로 넘어가면 IN, "
                "왼쪽에서 오른쪽으로 넘어가면 OUT으로 계산합니다."
            ),
            style="Card.TLabel",
            wraplength=820,
        ).pack(anchor="w", pady=(4, 0))

        self.canvas = tk.Canvas(
            self.center_panel,
            width=960,
            height=600,
            bg="#0f0b08",
            highlightthickness=0,
            cursor="crosshair",
        )
        self.canvas.pack(fill="both", expand=True, pady=(12, 8))
        ttk.Label(self.center_panel, textvariable=self.line_var, style="Card.TLabel", wraplength=900).pack(anchor="w")

    def _build_results(self) -> None:
        ttk.Label(self.right_panel, text="집계 결과", style="CardTitle.TLabel").pack(anchor="w")
        results_frame = ttk.Frame(self.right_panel, style="Card.TFrame")
        results_frame.pack(fill="x", pady=(12, 14))
        self.results_tree = ttk.Treeview(
            results_frame,
            columns=("hour", "total"),
            show="headings",
            height=12,
        )
        self.results_tree.pack(side="top", fill="x")
        results_x_scroll = ttk.Scrollbar(results_frame, orient="horizontal", command=self.results_tree.xview)
        results_x_scroll.pack(side="bottom", fill="x", pady=(6, 0))
        self.results_tree.configure(xscrollcommand=results_x_scroll.set)
        self._refresh_results_columns()

        ttk.Label(self.right_panel, text="이벤트 로그", style="CardTitle.TLabel").pack(anchor="w")
        self.event_listbox = tk.Listbox(self.right_panel, height=22, font=("Consolas", 10))
        self.event_listbox.pack(fill="both", expand=True, pady=(12, 0))

    def _bind_events(self) -> None:
        self.canvas.bind("<ButtonPress-1>", self.begin_line_drag)
        self.canvas.bind("<B1-Motion>", self.update_line_drag)
        self.canvas.bind("<ButtonRelease-1>", self.finish_line_drag)
        self.canvas.bind("<Configure>", lambda _event: self.refresh_preview())
        self.video_listbox.bind("<<ListboxSelect>>", self.on_video_selection_change)
        self.speed_combo.bind("<<ComboboxSelected>>", self.on_speed_mode_change)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def _make_labeled_entry(self, parent: ttk.Frame, label: str, variable: tk.Variable) -> None:
        row = ttk.Frame(parent, style="Card.TFrame")
        row.pack(fill="x", pady=(0, 10))
        ttk.Label(row, text=label, style="Card.TLabel").pack(anchor="w")
        ttk.Entry(row, textvariable=variable).pack(fill="x", pady=(4, 0))

    def _make_metric(self, parent: ttk.Frame, label: str, variable: tk.StringVar) -> None:
        card = ttk.Frame(parent, style="Card.TFrame")
        card.pack(fill="x", pady=(0, 8))
        ttk.Label(card, text=label, style="Card.TLabel").pack(anchor="w")
        ttk.Label(card, textvariable=variable, style="Card.TLabel", wraplength=320).pack(anchor="w", pady=(2, 0))

    def _update_left_panel_scrollregion(self, _event: tk.Event | None = None) -> None:
        self.left_panel_canvas.configure(scrollregion=self.left_panel_canvas.bbox("all"))

    def _resize_left_panel_inner(self, event: tk.Event) -> None:
        self.left_panel_canvas.itemconfigure(self.left_panel_window, width=event.width)

    def _get_selected_categories(self) -> list[str]:
        return [category for category in CATEGORY_ORDER if self.category_enabled_vars[category].get()]

    def _ensure_valid_selected_categories(self) -> list[str]:
        selected_categories = self._get_selected_categories()
        if selected_categories:
            return selected_categories
        self.category_enabled_vars["person"].set(True)
        return ["person"]

    def _refresh_results_columns(self) -> None:
        visible_categories = tuple(self.active_categories)
        columns = ["hour"]
        for category in visible_categories:
            columns.extend([f"{category}_in", f"{category}_out"])
        columns.append("total")
        self.results_tree.configure(columns=tuple(columns))
        self.results_tree.heading("hour", text="시간 구간")
        self.results_tree.column("hour", width=220, anchor="w")
        for category in visible_categories:
            short_label = CATEGORY_TABLE_LABELS.get(category, category)
            self.results_tree.heading(f"{category}_in", text=f"{short_label} IN")
            self.results_tree.heading(f"{category}_out", text=f"{short_label} OUT")
            self.results_tree.column(f"{category}_in", width=84, anchor="center")
            self.results_tree.column(f"{category}_out", width=84, anchor="center")
        self.results_tree.heading("total", text="합계")
        self.results_tree.column("total", width=76, anchor="center")

    def on_category_filter_change(self) -> None:
        selected_categories = self._ensure_valid_selected_categories()
        if self.processing_active:
            return
        self.active_categories = tuple(selected_categories)
        self._refresh_results_columns()
        self.populate_results(self.hourly_counts)
        self._update_total_metrics(self.hourly_counts)

    def _update_total_metrics(self, rows: list[dict]) -> None:
        in_total, out_total, category_totals = summarize_direction_counts(rows, self.active_categories)
        self.total_in_var.set(str(in_total))
        self.total_out_var.set(str(out_total))
        self.person_total_var.set(str(category_totals.get("person", 0)))
        self.bicycle_total_var.set(str(category_totals.get("bicycle", 0)))
        self.motorcycle_total_var.set(str(category_totals.get("motorcycle", 0)))
        self.car_total_var.set(str(category_totals.get("car", 0)))
        self.truck_total_var.set(str(category_totals.get("truck", 0)))

    def _get_speed_multiplier(self) -> float:
        mapping = {
            "1x": 1.0,
            "1.5x": 1.5,
            "2x": 2.0,
            "3x": 3.0,
        }
        return mapping.get(self.speed_mode_var.get(), 1.0)

    def _update_speed_accuracy_label(self) -> None:
        multiplier = self._get_speed_multiplier()
        try:
            base_interval = max(0.1, float(self.sample_interval_var.get()))
        except Exception:
            base_interval = 0.2
        effective_interval = base_interval * multiplier
        if multiplier <= 1.0:
            accuracy = "매우 높음"
        elif multiplier <= 1.5:
            accuracy = "높음"
        elif multiplier <= 2.0:
            accuracy = "보통, 겹침 장면 누락 가능"
        else:
            accuracy = "낮아질 수 있음, 혼잡 장면 비추천"
        self.speed_accuracy_var.set(f"예상 정확도: {accuracy} | 유효 샘플링 {effective_interval:.2f}초")

    def on_speed_mode_change(self, _event: tk.Event | None = None) -> None:
        self._update_speed_accuracy_label()

    def _set_video_selection_controls_state(self, state: str) -> None:
        self.video_listbox.configure(state=state)
        self.select_all_videos_button.configure(state=state)
        self.clear_video_selection_button.configure(state=state)

    def _set_category_controls_state(self, state: str) -> None:
        for checkbutton in self.category_checkbuttons:
            checkbutton.configure(state=state)

    def _populate_video_list(self, files: list[Path]) -> None:
        self.video_listbox.delete(0, tk.END)
        for file in files:
            self.video_listbox.insert(tk.END, file.name)

    def _get_selected_video_indices(self) -> list[int]:
        return [int(index) for index in self.video_listbox.curselection()]

    def _apply_video_selection(self) -> None:
        selected_indices = self._get_selected_video_indices()
        self.video_files = [self.available_video_files[index] for index in selected_indices]

        selected_count = len(self.video_files)
        total_count = len(self.available_video_files)
        if self.input_folder is not None:
            self.folder_label_var.set(f"{self.input_folder} ({selected_count}/{total_count}개 선택)")
        else:
            self.folder_label_var.set(f"{selected_count}/{total_count}개 선택")

        if not self.video_files:
            self.preview_video_path = None
            self.preview_frame = None
            self.preview_photo = None
            self.frame_size = (0, 0)
            self.preview_video_var.set("미리보기 영상이 없습니다.")
            self.duration_var.set("0분")
            self.canvas.delete("all")
            self.set_status("분석할 영상을 하나 이상 선택해 주세요.")
            return

        preview_target = self.video_files[0]
        if self.preview_video_path != preview_target:
            preview_info = self._load_preview_frame(preview_target)
            if preview_info is not None:
                frame, _first_duration = preview_info
                self.preview_video_path = preview_target
                self.preview_frame = frame
                self.frame_size = (frame.shape[1], frame.shape[0])
                self.refresh_preview()
        self.preview_video_var.set(f"미리보기: {preview_target.name}")
        total_duration = sum(self._get_video_duration(file) for file in self.video_files)
        self.duration_var.set(format_duration(total_duration))

    def select_all_videos(self) -> None:
        if not self.available_video_files or self.processing_active:
            return
        self.video_listbox.selection_set(0, tk.END)
        self._apply_video_selection()

    def clear_video_selection(self) -> None:
        if self.processing_active:
            return
        self.video_listbox.selection_clear(0, tk.END)
        self._apply_video_selection()

    def on_video_selection_change(self, _event: tk.Event | None = None) -> None:
        if self.processing_active:
            return
        self._apply_video_selection()

    def open_folder(self) -> None:
        if self.processing_active:
            return

        path = filedialog.askdirectory(title="영상 폴더 선택")
        if not path:
            return

        folder = Path(path)
        files = sorted([file for file in folder.iterdir() if file.suffix.lower() in VIDEO_EXTENSIONS])
        if not files:
            messagebox.showinfo("영상 없음", "선택한 폴더에 읽을 수 있는 영상 파일이 없습니다.")
            return

        preview_info = self._load_preview_frame(files[0])
        if preview_info is None:
            messagebox.showerror("영상 읽기 실패", "첫 번째 영상의 프레임을 읽지 못했습니다.")
            return

        frame, _first_duration = preview_info
        self.input_folder = folder
        self.available_video_files = files
        self.video_files = files.copy()
        self.preview_video_path = files[0]
        self.preview_frame = frame
        self.frame_size = (frame.shape[1], frame.shape[0])
        self.normalized_line = None
        self.line_start_frame = None
        self.line_end_frame = None
        self.drag_line_end_frame = None
        self.hourly_counts = convert_counts_to_rows(build_empty_hour_counts())
        self.events = []
        self.total_in_var.set("0")
        self.total_out_var.set("0")
        self.person_total_var.set("0")
        self.bicycle_total_var.set("0")
        self.motorcycle_total_var.set("0")
        self.car_total_var.set("0")
        self.truck_total_var.set("0")
        self.debug_log_path = None
        self._set_video_selection_controls_state("normal")
        self._populate_video_list(files)
        self.video_listbox.selection_set(0, tk.END)
        self._apply_video_selection()
        self.populate_results(self.hourly_counts)
        self.populate_events([])
        self.export_button.configure(state="disabled")
        self.update_line_label()
        self.set_status("첫 영상 프레임이 준비되었습니다. 카운팅 선을 지정해 주세요.")
        self.ocr_status_var.set("하단 시간 OCR 대기")
        self.refresh_preview()

    def _load_preview_frame(self, video_path: Path) -> tuple[np.ndarray, float] | None:
        capture = cv2.VideoCapture(str(video_path))
        ok, frame = capture.read()
        fps = capture.get(cv2.CAP_PROP_FPS) or 0.0
        frame_count = capture.get(cv2.CAP_PROP_FRAME_COUNT) or 0.0
        capture.release()
        if not ok or frame is None:
            return None
        return frame, (frame_count / fps if fps > 0 else 0.0)

    def _get_video_duration(self, video_path: Path) -> float:
        capture = cv2.VideoCapture(str(video_path))
        fps = capture.get(cv2.CAP_PROP_FPS) or 0.0
        frame_count = capture.get(cv2.CAP_PROP_FRAME_COUNT) or 0.0
        capture.release()
        return frame_count / fps if fps > 0 else 0.0

    def _prime_timestamp_reader(
        self,
        capture: cv2.VideoCapture,
        first_frame: np.ndarray,
        sample_times: list[float],
        timestamp_reader: VideoTimestampReader,
    ) -> tuple[str, float, list[dict]]:
        debug_rows: list[dict] = []
        self._write_ocr_debug_crops(first_frame)
        if not sample_times:
            timestamp_reader.read(first_frame, 0.0)
            debug_rows.append(
                {
                    "row_type": "anchor",
                    "video_name": "",
                    "elapsed_seconds": 0.0,
                    "ocr_text": timestamp_reader.last_text,
                    "ocr_conf": timestamp_reader.confidence,
                    "resolved_timestamp": timestamp_reader.last_text,
                    "bucket_key": "",
                    "event_category": "",
                    "event_direction": "",
                    "anchor_note": "single-frame-prime",
                }
            )
            return timestamp_reader.last_text, timestamp_reader.confidence, debug_rows

        max_probe_count = min(len(sample_times), 180)
        anchor_candidates: list[dict] = []
        for sample_index in range(max_probe_count):
            elapsed_seconds = sample_times[sample_index]
            if sample_index == 0:
                frame = first_frame
            else:
                capture.set(cv2.CAP_PROP_POS_MSEC, elapsed_seconds * 1000)
                ok, frame = capture.read()
                if not ok or frame is None:
                    continue
            for candidate in timestamp_reader.read_candidates(frame, None):
                anchor_candidates.append(
                    {
                        "elapsed_seconds": elapsed_seconds,
                        "parsed": candidate["parsed"],
                        "text": candidate["text"],
                        "confidence": candidate["confidence"],
                        "anchor_epoch": candidate["parsed"].timestamp() - elapsed_seconds,
                    }
                )

        best_anchor = choose_best_anchor_candidate(anchor_candidates)
        for candidate in anchor_candidates:
            debug_rows.append(
                {
                    "row_type": "anchor_candidate",
                    "video_name": "",
                    "elapsed_seconds": candidate["elapsed_seconds"],
                    "ocr_text": candidate["text"],
                    "ocr_conf": candidate["confidence"],
                    "resolved_timestamp": format_display_timestamp(candidate["parsed"]),
                    "bucket_key": f'hour:{candidate["parsed"].hour:02d}',
                    "event_category": "",
                    "event_direction": "",
                    "anchor_note": "selected" if best_anchor is candidate else "",
                }
            )
        if best_anchor is not None:
            timestamp_reader.previous_timestamp = best_anchor["parsed"]
            timestamp_reader.previous_elapsed_seconds = best_anchor["elapsed_seconds"]
            timestamp_reader.last_text = format_display_timestamp(best_anchor["parsed"])
            timestamp_reader.confidence = best_anchor["confidence"]
            return timestamp_reader.last_text, timestamp_reader.confidence, debug_rows
        return "", 0.0, debug_rows

    def _write_ocr_debug_crops(self, frame: np.ndarray) -> None:
        extractor = GlyphTemplateOCR()
        for corner in extractor.overlay_corners:
            crop = extractor.extract_overlay(frame, corner)
            output_path = self.base_dir / f"ocr_debug_{corner}.png"
            encoded, buffer = cv2.imencode(".png", crop)
            if encoded:
                output_path.write_bytes(buffer.tobytes())

    def _resolve_dnn_runtime(self) -> tuple[int, int, str]:
        backend_id = getattr(cv2.dnn, "DNN_BACKEND_OPENCV", 3)
        target_id = getattr(cv2.dnn, "DNN_TARGET_CPU", 0)
        device_label = "CPU"

        cuda_module = getattr(cv2, "cuda", None)
        if cuda_module is None:
            return backend_id, target_id, device_label

        try:
            cuda_count = int(cuda_module.getCudaEnabledDeviceCount())
        except Exception:
            cuda_count = 0

        if cuda_count <= 0:
            return backend_id, target_id, device_label

        backend_id = getattr(cv2.dnn, "DNN_BACKEND_CUDA", backend_id)
        target_id = getattr(cv2.dnn, "DNN_TARGET_CUDA_FP16", getattr(cv2.dnn, "DNN_TARGET_CUDA", target_id))
        device_label = "GPU(CUDA)"
        return backend_id, target_id, device_label

    def _detector_runtime_suffix(self) -> str:
        return f" / {self.detector_runtime_name} / {self.dnn_device_label}"

    def _sync_detector_runtime(self, detector) -> None:
        self.detector_runtime_name = getattr(detector, "runtime_label", "OpenCV DNN")
        self.dnn_device_label = getattr(detector, "device_label", "CPU")

    def _create_yolox_detector(self, wrapper_path: Path, model_path: Path, backend_id: int, target_id: int):
        spec = importlib.util.spec_from_file_location("yolox_module", wrapper_path)
        if spec is None or spec.loader is None:
            raise RuntimeError("YOLOX 모듈 로드 실패")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module.YoloX(
            str(model_path),
            confThreshold=max(0.10, float(self.score_threshold_var.get()) * 0.5),
            nmsThreshold=0.45,
            objThreshold=0.15,
            backendId=backend_id,
            targetId=target_id,
        )

    def _create_person_detector(self, wrapper_path: Path, model_path: Path, backend_id: int, target_id: int):
        spec = importlib.util.spec_from_file_location("mediapipe_module", wrapper_path)
        if spec is None or spec.loader is None:
            raise RuntimeError("MPPersonDet 모듈 로드 실패")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module.MPPersonDet(
            str(model_path),
            scoreThreshold=max(0.10, float(self.score_threshold_var.get()) * 0.7),
            nmsThreshold=0.3,
            backendId=backend_id,
            targetId=target_id,
        )

    def _initialize_detector(self) -> None:
        yolox_model_path = self.models_dir / "object_detection_yolox_2022nov.onnx"
        yolox_wrapper_path = self.models_dir / "yolox.py"
        mediapipe_model_path = self.models_dir / "person_detection_mediapipe_2023mar.onnx"
        mediapipe_wrapper_path = self.models_dir / "mp_persondet.py"

        self.yolox_detector = None
        self.person_detector = None
        self.dnn_backend_id, self.dnn_target_id, self.dnn_device_label = self._resolve_dnn_runtime()

        if yolox_model_path.exists() and yolox_wrapper_path.exists():
            try:
                self.yolox_detector = self._create_yolox_detector(
                    yolox_wrapper_path,
                    yolox_model_path,
                    self.dnn_backend_id,
                    self.dnn_target_id,
                )
                self._sync_detector_runtime(self.yolox_detector)
                self.detector_status_var.set(f"YOLOX (보행자/자전거/킥보드/오토바이/승용/트럭){self._detector_runtime_suffix()}")
                self.ocr_status_var.set("템플릿 OCR 준비")
                return
            except Exception:
                self.yolox_detector = None
                if self.dnn_device_label != "CPU":
                    self.dnn_backend_id = getattr(cv2.dnn, "DNN_BACKEND_OPENCV", 3)
                    self.dnn_target_id = getattr(cv2.dnn, "DNN_TARGET_CPU", 0)
                    self.dnn_device_label = "CPU"
                    try:
                        self.yolox_detector = self._create_yolox_detector(
                            yolox_wrapper_path,
                            yolox_model_path,
                            self.dnn_backend_id,
                            self.dnn_target_id,
                        )
                        self._sync_detector_runtime(self.yolox_detector)
                        self.detector_status_var.set(f"YOLOX (보행자/자전거/킥보드/오토바이/승용/트럭){self._detector_runtime_suffix()}")
                        self.ocr_status_var.set("템플릿 OCR 준비")
                        return
                    except Exception:
                        self.yolox_detector = None

        if mediapipe_model_path.exists() and mediapipe_wrapper_path.exists():
            try:
                self.person_detector = self._create_person_detector(
                    mediapipe_wrapper_path,
                    mediapipe_model_path,
                    self.dnn_backend_id,
                    self.dnn_target_id,
                )
                self._sync_detector_runtime(self.person_detector)
                self.detector_status_var.set(f"MPPersonDet (보행자만){self._detector_runtime_suffix()}")
                self.ocr_status_var.set("템플릿 OCR 준비")
                return
            except Exception:
                self.person_detector = None
                if self.dnn_device_label != "CPU":
                    self.dnn_backend_id = getattr(cv2.dnn, "DNN_BACKEND_OPENCV", 3)
                    self.dnn_target_id = getattr(cv2.dnn, "DNN_TARGET_CPU", 0)
                    self.dnn_device_label = "CPU"
                    try:
                        self.person_detector = self._create_person_detector(
                            mediapipe_wrapper_path,
                            mediapipe_model_path,
                            self.dnn_backend_id,
                            self.dnn_target_id,
                        )
                        self._sync_detector_runtime(self.person_detector)
                        self.detector_status_var.set(f"MPPersonDet (보행자만){self._detector_runtime_suffix()}")
                        self.ocr_status_var.set("템플릿 OCR 준비")
                        return
                    except Exception:
                        self.person_detector = None

        self.detector_status_var.set(f"기본 HOG fallback (보행자만) / CPU")
        self.ocr_status_var.set("템플릿 OCR 준비")

    def begin_line_drag(self, event: tk.Event) -> None:
        if self.processing_active or self.preview_frame is None:
            return
        point = self.canvas_to_frame(event.x, event.y)
        if point is None:
            return
        self.line_start_frame = point
        self.drag_line_end_frame = point
        self.refresh_preview()

    def update_line_drag(self, event: tk.Event) -> None:
        if self.line_start_frame is None or self.preview_frame is None:
            return
        point = self.canvas_to_frame(event.x, event.y)
        if point is None:
            return
        self.drag_line_end_frame = point
        self.refresh_preview()

    def finish_line_drag(self, event: tk.Event) -> None:
        if self.line_start_frame is None or self.preview_frame is None:
            return

        point = self.canvas_to_frame(event.x, event.y)
        if point is None:
            point = self.drag_line_end_frame if self.drag_line_end_frame is not None else self.line_start_frame

        if point is None:
            return

        if math.hypot(point[0] - self.line_start_frame[0], point[1] - self.line_start_frame[1]) < 20:
            self.set_status("카운팅 선이 너무 짧습니다. 더 길게 그려 주세요.")
            self.line_start_frame = None
            self.drag_line_end_frame = None
            self.refresh_preview()
            return

        width, height = self.frame_size
        self.normalized_line = (
            (self.line_start_frame[0] / width, self.line_start_frame[1] / height),
            (point[0] / width, point[1] / height),
        )
        self.line_start_frame = None
        self.line_end_frame = point
        self.drag_line_end_frame = None
        self.update_line_label()
        self.refresh_preview()
        self.set_status("카운팅 선이 설정되었습니다. 폴더 분석을 시작할 수 있습니다.")

    def clear_line(self) -> None:
        self.normalized_line = None
        self.line_start_frame = None
        self.line_end_frame = None
        self.drag_line_end_frame = None
        self.update_line_label()
        self.refresh_preview()
        self.set_status("카운팅 선을 초기화했습니다.")

    def refresh_preview(self, frame: np.ndarray | None = None, detections: list[dict] | None = None) -> None:
        source = frame if frame is not None else self.preview_frame
        if source is None:
            self.canvas.delete("all")
            return

        preview = source.copy()
        self._draw_timestamp_overlay_box(preview)

        line_to_draw = self._get_current_frame_line()
        if line_to_draw is not None:
            start, end = line_to_draw
            cv2.line(preview, start, end, (29, 200, 186), 3, cv2.LINE_AA)
            cv2.circle(preview, start, 6, (255, 214, 0), -1, cv2.LINE_AA)
            cv2.circle(preview, end, 6, (255, 255, 255), -1, cv2.LINE_AA)
            draw_direction_hint(preview, start, end)
        else:
            guide_lines = [
                "카운팅 선을 그리려면",
                "영상 위를 마우스로 드래그하세요",
            ]
            overlay = preview.copy()
            box_w = min(preview.shape[1] - 40, 420)
            box_h = 96
            box_x = max(20, (preview.shape[1] - box_w) // 2)
            box_y = max(20, preview.shape[0] - box_h - 28)
            cv2.rectangle(overlay, (box_x, box_y), (box_x + box_w, box_y + box_h), (25, 20, 18), -1)
            cv2.addWeighted(overlay, 0.45, preview, 0.55, 0, preview)
            for index, text in enumerate(guide_lines):
                cv2.putText(
                    preview,
                    text,
                    (box_x + 20, box_y + 34 + (index * 28)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (245, 240, 230),
                    2,
                    cv2.LINE_AA,
                )

        if detections:
            for detection in detections:
                x, y, w, h = detection["bbox"]
                color = (0, 214, 255) if detection.get("category") == "person" else (255, 170, 70)
                if detection.get("direction") == "IN":
                    color = (29, 200, 186)
                elif detection.get("direction") == "OUT":
                    color = (100, 180, 255)
                cv2.rectangle(preview, (x, y), (x + w, y + h), color, 2)
                category_label = CATEGORY_LABELS.get(detection.get("category", "person"), detection.get("category", "unknown"))
                direction_suffix = f' {detection["direction"]}' if detection.get("direction") else ""
                label = f'{category_label} {detection.get("track_id", "?")}{direction_suffix}'
                cv2.putText(preview, label, (x, max(18, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

        canvas_width = max(320, self.canvas.winfo_width())
        canvas_height = max(240, self.canvas.winfo_height())
        image_height, image_width = preview.shape[:2]
        scale = min(canvas_width / image_width, canvas_height / image_height)
        draw_width = max(1, int(image_width * scale))
        draw_height = max(1, int(image_height * scale))
        offset_x = (canvas_width - draw_width) // 2
        offset_y = (canvas_height - draw_height) // 2
        self.display_box = (offset_x, offset_y, scale)

        resized = cv2.resize(preview, (draw_width, draw_height), interpolation=cv2.INTER_AREA)
        image = Image.fromarray(cv2.cvtColor(resized, cv2.COLOR_BGR2RGB))
        self.preview_photo = ImageTk.PhotoImage(image=image)
        self.canvas.delete("all")
        self.canvas.create_image(offset_x, offset_y, anchor="nw", image=self.preview_photo)

    def _draw_timestamp_overlay_box(self, frame: np.ndarray) -> None:
        extractor = GlyphTemplateOCR()
        overlay_styles = {
            "bottom_left": ((200, 120, 255), "OCR-L"),
            "bottom_right": ((80, 220, 255), "OCR-R"),
        }
        for corner in extractor.overlay_corners:
            x1, y1, x2, y2 = extractor.get_overlay_bounds(frame.shape, corner)
            color, label = overlay_styles.get(corner, ((200, 120, 255), "OCR"))
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 1, cv2.LINE_AA)
            cv2.putText(frame, label, (x1, max(18, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)

    def _get_current_frame_line(self) -> tuple[tuple[int, int], tuple[int, int]] | None:
        if self.preview_frame is None:
            return None
        if self.line_start_frame is not None and self.drag_line_end_frame is not None:
            return self.line_start_frame, self.drag_line_end_frame
        if self.normalized_line is None:
            return None
        return scale_normalized_line(self.normalized_line, self.frame_size)

    def canvas_to_frame(self, canvas_x: int, canvas_y: int) -> tuple[int, int] | None:
        if self.preview_frame is None:
            return None
        offset_x, offset_y, scale = self.display_box
        frame_x = int((canvas_x - offset_x) / scale)
        frame_y = int((canvas_y - offset_y) / scale)
        width, height = self.frame_size
        if frame_x < 0 or frame_y < 0 or frame_x >= width or frame_y >= height:
            return None
        return frame_x, frame_y

    def start_processing(self) -> None:
        if self.processing_active:
            return
        if not self.available_video_files:
            messagebox.showinfo("폴더 필요", "먼저 영상 폴더를 선택해 주세요.")
            return
        if not self.video_files:
            messagebox.showinfo("영상 선택 필요", "분석할 영상을 하나 이상 선택해 주세요.")
            return
        if self.normalized_line is None:
            messagebox.showinfo("카운팅 선 필요", "먼저 카운팅 선을 지정해 주세요.")
            return

        try:
            sample_interval = max(0.1, float(self.sample_interval_var.get()))
            match_distance = max(20, int(self.match_distance_var.get()))
            cooldown = max(1, int(self.cooldown_var.get()))
            score_threshold = min(0.95, max(0.1, float(self.score_threshold_var.get())))
            hog_scale = min(1.0, max(0.3, float(self.hog_scale_var.get())))
        except Exception:
            messagebox.showerror("입력 오류", "옵션 값이 올바르지 않습니다.")
            return

        selected_categories = self._get_selected_categories()
        if not selected_categories:
            messagebox.showinfo("카운팅 대상 필요", "카운팅할 종류를 하나 이상 선택해 주세요.")
            return

        speed_multiplier = self._get_speed_multiplier()
        effective_sample_interval = sample_interval * speed_multiplier
        self.active_categories = tuple(selected_categories)
        self._refresh_results_columns()

        if self.yolox_detector is not None:
            self.yolox_detector.confThreshold = max(0.10, score_threshold * 0.5)
            self.yolox_detector.objThreshold = 0.15
        if self.person_detector is not None:
            self.person_detector.score_threshold = max(0.10, score_threshold * 0.7)

        self.processing_active = True
        self.process_button.configure(state="disabled")
        self.export_button.configure(state="disabled")
        self.open_folder_button.configure(state="disabled")
        self.clear_line_button.configure(state="disabled")
        self._set_video_selection_controls_state("disabled")
        self._set_category_controls_state("disabled")
        self.speed_combo.configure(state="disabled")
        self.total_in_var.set("0")
        self.total_out_var.set("0")
        self.person_total_var.set("0")
        self.bicycle_total_var.set("0")
        self.motorcycle_total_var.set("0")
        self.car_total_var.set("0")
        self.truck_total_var.set("0")
        self.debug_log_path = None
        self.populate_results(convert_counts_to_rows(build_empty_hour_counts()))
        self.populate_events([])
        self._update_speed_accuracy_label()
        self.set_status(f"폴더 분석을 시작합니다. 속도 {self.speed_mode_var.get()} / 유효 샘플링 {effective_sample_interval:.2f}초")
        self.progress_var.set("0%")

        self.processing_thread = threading.Thread(
            target=self._process_folder_worker,
            args=(effective_sample_interval, match_distance, cooldown, hog_scale, tuple(selected_categories)),
            daemon=True,
        )
        self.processing_thread.start()

    def _process_folder_worker(self, sample_interval: float, match_distance: int, cooldown_seconds: int, hog_scale: float, selected_categories: tuple[str, ...]) -> None:
        aggregate_counts: dict[str, dict] = build_empty_hour_counts()
        all_events: list[dict] = []
        debug_rows: list[dict] = []
        total_duration = 0.0
        processed_frames = 0
        estimated_frames = 0
        per_video_samples: list[list[float]] = []
        last_reported_progress = -1
        progress_emit_every = 3

        for video_path in self.video_files:
            capture = cv2.VideoCapture(str(video_path))
            fps = capture.get(cv2.CAP_PROP_FPS) or 0.0
            frame_count = capture.get(cv2.CAP_PROP_FRAME_COUNT) or 0.0
            duration = frame_count / fps if fps > 0 else 0.0
            total_duration += duration
            sample_times = build_sample_times(duration, sample_interval)
            per_video_samples.append(sample_times)
            estimated_frames += len(sample_times)
            capture.release()

        try:
            for video_index, video_path in enumerate(self.video_files):
                capture = cv2.VideoCapture(str(video_path))
                ok, first_frame = capture.read()
                if not ok or first_frame is None:
                    capture.release()
                    continue

                frame_h, frame_w = first_frame.shape[:2]
                line_start, line_end = scale_normalized_line(self.normalized_line, (frame_w, frame_h))
                tracker = LineCrossingTracker()
                timestamp_reader = VideoTimestampReader()
                filename_anchor = parse_timestamp_from_video_name(video_path.name)
                if filename_anchor is not None:
                    timestamp_reader.previous_timestamp = filename_anchor
                    timestamp_reader.previous_elapsed_seconds = 0.0
                    timestamp_reader.last_text = format_display_timestamp(filename_anchor)
                    timestamp_reader.confidence = 1.0
                sample_times = per_video_samples[video_index]
                if filename_anchor is not None:
                    initial_timestamp_text = format_display_timestamp(filename_anchor)
                    initial_ocr_conf = 1.0
                    prime_debug_rows: list[dict] = []
                else:
                    initial_timestamp_text, initial_ocr_conf, prime_debug_rows = self._prime_timestamp_reader(
                        capture,
                        first_frame,
                        sample_times,
                        timestamp_reader,
                    )
                for row in prime_debug_rows:
                    row["video_name"] = video_path.name
                debug_rows.extend(prime_debug_rows)
                if filename_anchor is not None:
                    debug_rows.append(
                        {
                            "row_type": "filename_anchor",
                            "video_name": video_path.name,
                            "elapsed_seconds": 0.0,
                            "ocr_text": video_path.name,
                            "ocr_conf": 1.0,
                            "resolved_timestamp": format_display_timestamp(filename_anchor),
                            "bucket_key": f'hour:{filename_anchor.hour:02d}',
                            "event_category": "",
                            "event_direction": "",
                            "anchor_note": "filename",
                        }
                    )

                for sample_index, elapsed_seconds in enumerate(sample_times):
                    if sample_index == 0:
                        frame = first_frame
                    else:
                        capture.set(cv2.CAP_PROP_POS_MSEC, elapsed_seconds * 1000)
                        ok, frame = capture.read()
                        if not ok or frame is None:
                            continue

                    detections = self.detect_people(frame, hog_scale, selected_categories)
                    if filename_anchor is not None:
                        timestamp_dt = filename_anchor + timedelta(seconds=elapsed_seconds)
                        timestamp_text = format_display_timestamp(timestamp_dt)
                        ocr_conf = 1.0
                    else:
                        timestamp_dt, timestamp_text, ocr_conf = timestamp_reader.read(frame, elapsed_seconds)
                    sample_bucket_key = get_event_hour_bucket_key(
                        {
                            "timestamp": timestamp_dt,
                            "timestamp_text": timestamp_text,
                            "elapsed_seconds": elapsed_seconds,
                        }
                    )
                    debug_rows.append(
                        {
                            "row_type": "sample",
                            "video_name": video_path.name,
                            "elapsed_seconds": elapsed_seconds,
                            "ocr_text": timestamp_text,
                            "ocr_conf": ocr_conf,
                            "resolved_timestamp": format_display_timestamp(timestamp_dt) if timestamp_dt is not None else "",
                            "bucket_key": sample_bucket_key,
                            "event_category": "",
                            "event_direction": "",
                            "anchor_note": "",
                        }
                    )

                    frame_events = tracker.update(
                        detections=detections,
                        current_time=elapsed_seconds,
                        match_distance=match_distance,
                        cooldown_seconds=cooldown_seconds,
                        line_start=line_start,
                        line_end=line_end,
                        timestamp_value=timestamp_dt,
                        timestamp_text=timestamp_text,
                        video_name=video_path.name,
                    )

                    for event in frame_events:
                        bucket_key = get_event_hour_bucket_key(event)
                        increment_bucket_count(
                            aggregate_counts,
                            bucket_key,
                            event.get("category", "person"),
                            event["direction"],
                        )
                        all_events.append(event)
                        debug_rows.append(
                            {
                                "row_type": "event",
                                "video_name": video_path.name,
                                "elapsed_seconds": event["elapsed_seconds"],
                                "ocr_text": event["timestamp_text"],
                                "ocr_conf": "",
                                "resolved_timestamp": format_display_timestamp(event["timestamp"]) if event["timestamp"] is not None else "",
                                "bucket_key": bucket_key,
                                "event_category": event.get("category", ""),
                                "event_direction": event["direction"],
                                "anchor_note": "",
                            }
                        )

                    processed_frames += 1
                    progress = int((processed_frames / max(1, estimated_frames)) * 100)
                    should_emit_progress = (
                        sample_index == 0
                        or sample_index == len(sample_times) - 1
                        or progress != last_reported_progress
                        or (sample_index + 1) % progress_emit_every == 0
                    )
                    if should_emit_progress:
                        last_reported_progress = progress
                        self.processing_queue.put(
                            {
                                "type": "progress",
                                "frame": frame,
                                "detections": detections,
                                "progress": progress,
                                "status": (
                                    f"분석 중... {video_index + 1}/{len(self.video_files)} "
                                    f"{video_path.name} / {sample_index + 1}/{len(sample_times)} 샘플 "
                                    f"(유효 샘플링 {sample_interval:.2f}초)"
                                ),
                                "counts": aggregate_counts.copy(),
                                "events": all_events[-60:],
                                "ocr_text": timestamp_text or initial_timestamp_text,
                                "ocr_conf": max(ocr_conf, initial_ocr_conf),
                                "preview_video": video_path.name,
                            }
                        )

                capture.release()
        except Exception as exc:
            self.processing_queue.put({"type": "error", "message": str(exc)})
            return

        self.processing_queue.put({"type": "done", "counts": aggregate_counts, "events": all_events, "total_duration": total_duration, "debug_rows": debug_rows})

    def detect_people(self, frame: np.ndarray, hog_scale: float, selected_categories: tuple[str, ...] | list[str] | None = None) -> list[dict]:
        selected_category_set = set(selected_categories or CATEGORY_ORDER)
        if self.yolox_detector is not None:
            try:
                detections = [d for d in self._detect_with_yolox(frame) if d.get("category") in selected_category_set]
                self._sync_detector_runtime(self.yolox_detector)
                self.detector_status_var.set(f"YOLOX (보행자/자전거/킥보드/오토바이/승용/트럭){self._detector_runtime_suffix()}")
                return detections
            except Exception:
                self.yolox_detector = None
                self.detector_status_var.set(f"YOLOX 실패, fallback 전환 / {self.detector_runtime_name} / {self.dnn_device_label}")

        if "person" not in selected_category_set:
            return []

        if self.person_detector is not None:
            try:
                detections = self._detect_with_mediapipe(frame)
                self._sync_detector_runtime(self.person_detector)
                self.detector_status_var.set(f"MPPersonDet (보행자만){self._detector_runtime_suffix()}")
                return detections
            except Exception:
                self.person_detector = None
                self.detector_status_var.set(f"MPPersonDet 실패, fallback 전환 / {self.detector_runtime_name} / {self.dnn_device_label}")

        self.detector_status_var.set("기본 HOG fallback (보행자만) / CPU")
        return self._detect_with_hog(frame, hog_scale)

    def _detect_with_yolox(self, frame: np.ndarray) -> list[dict]:
        padded, scale, pad_x, pad_y = letterbox_to_square(frame, size=640)
        rows = self.yolox_detector.infer(padded)
        detections: list[dict] = []
        frame_h, frame_w = frame.shape[:2]
        if rows is None or len(rows) == 0:
            return detections
        for row in rows:
            x, y, w, h, score, class_id = row[:6]
            category = YOLOX_CLASS_TO_CATEGORY.get(int(class_id))
            if category is None:
                continue
            x1 = max(0, int(round((x - pad_x) / scale)))
            y1 = max(0, int(round((y - pad_y) / scale)))
            x2 = min(frame_w - 1, int(round((x + w - pad_x) / scale)))
            y2 = min(frame_h - 1, int(round((y + h - pad_y) / scale)))
            box_w = max(0, x2 - x1)
            box_h = max(0, y2 - y1)
            if not is_valid_detection_size(category, box_w, box_h):
                continue
            detections.append({"bbox": (x1, y1, box_w, box_h), "score": float(score), "centroid": (x1 + box_w / 2, y1 + box_h), "category": category})
        return detections

    def _detect_with_mediapipe(self, frame: np.ndarray) -> list[dict]:
        rows = self.person_detector.infer(frame)
        detections: list[dict] = []
        frame_h, frame_w = frame.shape[:2]
        for row in rows:
            x1, y1, x2, y2 = row[:4]
            score = float(row[-1])
            x = max(0, int(round(x1)))
            y = max(0, int(round(y1)))
            x2 = min(frame_w - 1, int(round(x2)))
            y2 = min(frame_h - 1, int(round(y2)))
            box_w = max(0, x2 - x)
            box_h = max(0, y2 - y)
            if not is_valid_detection_size("person", box_w, box_h):
                continue
            detections.append({"bbox": (x, y, box_w, box_h), "score": score, "centroid": (x + box_w / 2, y + box_h), "category": "person"})
        return detections

    def _detect_with_hog(self, frame: np.ndarray, hog_scale: float) -> list[dict]:
        resized = cv2.resize(frame, None, fx=hog_scale, fy=hog_scale, interpolation=cv2.INTER_AREA) if hog_scale != 1.0 else frame
        rects, weights = self.hog.detectMultiScale(resized, winStride=(8, 8), padding=(8, 8), scale=1.05)
        boxes: list[tuple[int, int, int, int, float]] = []
        weights_array = weights.flatten() if len(weights) else []
        for (x, y, w, h), weight in zip(rects, weights_array):
            if hog_scale != 1.0:
                x = int(x / hog_scale)
                y = int(y / hog_scale)
                w = int(w / hog_scale)
                h = int(h / hog_scale)
            boxes.append((x, y, w, h, float(weight)))
        return [{"bbox": (x, y, w, h), "score": score, "centroid": (x + w / 2, y + h), "category": "person"} for x, y, w, h, score in non_max_suppression(boxes, overlap_threshold=0.35)]

    def _poll_processing_queue(self) -> None:
        try:
            while True:
                item = self.processing_queue.get_nowait()
                self._handle_worker_item(item)
        except queue.Empty:
            pass
        self.after(120, self._poll_processing_queue)

    def _handle_worker_item(self, item: dict) -> None:
        item_type = item["type"]
        if item_type == "progress":
            self.progress_var.set(f'{item["progress"]}%')
            self.preview_video_var.set(f'처리 중: {item["preview_video"]}')
            self.set_status(item["status"])
            self.ocr_status_var.set(f'하단 OCR: {item["ocr_text"] or "미검출"} / conf {item["ocr_conf"]:.2f}')
            self.refresh_preview(item["frame"], item["detections"])
            rows = convert_counts_to_rows(item["counts"])
            self.populate_results(rows)
            self.populate_events(item["events"])
            self._update_total_metrics(rows)
        elif item_type == "done":
            self.processing_active = False
            self.process_button.configure(state="normal")
            self.export_button.configure(state="normal")
            self.open_folder_button.configure(state="normal")
            self.clear_line_button.configure(state="normal")
            self._set_video_selection_controls_state("normal")
            self._set_category_controls_state("normal")
            self.speed_combo.configure(state="readonly")
            self._update_speed_accuracy_label()
            self.hourly_counts = convert_counts_to_rows(item["counts"])
            self.events = item["events"]
            self.populate_results(self.hourly_counts)
            self.populate_events(self.events)
            self._update_total_metrics(self.hourly_counts)
            self.duration_var.set(format_duration(item["total_duration"]))
            self.progress_var.set("100%")
            self.debug_log_path = self._write_debug_log(item.get("debug_rows", []))
            self.set_status("폴더 분석이 완료되었습니다.")
            if self.debug_log_path is not None:
                self.ocr_status_var.set(f"하단 시간 기준 집계 완료 / 디버그 로그: {self.debug_log_path.name}")
            else:
                self.ocr_status_var.set("하단 시간 기준 집계 완료")
        elif item_type == "error":
            self.processing_active = False
            self.process_button.configure(state="normal")
            self.export_button.configure(state="disabled")
            self.open_folder_button.configure(state="normal")
            self.clear_line_button.configure(state="normal")
            self._set_video_selection_controls_state("normal")
            self._set_category_controls_state("normal")
            self.speed_combo.configure(state="readonly")
            self._update_speed_accuracy_label()
            self.set_status("분석 중 오류가 발생했습니다.")
            messagebox.showerror("분석 실패", item["message"])

    def _write_debug_log(self, debug_rows: list[dict]) -> Path | None:
        if not debug_rows:
            return None
        path = self.base_dir / "ocr_debug_last_run.csv"
        fieldnames = [
            "row_type",
            "video_name",
            "elapsed_seconds",
            "ocr_text",
            "ocr_conf",
            "resolved_timestamp",
            "bucket_key",
            "event_category",
            "event_direction",
            "anchor_note",
        ]
        with open(path, "w", newline="", encoding="utf-8-sig") as file:
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            writer.writeheader()
            for row in debug_rows:
                writer.writerow({name: row.get(name, "") for name in fieldnames})
        return path

    def populate_results(self, rows: list[dict]) -> None:
        self.results_tree.delete(*self.results_tree.get_children())
        visible_categories = tuple(self.active_categories)
        if not rows:
            self.results_tree.insert("", "end", values=("집계 결과가 아직 없습니다.", *["" for _ in range((len(visible_categories) * 2) + 1)]))
            return
        for row in rows:
            values = [row["label"]]
            for category in visible_categories:
                values.extend([row[f"{category}_in"], row[f"{category}_out"]])
            values.append(compute_row_total(row, visible_categories))
            self.results_tree.insert("", "end", values=tuple(values))

    def populate_events(self, events: list[dict]) -> None:
        self.event_listbox.delete(0, tk.END)
        if not events:
            self.event_listbox.insert(tk.END, "이벤트 로그가 없습니다.")
            return
        for event in reversed(events[-250:]):
            event_time = event["timestamp_text"] or format_elapsed(event["elapsed_seconds"])
            category_label = CATEGORY_LABELS.get(event.get("category", "person"), event.get("category", "unknown"))
            self.event_listbox.insert(tk.END, f'[{category_label} {event["direction"]}] {event_time} | {event["video_name"]} | track {event["track_id"]}')

    def _get_result_export_header(self) -> list[str]:
        header = ["time_range"]
        for category in self.active_categories:
            header.extend([f"{category}_in", f"{category}_out"])
        header.append("total")
        return header

    def _get_result_export_row(self, row: dict) -> list:
        values = [row["label"]]
        for category in self.active_categories:
            values.extend([row.get(f"{category}_in", 0), row.get(f"{category}_out", 0)])
        values.append(compute_row_total(row, self.active_categories))
        return values

    def _build_quarter_rows_from_events(self) -> list[dict]:
        quarter_counts = build_empty_quarter_hour_counts()
        for event in self.events:
            increment_bucket_count(
                quarter_counts,
                get_event_quarter_bucket_key(event),
                event.get("category", "person"),
                event["direction"],
            )
        return convert_counts_to_rows(quarter_counts)

    def _write_results_csv(self, path: Path, hourly_rows: list[dict], quarter_rows: list[dict]) -> Path:
        companion_path = path.with_name(f"{path.stem}_15min.csv")
        header = self._get_result_export_header()
        with open(path, "w", newline="", encoding="utf-8-sig") as file:
            writer = csv.writer(file)
            writer.writerow(header)
            for row in hourly_rows:
                writer.writerow(self._get_result_export_row(row))
            writer.writerow([])
            writer.writerow(["category", "direction", "timestamp", "video_name", "track_id", "elapsed_seconds"])
            for event in self.events:
                writer.writerow([
                    event.get("category", "person"),
                    event["direction"],
                    event["timestamp_text"],
                    event["video_name"],
                    event["track_id"],
                    f'{event["elapsed_seconds"]:.2f}',
                ])
        with open(companion_path, "w", newline="", encoding="utf-8-sig") as file:
            writer = csv.writer(file)
            writer.writerow(header)
            for row in quarter_rows:
                writer.writerow(self._get_result_export_row(row))
        return companion_path

    def _write_results_workbook(self, path: Path, hourly_rows: list[dict], quarter_rows: list[dict]) -> None:
        if Workbook is None:
            raise RuntimeError("openpyxl이 설치되지 않아 XLSX 저장을 할 수 없습니다.")

        workbook = Workbook()
        hourly_sheet = workbook.active
        hourly_sheet.title = "hourly_counts"
        header = self._get_result_export_header()
        hourly_sheet.append(header)
        for row in hourly_rows:
            hourly_sheet.append(self._get_result_export_row(row))

        quarter_sheet = workbook.create_sheet("quarter_hour_counts")
        quarter_sheet.append(header)
        for row in quarter_rows:
            quarter_sheet.append(self._get_result_export_row(row))

        event_sheet = workbook.create_sheet("events")
        event_sheet.append(["category", "direction", "timestamp", "video_name", "track_id", "elapsed_seconds"])
        for event in self.events:
            event_sheet.append([
                event.get("category", "person"),
                event["direction"],
                event["timestamp_text"],
                event["video_name"],
                event["track_id"],
                f'{event["elapsed_seconds"]:.2f}',
            ])
        workbook.save(path)

    def save_csv(self) -> None:
        if not self.hourly_counts:
            messagebox.showinfo("저장할 결과 없음", "먼저 분석을 실행해 주세요.")
            return
        path = filedialog.asksaveasfilename(
            title="결과 저장",
            defaultextension=".xlsx",
            filetypes=[("Excel Workbook", "*.xlsx"), ("CSV Files", "*.csv")],
            initialfile="line_crossing_counts.xlsx",
        )
        if not path:
            return
        save_path = Path(path)
        quarter_rows = self._build_quarter_rows_from_events()
        if save_path.suffix.lower() == ".xlsx":
            self._write_results_workbook(save_path, self.hourly_counts, quarter_rows)
            self.set_status("XLSX 저장이 완료되었습니다. 15분 단위 시트가 함께 저장되었습니다.")
            return

        companion_path = self._write_results_csv(save_path, self.hourly_counts, quarter_rows)
        self.set_status(f"CSV 저장이 완료되었습니다. 15분 단위 파일: {companion_path.name}")

    def update_line_label(self) -> None:
        if self.normalized_line is None:
            self.line_var.set("카운팅 선이 아직 지정되지 않았습니다.")
            return
        start, end = scale_normalized_line(self.normalized_line, self.frame_size)
        self.line_var.set(f"선: ({start[0]}, {start[1]}) -> ({end[0]}, {end[1]}) | 시작점->끝점 방향 기준 오른쪽→왼쪽 = IN, 왼쪽→오른쪽 = OUT")

    def set_status(self, text: str) -> None:
        self.status_var.set(text)

    def on_close(self) -> None:
        if self.processing_active and not messagebox.askyesno("종료 확인", "분석 중입니다. 창을 닫을까요?"):
            return
        self.destroy()


def scale_normalized_line(normalized_line: tuple[tuple[float, float], tuple[float, float]], frame_size: tuple[int, int]) -> tuple[tuple[int, int], tuple[int, int]]:
    width, height = frame_size
    start = (int(round(normalized_line[0][0] * width)), int(round(normalized_line[0][1] * height)))
    end = (int(round(normalized_line[1][0] * width)), int(round(normalized_line[1][1] * height)))
    return start, end


def draw_direction_hint(frame: np.ndarray, start: tuple[int, int], end: tuple[int, int]) -> None:
    midpoint = ((start[0] + end[0]) // 2, (start[1] + end[1]) // 2)
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy) or 1.0
    normal = (-dy / length, dx / length)
    in_pos = (int(midpoint[0] + normal[0] * 42), int(midpoint[1] + normal[1] * 42))
    out_pos = (int(midpoint[0] - normal[0] * 42), int(midpoint[1] - normal[1] * 42))
    cv2.putText(frame, "IN", in_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 220, 180), 2, cv2.LINE_AA)
    cv2.putText(frame, "OUT", out_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 215, 120), 2, cv2.LINE_AA)


def compare_binary_images(first: np.ndarray, second: np.ndarray) -> float:
    first = first.astype(np.float32) / 255.0
    second = second.astype(np.float32) / 255.0
    denom = np.linalg.norm(first) * np.linalg.norm(second)
    return 0.0 if denom == 0 else float(np.sum(first * second) / denom)


def clean_timestamp_text(text: str) -> str:
    cleaned = text.replace("..", ":").replace("::", ":")
    cleaned = re.sub(r"[^0-9:/.\- ]+", "", cleaned)
    return re.sub(r"\s+", " ", cleaned).strip()


def parse_timestamp_from_video_name(file_name: str) -> datetime | None:
    patterns = [
        (r"(\d{8})_(\d{6})", "%Y%m%d%H%M%S"),
        (r"(\d{4}-\d{2}-\d{2})[_ ](\d{2}-\d{2}-\d{2})", "%Y-%m-%d%H-%M-%S"),
    ]
    for pattern, fmt in patterns:
        match = re.search(pattern, file_name)
        if not match:
            continue
        try:
            combined = "".join(match.groups())
            return datetime.strptime(combined, fmt)
        except ValueError:
            continue
    return None


def parse_timestamp_text(text: str, previous: datetime | None) -> datetime | None:
    if not text:
        return None
    normalized = re.sub(r"\s+", " ", text.strip()).replace(".", ":")
    candidates = [normalized, normalized.replace(" ", "")]
    candidates.extend(re.findall(r"\d{2,4}[-/]\d{1,2}[-/]\d{1,2}\s*\d{1,2}:\d{2}(?::\d{2})?|\d{1,2}:\d{2}(?::\d{2})?", normalized))
    patterns = ["%Y-%m-%d %H:%M:%S", "%Y/%m/%d %H:%M:%S", "%Y-%m-%d %H:%M", "%Y/%m/%d %H:%M", "%y-%m-%d %H:%M:%S", "%y/%m/%d %H:%M:%S", "%m-%d %H:%M:%S", "%m/%d %H:%M:%S", "%H:%M:%S", "%H:%M"]
    base_date = previous.date() if previous else datetime.today().date()
    for candidate in candidates:
        for pattern in patterns:
            try:
                parsed = datetime.strptime(candidate, pattern)
                if pattern in {"%H:%M:%S", "%H:%M"}:
                    return datetime.combine(base_date, parsed.time())
                if pattern in {"%m-%d %H:%M:%S", "%m/%d %H:%M:%S"}:
                    return datetime(base_date.year, parsed.month, parsed.day, parsed.hour, parsed.minute, parsed.second)
                if pattern in {"%Y-%m-%d %H:%M", "%Y/%m/%d %H:%M"}:
                    return parsed.replace(second=0)
                return parsed
            except ValueError:
                continue
    return None


def format_display_timestamp(value: datetime) -> str:
    return value.strftime("%Y-%m-%d %H:%M:%S")


def floor_to_hour(value: datetime | None) -> datetime | None:
    return None if value is None else value.replace(minute=0, second=0, microsecond=0)


def format_hour_range_label(start: datetime, end: datetime) -> str:
    return f'{start.strftime("%H:%M")} ~ {end.strftime("%H:%M")}'


def format_elapsed_hour_range_label(start_seconds: int, end_seconds: int) -> str:
    return f'{format_elapsed(start_seconds)[:5]} ~ {format_elapsed(end_seconds)[:5]}'


def format_hour_range_label_by_hour(hour_index: int) -> str:
    start_label = f"{hour_index:02d}:00"
    end_label = "24:00" if hour_index == 23 else f"{hour_index + 1:02d}:00"
    return f"{start_label} ~ {end_label}"


def format_quarter_range_label_by_index(bucket_index: int) -> str:
    start_minutes = bucket_index * 15
    end_minutes = start_minutes + 15
    start_hour, start_minute = divmod(start_minutes, 60)
    end_hour, end_minute = divmod(end_minutes, 60)
    return f"{start_hour:02d}:{start_minute:02d} ~ {end_hour:02d}:{end_minute:02d}"


def make_empty_count_entry(label: str, start: int, end: int, sort_key: int) -> dict:
    entry = {
        "start": start,
        "end": end,
        "label": label,
        "sort_group": 0,
        "sort_key": sort_key,
    }
    for category in CATEGORY_ORDER:
        entry[f"{category}_in"] = 0
        entry[f"{category}_out"] = 0
    return entry


def build_empty_hour_counts() -> dict[str, dict]:
    counts: dict[str, dict] = {}
    for hour_index in range(24):
        counts[f"hour:{hour_index:02d}"] = make_empty_count_entry(
            label=format_hour_range_label_by_hour(hour_index),
            start=hour_index,
            end=(hour_index + 1) % 24,
            sort_key=hour_index,
        )
    return counts


def build_empty_quarter_hour_counts() -> dict[str, dict]:
    counts: dict[str, dict] = {}
    for bucket_index in range(96):
        counts[f"quarter:{bucket_index:02d}"] = make_empty_count_entry(
            label=format_quarter_range_label_by_index(bucket_index),
            start=bucket_index * 15,
            end=(bucket_index + 1) * 15,
            sort_key=bucket_index,
        )
    return counts


def get_event_hour_bucket_key(event: dict) -> str:
    timestamp = event.get("timestamp")
    if timestamp is not None:
        return f'hour:{timestamp.hour:02d}'
    timestamp_text = str(event.get("timestamp_text", "")).strip()
    if timestamp_text:
        parsed = parse_timestamp_text(timestamp_text, None)
        if parsed is not None:
            return f'hour:{parsed.hour:02d}'
    elapsed_seconds = max(0, float(event.get("elapsed_seconds", 0.0)))
    hour_index = int(elapsed_seconds // 3600) % 24
    return f'hour:{hour_index:02d}'


def get_event_quarter_bucket_key(event: dict) -> str:
    timestamp = event.get("timestamp")
    if timestamp is not None:
        bucket_index = (timestamp.hour * 4) + (timestamp.minute // 15)
        return f"quarter:{bucket_index:02d}"
    timestamp_text = str(event.get("timestamp_text", "")).strip()
    if timestamp_text:
        parsed = parse_timestamp_text(timestamp_text, None)
        if parsed is not None:
            bucket_index = (parsed.hour * 4) + (parsed.minute // 15)
            return f"quarter:{bucket_index:02d}"
    elapsed_seconds = max(0, float(event.get("elapsed_seconds", 0.0)))
    bucket_index = int(elapsed_seconds // 900) % 96
    return f"quarter:{bucket_index:02d}"


def increment_bucket_count(counts: dict[str, dict], bucket_key: str, category: str, direction: str) -> None:
    count_key = f"{category}_{direction.lower()}"
    bucket = counts.get(bucket_key)
    if bucket is None or count_key not in bucket:
        return
    bucket[count_key] += 1


def choose_best_anchor_candidate(anchor_candidates: list[dict]) -> dict | None:
    if not anchor_candidates:
        return None

    tolerance_seconds = 90.0
    best_group: list[dict] = []
    for candidate in anchor_candidates:
        group = [
            other
            for other in anchor_candidates
            if abs(other["anchor_epoch"] - candidate["anchor_epoch"]) <= tolerance_seconds
        ]
        if not best_group:
            best_group = group
            continue
        current_score = sum(item["confidence"] for item in group)
        best_score = sum(item["confidence"] for item in best_group)
        if len(group) > len(best_group) or (len(group) == len(best_group) and current_score > best_score):
            best_group = group

    if len(best_group) < 2:
        high_conf = [item for item in anchor_candidates if item["confidence"] >= 0.72]
        if high_conf:
            return max(high_conf, key=lambda item: (item["confidence"], len(item["text"])))
        return None

    return max(best_group, key=lambda item: (item["confidence"], len(item["text"])))


def convert_counts_to_rows(counts: dict[str, dict]) -> list[dict]:
    ordered_items = sorted(counts.items(), key=lambda item: item[1]["sort_key"])
    rows: list[dict] = []
    for _bucket_key, info in ordered_items:
        row = {
            "start": info["start"],
            "end": info["end"],
            "label": info["label"],
        }
        for category in CATEGORY_ORDER:
            row[f"{category}_in"] = info[f"{category}_in"]
            row[f"{category}_out"] = info[f"{category}_out"]
        rows.append(row)
    return rows


def compute_row_total(row: dict, categories: tuple[str, ...] | list[str] | None = None) -> int:
    categories = categories or CATEGORY_ORDER
    return sum(row.get(f"{category}_in", 0) + row.get(f"{category}_out", 0) for category in categories)


def summarize_direction_counts(rows: list[dict], categories: tuple[str, ...] | list[str] | None = None) -> tuple[int, int, dict[str, int]]:
    categories = tuple(categories or CATEGORY_ORDER)
    category_totals = {
        category: sum(row.get(f"{category}_in", 0) + row.get(f"{category}_out", 0) for row in rows)
        for category in categories
    }
    in_total = sum(row.get(f"{category}_in", 0) for row in rows for category in categories)
    out_total = sum(row.get(f"{category}_out", 0) for row in rows for category in categories)
    return in_total, out_total, category_totals


def is_valid_detection_size(category: str, box_w: int, box_h: int) -> bool:
    if box_w < 18 or box_h < 18:
        return False
    if category == "person":
        return box_w >= 20 and box_h >= 40
    if category == "bicycle":
        return box_w >= 24 and box_h >= 24
    if category == "motorcycle":
        return box_w >= 24 and box_h >= 24
    if category == "car":
        return box_w >= 28 and box_h >= 24
    if category == "truck":
        return box_w >= 34 and box_h >= 28
    return False


def predict_track_centroid(track: CrossingTrack, current_time: float) -> tuple[float, float]:
    delta = max(0.0, current_time - track.last_seen)
    return (
        track.centroid[0] + track.velocity[0] * delta,
        track.centroid[1] + track.velocity[1] * delta,
    )


def update_track_from_detection(track: CrossingTrack, detection: dict, current_time: float, high_confidence: bool) -> None:
    delta = max(1e-6, current_time - track.last_seen)
    velocity = (
        (detection["centroid"][0] - track.centroid[0]) / delta,
        (detection["centroid"][1] - track.centroid[1]) / delta,
    )
    track.velocity = (
        (track.velocity[0] * 0.4) + (velocity[0] * 0.6),
        (track.velocity[1] * 0.4) + (velocity[1] * 0.6),
    )
    track.bbox = detection["bbox"]
    track.previous_centroid = track.centroid
    track.centroid = detection["centroid"]
    track.last_seen = current_time
    track.seen_count += 1
    if high_confidence:
        track.high_conf_hits += 1


def classify_point_side(point: tuple[float, float], line_start: tuple[int, int], line_end: tuple[int, int], margin_px: float) -> int:
    distance = signed_distance_to_line(point, line_start, line_end)
    if distance > margin_px:
        return 1
    if distance < -margin_px:
        return -1
    return 0


def signed_distance_to_line(point: tuple[float, float], line_start: tuple[int, int], line_end: tuple[int, int]) -> float:
    dx = line_end[0] - line_start[0]
    dy = line_end[1] - line_start[1]
    length = math.hypot(dx, dy) or 1.0
    return ((dx * (point[1] - line_start[1])) - (dy * (point[0] - line_start[0]))) / length


def signed_side(point: tuple[float, float], line_start: tuple[int, int], line_end: tuple[int, int]) -> float:
    return (line_end[0] - line_start[0]) * (point[1] - line_start[1]) - (line_end[1] - line_start[1]) * (point[0] - line_start[0])


def segments_intersect(p1: tuple[float, float], p2: tuple[float, float], q1: tuple[float, float], q2: tuple[float, float]) -> bool:
    o1 = orientation(p1, p2, q1)
    o2 = orientation(p1, p2, q2)
    o3 = orientation(q1, q2, p1)
    o4 = orientation(q1, q2, p2)
    return o1 * o2 < 0 and o3 * o4 < 0


def orientation(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def letterbox_to_square(image: np.ndarray, size: int = 640, pad_value: tuple[int, int, int] = (114, 114, 114)) -> tuple[np.ndarray, float, int, int]:
    height, width = image.shape[:2]
    scale = min(size / width, size / height)
    resized_width = max(1, int(round(width * scale)))
    resized_height = max(1, int(round(height * scale)))
    resized = cv2.resize(image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((size, size, 3), pad_value, dtype=np.uint8)
    pad_x = (size - resized_width) // 2
    pad_y = (size - resized_height) // 2
    canvas[pad_y : pad_y + resized_height, pad_x : pad_x + resized_width] = resized
    return canvas, scale, pad_x, pad_y


def non_max_suppression(boxes: list[tuple[int, int, int, int, float]], overlap_threshold: float) -> list[tuple[int, int, int, int, float]]:
    if not boxes:
        return []
    selected: list[tuple[int, int, int, int, float]] = []
    candidates = sorted(boxes, key=lambda item: item[4], reverse=True)
    while candidates:
        current = candidates.pop(0)
        selected.append(current)
        candidates = [candidate for candidate in candidates if intersection_over_union(current, candidate) < overlap_threshold]
    return selected


def intersection_over_union(first: tuple[int, int, int, int, float], second: tuple[int, int, int, int, float]) -> float:
    ax, ay, aw, ah, _ = first
    bx, by, bw, bh, _ = second
    inter_x1 = max(ax, bx)
    inter_y1 = max(ay, by)
    inter_x2 = min(ax + aw, bx + bw)
    inter_y2 = min(ay + ah, by + bh)
    inter_w = max(0, inter_x2 - inter_x1)
    inter_h = max(0, inter_y2 - inter_y1)
    intersection = inter_w * inter_h
    union = aw * ah + bw * bh - intersection
    return intersection / union if union > 0 else 0.0


def bbox_iou(first: tuple[int, int, int, int], second: tuple[int, int, int, int]) -> float:
    ax, ay, aw, ah = first
    bx, by, bw, bh = second
    inter_x1 = max(ax, bx)
    inter_y1 = max(ay, by)
    inter_x2 = min(ax + aw, bx + bw)
    inter_y2 = min(ay + ah, by + bh)
    inter_w = max(0, inter_x2 - inter_x1)
    inter_h = max(0, inter_y2 - inter_y1)
    intersection = inter_w * inter_h
    union = aw * ah + bw * bh - intersection
    return intersection / union if union > 0 else 0.0


def build_sample_times(duration: float, interval: float) -> list[float]:
    if duration <= 0:
        return [0.0]
    times: list[float] = []
    current = 0.0
    while current <= duration:
        times.append(round(current, 3))
        current += interval
    if times[-1] < duration:
        times.append(duration)
    return times


def format_duration(seconds: float) -> str:
    if seconds <= 0:
        return "0분"
    minutes = round(seconds / 60)
    if minutes < 60:
        return f"{minutes}분"
    hours = minutes // 60
    remain = minutes % 60
    return f"{hours}시간 {remain}분" if remain else f"{hours}시간"


def format_elapsed(seconds: float) -> str:
    seconds = max(0, int(seconds))
    return f'{str(seconds // 3600).zfill(2)}:{str((seconds % 3600) // 60).zfill(2)}:{str(seconds % 60).zfill(2)}'


if __name__ == "__main__":
    app = PersonCounterApp()
    app.mainloop()
