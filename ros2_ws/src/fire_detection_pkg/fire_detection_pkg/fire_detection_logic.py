#!/usr/bin/env python3
import cv2
import numpy as np

# Summit logic
from .summit import (
    Summit,
    choisir_meilleure_cible,
    build_or_update_summits_from_matrices
)

# ================================
# PARAMETERS
# ================================
T_MIN = 20.0
T_MAX = 900.0

W_RED = 1.0
W_ORANGE = 0.5

BLACK_PERSON_THRESH = 0.03

EXPECTED_ROWS = 3
EXPECTED_COLS = 3


# ================================
# TEMPERATURE CALCULATION
# ================================
def temperature_from_counts(red_count, orange_count, black_count, total_pixels):
    effective_area = max(1, total_pixels - black_count)

    total_fire = red_count + orange_count
    if total_fire == 0:
        return T_MIN

    fire_coverage = total_fire / effective_area
    mix_red = red_count / total_fire
    mix_orange = orange_count / total_fire

    mix_score = W_RED * mix_red + W_ORANGE * mix_orange
    fire_score = np.clip(fire_coverage * mix_score, 0.0, 1.0)

    return T_MIN + fire_score * (T_MAX - T_MIN)


# ================================
# COLOR MASKS
# ================================
def get_color_masks(frame_bgr):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    red_l1 = np.array([0, 130, 80])
    red_u1 = np.array([5, 255, 255])
    red_l2 = np.array([170, 130, 80])
    red_u2 = np.array([180, 255, 255])

    orange_l = np.array([6, 150, 100])
    orange_u = np.array([20, 255, 255])

    black_l = np.array([0, 0, 0])
    black_u = np.array([180, 255, 60])

    red_mask = cv2.bitwise_or(
        cv2.inRange(hsv, red_l1, red_u1),
        cv2.inRange(hsv, red_l2, red_u2)
    )
    orange_mask = cv2.inRange(hsv, orange_l, orange_u)
    black_mask = cv2.inRange(hsv, black_l, black_u)

    kernel = np.ones((3, 3), np.uint8)
    red_mask = cv2.dilate(cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel), kernel)
    orange_mask = cv2.dilate(cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, kernel), kernel)
    black_mask = cv2.dilate(cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, kernel), kernel)

    return red_mask, orange_mask, black_mask


# ================================
# WINDOW DETECTION
# ================================
def detect_windows_from_firemask(red_mask, orange_mask):
    fire_mask = cv2.bitwise_or(red_mask, orange_mask)
    fire_mask = cv2.morphologyEx(fire_mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

    contours, _ = cv2.findContours(fire_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    rects = []
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        area = w * h

        if area < 400:
            continue

        aspect = w / float(h)
        if aspect < 0.25 or aspect > 3.0:
            continue

        rects.append((x, y, w, h))

    rects.sort(key=lambda r: (r[1] + r[3]/2, r[0] + r[2]/2))
    return rects


# ================================
# GRID GROUPING
# ================================
def group_rects_into_grid(rects):

    if not rects:
        return []

    centers_y = [y + h/2 for (_, y, _, h) in rects]
    heights = [h for (_, _, _, h) in rects]

    median_h = np.median(heights)
    row_thresh = 0.6 * median_h

    rows = []
    current = [rects[0]]
    current_y = centers_y[0]

    for i in range(1, len(rects)):
        cy = centers_y[i]
        if abs(cy - current_y) <= row_thresh:
            current.append(rects[i])
        else:
            current.sort(key=lambda r: r[0] + r[2]/2)
            rows.append(current)
            current = [rects[i]]
            current_y = cy

    current.sort(key=lambda r: r[0] + r[2]/2)
    rows.append(current)
    return rows


# ================================
# MAIN PROCESSING FUNCTION
# ================================
def process_frame(frame_bgr):

    red_mask, orange_mask, black_mask = get_color_masks(frame_bgr)
    annotated = frame_bgr.copy()

    rects = detect_windows_from_firemask(red_mask, orange_mask)
    rows = group_rects_into_grid(rects)

    # FIXED OUTPUT MATRICES
    temp_matrix = np.full((EXPECTED_ROWS, EXPECTED_COLS), np.nan, dtype=np.float32)
    person_matrix = np.zeros((EXPECTED_ROWS, EXPECTED_COLS), dtype=int)

    temps_flat = []
    persons_flat = []
    windows_flat = []

    for r in range(min(len(rows), EXPECTED_ROWS)):
        row = rows[r]
        for c in range(min(len(row), EXPECTED_COLS)):
            x, y, w, h = row[c]

            red_win = red_mask[y:y+h, x:x+w]
            orange_win = orange_mask[y:y+h, x:x+w]
            black_win = black_mask[y:y+h, x:x+w]

            total_pixels = red_win.size
            red_count = np.count_nonzero(red_win)
            orange_count = np.count_nonzero(orange_win)
            black_count = np.count_nonzero(black_win)

            T = temperature_from_counts(red_count, orange_count, black_count, total_pixels)
            person = (black_count / max(1, total_pixels)) >= BLACK_PERSON_THRESH

            temp_matrix[r, c] = T
            person_matrix[r, c] = 1 if person else 0

            temps_flat.append(T)
            persons_flat.append(person)
            windows_flat.append((x, y, w, h))

            # Annotation
            cv2.rectangle(annotated, (x, y), (x+w, y+h), (0, 255, 255), 2)
            label = f"{int(T)}C" + (" P" if person else "")
            cv2.putText(
                annotated, label, (x + w//2 - 40, y + h//2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1
            )

    # PAD TO ALWAYS HAVE 9 WINDOWS (0..8)
    while len(temps_flat) < 9:
        temps_flat.append(np.nan)
        persons_flat.append(False)
        windows_flat.append(None)

    return annotated, temps_flat, persons_flat, windows_flat, temp_matrix, person_matrix
