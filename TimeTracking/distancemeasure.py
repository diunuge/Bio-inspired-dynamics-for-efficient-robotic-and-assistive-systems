import os
import sys
import csv

import cv2
import matplotlib.pyplot as plt
import numpy as np


script_dir = os.path.dirname(os.path.abspath(__file__))
video_file = sys.argv[1] if len(sys.argv) > 1 else "video1.MP4"
video_path = video_file if os.path.isabs(video_file) else os.path.join(script_dir, video_file)

cap = cv2.VideoCapture(video_path)
if not cap.isOpened():
    raise SystemExit(f"Error: Could not open video: {video_path}")

fps = cap.get(cv2.CAP_PROP_FPS)
if fps <= 0:
    fps = 30.0

print("Video opened successfully.")

# Tightened HSV thresholds. If your markers are darker/lighter in another video,
# adjust these first while keeping saturation/value fairly high.
COLOR_CONFIGS = {
    "green": {
        "lower": np.array([40, 80, 80]),
        "upper": np.array([85, 255, 255]),
        "draw_color": (0, 255, 0),
        "plot_color": "g",
    },
    "blue": {
        "lower": np.array([95, 80, 80]),
        "upper": np.array([130, 255, 255]),
        "draw_color": (255, 0, 0),
        "plot_color": "b",
    },
}

# Detection tuning for large round markers after the 0.5x resize.
MIN_MARKER_AREA = 3000
MAX_MARKER_AREA = 50000
MIN_MARKER_RADIUS = 18
MAX_MARKER_RADIUS = 140
ROUND_MIN_CIRCULARITY = 0.7
ROUND_MIN_FILL_RATIO = 0.65
ROUND_MIN_ASPECT_RATIO = 0.75
ROUND_MAX_ASPECT_RATIO = 1.25
BLUR_MIN_CIRCULARITY = 0.35
BLUR_MIN_FILL_RATIO = 0.4
BLUR_MAX_ELLIPSE_RATIO = 2.2
MIN_EXTENT = 0.45
MIN_SOLIDITY = 0.8
BASE_MATCH_DISTANCE = 60
MAX_MATCH_DISTANCE = 140
SPEED_MATCH_GAIN = 0.75
MISSED_FRAME_MATCH_BONUS = 12
MAX_MISSED_FRAMES = 12

OPEN_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
CLOSE_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
DILATE_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))


def clean_mask(mask):
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, OPEN_KERNEL, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, CLOSE_KERNEL, iterations=2)
    mask = cv2.dilate(mask, DILATE_KERNEL, iterations=1)
    return mask


def tracker_prediction(tracker):
    _, last_x, last_y = tracker["positions"][-1]
    if len(tracker["positions"]) < 2:
        return last_x, last_y, 0.0

    _, prev_x, prev_y = tracker["positions"][-2]
    vx = last_x - prev_x
    vy = last_y - prev_y
    predicted_x = last_x + vx * tracker["missed_frames"]
    predicted_y = last_y + vy * tracker["missed_frames"]
    speed = np.hypot(vx, vy)
    return predicted_x, predicted_y, speed


def select_reference_start(trackers):
    green_trackers = [
        (tid, data)
        for tid, data in trackers.items()
        if data["color_name"] == "green" and data["positions"]
    ]
    if green_trackers:
        reference_tid, reference_data = max(
            green_trackers,
            key=lambda item: (len(item[1]["positions"]), -item[1]["positions"][0][0]),
        )
        print(f"Using green tracker {reference_tid} start as distance reference.")
        return reference_data["start"]

    for tid, data in trackers.items():
        if data["positions"]:
            print(f"Warning: no green tracker found, using tracker {tid} start as fallback reference.")
            return data["start"]

    return None


def detect_markers(hsv_frame):
    detections = []
    combined_mask = np.zeros(hsv_frame.shape[:2], dtype=np.uint8)

    for color_name, config in COLOR_CONFIGS.items():
        mask = cv2.inRange(hsv_frame, config["lower"], config["upper"])
        mask = clean_mask(mask)
        combined_mask = cv2.bitwise_or(combined_mask, mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_MARKER_AREA or area > MAX_MARKER_AREA:
                continue

            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue

            circularity = 4 * np.pi * area / (perimeter * perimeter)
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = w / float(h)
            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            circle_area = np.pi * (radius ** 2)
            fill_ratio = area / circle_area if circle_area > 0 else 0
            extent = area / float(w * h)
            hull = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            solidity = area / hull_area if hull_area > 0 else 0

            if len(cnt) >= 5:
                (_, _), (major_axis, minor_axis), _ = cv2.fitEllipse(cnt)
                if minor_axis > 0:
                    ellipse_ratio = max(major_axis, minor_axis) / min(major_axis, minor_axis)
                else:
                    ellipse_ratio = float("inf")
            else:
                ellipse_ratio = max(aspect_ratio, 1.0 / aspect_ratio)

            round_marker = (
                circularity >= ROUND_MIN_CIRCULARITY
                and fill_ratio >= ROUND_MIN_FILL_RATIO
                and ROUND_MIN_ASPECT_RATIO <= aspect_ratio <= ROUND_MAX_ASPECT_RATIO
            )
            blurred_marker = (
                circularity >= BLUR_MIN_CIRCULARITY
                and fill_ratio >= BLUR_MIN_FILL_RATIO
                and ellipse_ratio <= BLUR_MAX_ELLIPSE_RATIO
                and extent >= MIN_EXTENT
                and solidity >= MIN_SOLIDITY
            )

            if radius < MIN_MARKER_RADIUS or radius > MAX_MARKER_RADIUS:
                continue
            if not (round_marker or blurred_marker):
                continue

            detections.append(
                {
                    "center": (int(cx), int(cy)),
                    "radius": int(radius),
                    "area": area,
                    "ellipse_ratio": ellipse_ratio,
                    "color_name": color_name,
                    "draw_color": config["draw_color"],
                }
            )

    detections.sort(key=lambda item: item["area"], reverse=True)
    return detections, combined_mask


# Trackers: id -> start position, latest positions, and marker color
trackers = {}
active_ids = set()
next_id = 0
frame_count = 0

# CSV file
csv_path = os.path.join(script_dir, "tracking_data.csv")
with open(csv_path, "w", newline="") as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["time_s", "object_id", "x", "y", "distance_from_green_start"])

while True:
    ret, frame = cap.read()
    if not ret:
        break

    h, w = frame.shape[:2]
    left = int(w * 0.2)
    right = int(w * 0.8)
    frame = frame[:, left:right]

    frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    filtered = cv2.medianBlur(frame, 5)
    hsv = cv2.cvtColor(filtered, cv2.COLOR_BGR2HSV)

    detections, combined_mask = detect_markers(hsv)
    color_mask = cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR)

    current_time = frame_count / fps
    for tid in list(active_ids):
        trackers[tid]["missed_frames"] += 1

    matched_ids = set()
    for detection in detections:
        cx, cy = detection["center"]
        best_id = None
        min_dist = float("inf")

        for tid in active_ids:
            if tid in matched_ids:
                continue

            tracker = trackers[tid]
            if tracker["color_name"] != detection["color_name"]:
                continue

            predicted_x, predicted_y, speed = tracker_prediction(tracker)
            allowed_dist = min(
                MAX_MATCH_DISTANCE,
                BASE_MATCH_DISTANCE
                + speed * SPEED_MATCH_GAIN
                + tracker["missed_frames"] * MISSED_FRAME_MATCH_BONUS,
            )
            dist = np.hypot(cx - predicted_x, cy - predicted_y)
            if dist < allowed_dist and dist < min_dist:
                min_dist = dist
                best_id = tid

        if best_id is not None:
            trackers[best_id]["positions"].append((current_time, cx, cy))
            trackers[best_id]["missed_frames"] = 0
            matched_ids.add(best_id)
        else:
            trackers[next_id] = {
                "start": (cx, cy),
                "positions": [(current_time, cx, cy)],
                "color_name": detection["color_name"],
                "draw_color": detection["draw_color"],
                "plot_color": COLOR_CONFIGS[detection["color_name"]]["plot_color"],
                "missed_frames": 0,
            }
            active_ids.add(next_id)
            matched_ids.add(next_id)
            next_id += 1

        cv2.circle(frame, detection["center"], detection["radius"], detection["draw_color"], 2)
        cv2.circle(color_mask, detection["center"], detection["radius"], detection["draw_color"], 2)

    stale_ids = {tid for tid in active_ids if trackers[tid]["missed_frames"] > MAX_MISSED_FRAMES}
    active_ids -= stale_ids

    for tid in active_ids:
        _, x, y = trackers[tid]["positions"][-1]
        draw_color = trackers[tid]["draw_color"]
        label = f"{trackers[tid]['color_name'][0].upper()}{tid}"
        cv2.circle(frame, (x, y), 5, draw_color, -1)
        cv2.putText(
            frame,
            label,
            (x + 10, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            draw_color,
            2,
            cv2.LINE_AA,
        )

    cv2.imshow("Tracking", frame)
    cv2.imshow("Mask", color_mask)
    if cv2.waitKey(1) == 27:
        break

    frame_count += 1

cap.release()
cv2.destroyAllWindows()

reference_start = select_reference_start(trackers)
with open(csv_path, "a", newline="") as csvfile:
    writer = csv.writer(csvfile)
    for tid, data in trackers.items():
        if reference_start is None:
            continue
        start_x, start_y = reference_start
        for t, x, y in data["positions"]:
            dist = np.hypot(x - start_x, y - start_y)
            writer.writerow([t, tid, x, y, dist])

plt.figure(figsize=(10, 6))
if reference_start is not None:
    for tid, data in trackers.items():
        times = [pos[0] for pos in data["positions"]]
        distances = [
            np.hypot(pos[1] - reference_start[0], pos[2] - reference_start[1])
            for pos in data["positions"]
        ]
        plt.plot(times, distances, label=f"{data['color_name'].title()} {tid}", color=data["plot_color"])
else:
    print("No reference point available for distance plotting.")

plt.xlabel("Time (s)")
plt.ylabel("Distance from Green Start (pixels)")
plt.title("Distance from Green Marker Start")
if reference_start is not None:
    plt.legend()
plt.grid(True)
plt.show()
