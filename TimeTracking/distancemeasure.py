import os
import sys
import csv

import cv2
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import CheckButtons


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
DEFAULT_COLOR_CONFIGS = {
    "green": {
        "lower": np.array([40, 70, 80]),
        "upper": np.array([85, 255, 255]),
        "draw_color": (0, 255, 0),
        "plot_color": "g",
    },
    "blue": {
        "lower": np.array([95, 70, 80]),
        "upper": np.array([130, 255, 255]),
        "draw_color": (255, 0, 0),
        "plot_color": "b",
    },
}

CONTROL_WINDOW = "Controls"
TRACKING_WINDOW = "Tracking"
MASK_WINDOW = "Mask"
TRACKBAR_PREFIXES = {
    "green": "G",
    "blue": "B",
}
# Preview-only scale. Change this before running if the Tracking/Mask windows
# are too large or too small on screen. This does not affect detection.
DISPLAY_SCALE_PERCENT = 60

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
# More permissive limits used only when updating an already-tracked marker.
TRACK_MIN_MARKER_AREA = 1200
TRACK_MIN_MARKER_RADIUS = 12
TRACK_MIN_CIRCULARITY = 0.12
TRACK_MIN_FILL_RATIO = 0.2
TRACK_MAX_ELLIPSE_RATIO = 4.0
TRACK_MIN_EXTENT = 0.18
TRACK_MIN_SOLIDITY = 0.45
BASE_MATCH_DISTANCE = 60
MAX_MATCH_DISTANCE = 140
SPEED_MATCH_GAIN = 0.75
MISSED_FRAME_MATCH_BONUS = 12
MAX_MISSED_FRAMES = 12

OPEN_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
CLOSE_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
DILATE_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))


def clone_color_configs(source_configs):
    return {
        color_name: {
            "lower": config["lower"].copy(),
            "upper": config["upper"].copy(),
            "draw_color": config["draw_color"],
            "plot_color": config["plot_color"],
        }
        for color_name, config in source_configs.items()
    }


def on_trackbar_change(_value):
    pass


def create_hsv_trackbars(window_name, color_configs):
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    for color_name, prefix in TRACKBAR_PREFIXES.items():
        lower = color_configs[color_name]["lower"]
        upper = color_configs[color_name]["upper"]
        cv2.createTrackbar(f"{prefix} LH", window_name, int(lower[0]), 179, on_trackbar_change)
        cv2.createTrackbar(f"{prefix} LS", window_name, int(lower[1]), 255, on_trackbar_change)
        cv2.createTrackbar(f"{prefix} LV", window_name, int(lower[2]), 255, on_trackbar_change)
        cv2.createTrackbar(f"{prefix} UH", window_name, int(upper[0]), 179, on_trackbar_change)
        cv2.createTrackbar(f"{prefix} US", window_name, int(upper[1]), 255, on_trackbar_change)
        cv2.createTrackbar(f"{prefix} UV", window_name, int(upper[2]), 255, on_trackbar_change)


def set_hsv_trackbars(window_name, color_configs):
    for color_name, prefix in TRACKBAR_PREFIXES.items():
        lower = color_configs[color_name]["lower"]
        upper = color_configs[color_name]["upper"]
        cv2.setTrackbarPos(f"{prefix} LH", window_name, int(lower[0]))
        cv2.setTrackbarPos(f"{prefix} LS", window_name, int(lower[1]))
        cv2.setTrackbarPos(f"{prefix} LV", window_name, int(lower[2]))
        cv2.setTrackbarPos(f"{prefix} UH", window_name, int(upper[0]))
        cv2.setTrackbarPos(f"{prefix} US", window_name, int(upper[1]))
        cv2.setTrackbarPos(f"{prefix} UV", window_name, int(upper[2]))


def read_hsv_trackbars(window_name, base_configs):
    color_configs = clone_color_configs(base_configs)
    for color_name, prefix in TRACKBAR_PREFIXES.items():
        lh = cv2.getTrackbarPos(f"{prefix} LH", window_name)
        ls = cv2.getTrackbarPos(f"{prefix} LS", window_name)
        lv = cv2.getTrackbarPos(f"{prefix} LV", window_name)
        uh = cv2.getTrackbarPos(f"{prefix} UH", window_name)
        us = cv2.getTrackbarPos(f"{prefix} US", window_name)
        uv = cv2.getTrackbarPos(f"{prefix} UV", window_name)

        lower = np.array([min(lh, uh), min(ls, us), min(lv, uv)])
        upper = np.array([max(lh, uh), max(ls, us), max(lv, uv)])
        color_configs[color_name]["lower"] = lower
        color_configs[color_name]["upper"] = upper

    return color_configs


def print_color_configs(color_configs):
    print("COLOR_CONFIGS = {")
    for color_name in ("green", "blue"):
        config = color_configs[color_name]
        lower = config["lower"].tolist()
        upper = config["upper"].tolist()
        print(f'    "{color_name}": {{')
        print(f"        \"lower\": np.array({lower}),")
        print(f"        \"upper\": np.array({upper}),")
        print(f"        \"draw_color\": {config['draw_color']},")
        print(f"        \"plot_color\": \"{config['plot_color']}\",")
        print("    },")
    print("}")


def scale_for_display(image):
    if DISPLAY_SCALE_PERCENT == 100:
        return image

    scale = DISPLAY_SCALE_PERCENT / 100.0
    new_width = max(1, int(image.shape[1] * scale))
    new_height = max(1, int(image.shape[0] * scale))
    return cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_AREA)


def show_scaled_window(window_name, image):
    display_image = scale_for_display(image)
    cv2.imshow(window_name, display_image)
    cv2.resizeWindow(window_name, display_image.shape[1], display_image.shape[0])


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


def detect_markers(hsv_frame, color_configs):
    detections = []
    tracking_candidates = {color_name: [] for color_name in color_configs}
    combined_mask = np.zeros(hsv_frame.shape[:2], dtype=np.uint8)

    for color_name, config in color_configs.items():
        mask = cv2.inRange(hsv_frame, config["lower"], config["upper"])
        mask = clean_mask(mask)
        combined_mask = cv2.bitwise_or(combined_mask, mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour_index, cnt in enumerate(contours):
            area = cv2.contourArea(cnt)
            if area < TRACK_MIN_MARKER_AREA or area > MAX_MARKER_AREA:
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
            tracking_marker = (
                circularity >= TRACK_MIN_CIRCULARITY
                and fill_ratio >= TRACK_MIN_FILL_RATIO
                and ellipse_ratio <= TRACK_MAX_ELLIPSE_RATIO
                and extent >= TRACK_MIN_EXTENT
                and solidity >= TRACK_MIN_SOLIDITY
            )

            if radius < TRACK_MIN_MARKER_RADIUS or radius > MAX_MARKER_RADIUS:
                continue
            if not tracking_marker:
                continue

            candidate = {
                "candidate_id": f"{color_name}-{contour_index}",
                "center": (int(cx), int(cy)),
                "radius": int(radius),
                "area": area,
                "ellipse_ratio": ellipse_ratio,
                "color_name": color_name,
                "draw_color": config["draw_color"],
            }
            tracking_candidates[color_name].append(candidate)

            if area < MIN_MARKER_AREA:
                continue
            if radius < MIN_MARKER_RADIUS:
                continue
            if not (round_marker or blurred_marker):
                continue

            detections.append(candidate)

    detections.sort(key=lambda item: item["area"], reverse=True)
    for color_name in tracking_candidates:
        tracking_candidates[color_name].sort(key=lambda item: item["area"], reverse=True)
    return detections, tracking_candidates, combined_mask


# Trackers: id -> start position, latest positions, and marker color
trackers = {}
active_ids = set()
next_id = 0
frame_count = 0
COLOR_CONFIGS = clone_color_configs(DEFAULT_COLOR_CONFIGS)

create_hsv_trackbars(CONTROL_WINDOW, COLOR_CONFIGS)
cv2.namedWindow(TRACKING_WINDOW, cv2.WINDOW_NORMAL)
cv2.namedWindow(MASK_WINDOW, cv2.WINDOW_NORMAL)
print("Use the Controls window trackbars to tune HSV. Change DISPLAY_SCALE_PERCENT at the top before running to resize the preview windows.")
print("Press 'p' to print current HSV values, 'r' to reset HSV, ESC to quit.")

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
    COLOR_CONFIGS = read_hsv_trackbars(CONTROL_WINDOW, COLOR_CONFIGS)

    detections, tracking_candidates, combined_mask = detect_markers(hsv, COLOR_CONFIGS)
    color_mask = cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR)

    current_time = frame_count / fps
    for tid in list(active_ids):
        trackers[tid]["missed_frames"] += 1

    matched_ids = set()
    used_candidate_ids = set()
    for tid in sorted(active_ids, key=lambda tracker_id: trackers[tracker_id]["missed_frames"]):
        best_id = None
        best_detection = None
        min_dist = float("inf")
        tracker = trackers[tid]
        color_name = tracker["color_name"]
        candidate_pool = tracking_candidates[color_name]

        for detection in candidate_pool:
            if detection["candidate_id"] in used_candidate_ids:
                continue

            cx, cy = detection["center"]
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
                best_detection = detection

        if best_id is not None and best_detection is not None:
            cx, cy = best_detection["center"]
            trackers[best_id]["positions"].append((current_time, cx, cy))
            trackers[best_id]["missed_frames"] = 0
            matched_ids.add(best_id)
            used_candidate_ids.add(best_detection["candidate_id"])

    for detection in detections:
        if detection["candidate_id"] in used_candidate_ids:
            continue

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
            used_candidate_ids.add(detection["candidate_id"])
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
            used_candidate_ids.add(detection["candidate_id"])
            next_id += 1

    for candidate_list in tracking_candidates.values():
        for detection in candidate_list:
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

    show_scaled_window(TRACKING_WINDOW, frame)
    show_scaled_window(MASK_WINDOW, color_mask)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("p"):
        print_color_configs(COLOR_CONFIGS)
        print(f"DISPLAY_SCALE_PERCENT = {DISPLAY_SCALE_PERCENT}")
    elif key == ord("r"):
        COLOR_CONFIGS = clone_color_configs(DEFAULT_COLOR_CONFIGS)
        set_hsv_trackbars(CONTROL_WINDOW, COLOR_CONFIGS)
        print("HSV trackbars reset to defaults.")
    elif key == 27:
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

fig, ax = plt.subplots(figsize=(10, 6))
if reference_start is not None:
    lines_by_color = {"green": [], "blue": []}
    for tid, data in trackers.items():
        times = [pos[0] for pos in data["positions"]]
        distances = [
            np.hypot(pos[1] - reference_start[0], pos[2] - reference_start[1])
            for pos in data["positions"]
        ]
        line, = ax.plot(
            times,
            distances,
            label=f"{data['color_name'].title()} {tid}",
            color=data["plot_color"],
        )
        lines_by_color[data["color_name"]].append(line)

    plt.subplots_adjust(left=0.22)
    check_ax = fig.add_axes([0.03, 0.4, 0.13, 0.14])
    check = CheckButtons(check_ax, ["Green", "Blue"], [True, True])
    check_ax.set_title("Show")

    def toggle_color(label):
        color_name = label.lower()
        for line in lines_by_color.get(color_name, []):
            line.set_visible(not line.get_visible())
        fig.canvas.draw_idle()

    check.on_clicked(toggle_color)
else:
    print("No reference point available for distance plotting.")

ax.set_xlabel("Time (s)")
ax.set_ylabel("Distance from Green Start (pixels)")
ax.set_title("Distance from Green Marker Start")
if reference_start is not None:
    ax.legend()
ax.grid(True)
plt.show()
