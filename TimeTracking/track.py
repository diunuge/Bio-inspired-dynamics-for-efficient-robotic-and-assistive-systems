import os
import sys
import cv2
import numpy as np

script_dir = os.path.dirname(os.path.abspath(__file__))
video_file = sys.argv[1] if len(sys.argv) > 1 else "video.MP4"
video_path = video_file if os.path.isabs(video_file) else os.path.join(script_dir, video_file)

cap = cv2.VideoCapture(video_path)
if not cap.isOpened():
    raise SystemExit(f"Error: Could not open video: {video_path}")

# Check if video opened successfully and print feedback
if not cap.isOpened():
    print("Error: Could not open video.")
else:
    print("Video opened successfully.")

def nothing(x):
    pass

cv2.namedWindow("Mask")
cv2.createTrackbar("LH", "Mask", 35, 179, nothing)
cv2.createTrackbar("LS", "Mask", 55, 255, nothing)
cv2.createTrackbar("LV", "Mask", 50, 255, nothing)
cv2.createTrackbar("UH", "Mask", 85, 179, nothing)
cv2.createTrackbar("US", "Mask", 75, 255, nothing)
cv2.createTrackbar("UV", "Mask", 255, 255, nothing)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # crop to region of interest: vertical slice plus 20% side margins
    h, w = frame.shape[:2]
    left = int(w * 0.2)
    right = int(w * 0.8)
    frame = frame[0:2160, left:right]

    # Resize for faster processing AND better QUALITY
    frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)

    # Convert to HSV for color detection
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Example: detect red marker using live adjustable HSV ranges
    lh = cv2.getTrackbarPos("LH", "Mask")
    ls = cv2.getTrackbarPos("LS", "Mask")
    lv = cv2.getTrackbarPos("LV", "Mask")
    uh = cv2.getTrackbarPos("UH", "Mask")
    us = cv2.getTrackbarPos("US", "Mask")
    uv = cv2.getTrackbarPos("UV", "Mask")

    lower = np.array([lh, ls, lv])
    upper = np.array([uh, us, uv])
    mask = cv2.inRange(hsv, lower, upper)

    # SHOW MASK FOR DEBUGGING
    cv2.imshow("Mask", mask) 

    # Find contours (markers)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        if cv2.contourArea(cnt) > 50:
            x, y, w, h = cv2.boundingRect(cnt)
            cx = x + w//2
            cy = y + h//2

            cv2.circle(frame, (cx, cy), 5, (0,255,0), -1)

    cv2.imshow("Tracking", frame)
    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()