import cv2
import os
import glob
from ultralytics import YOLO

# =========================================================
# CONFIG
# =========================================================

MODEL_PATH = "rov.pt"     # pretrained model
MODE = "webcam"               # options: "image", "images", "video", "webcam"

# Input paths
IMAGE_PATH = "test.jpg"
IMAGES_FOLDER = "test_images"
VIDEO_PATH = "underwater.mp4"
WEBCAM_INDEX = 0

# Detection
CONF_THRESHOLD = 0.4
ALPHA = 0.6

# Stability
REQUIRED_STABLE_FRAMES = 3
LOST_TARGET_RESET_FRAMES = 8

# Alignment / approach thresholds
X_THRESHOLD = 0.08
Y_THRESHOLD = 0.08
AREA_CLOSE_THRESHOLD = 0.08
DESIRED_AREA = 0.10

# Control gains
K_YAW = 1.2
K_HEAVE = 1.2
K_SURGE = 0.8

# Command limits
MAX_YAW = 0.5
MAX_HEAVE = 0.5
MAX_SURGE = 0.5

# Search motion
SEARCH_YAW = 0.15

# Output
SAVE_OUTPUT_VIDEO = False
OUTPUT_VIDEO_PATH = "output_autonomous_test.mp4"

# =========================================================
# STATES
# =========================================================

SEARCH = "SEARCH"
ALIGN = "ALIGN"
APPROACH = "APPROACH"
HANDOVER = "HANDOVER"

# =========================================================
# HELPERS
# =========================================================

def clamp(value, min_val, max_val):
    return max(min(value, max_val), min_val)

def smooth(current, previous, alpha=0.6):
    if previous is None:
        return current
    return alpha * current + (1 - alpha) * previous

def choose_target(detections):
    if not detections:
        return None
    return max(detections, key=lambda d: d["area"])

def detect_objects(model, frame, conf_threshold):
    h, w = frame.shape[:2]
    results = model(frame, conf=conf_threshold, verbose=False)

    detections = []

    for r in results:
        if r.boxes is None:
            continue

        for box in r.boxes:
            cls_id = int(box.cls.item())
            conf = float(box.conf.item())
            x1, y1, x2, y2 = box.xyxy[0].tolist()

            x_center_px = (x1 + x2) / 2.0
            y_center_px = (y1 + y2) / 2.0
            bbox_w_px = x2 - x1
            bbox_h_px = y2 - y1

            x_center = x_center_px / w
            y_center = y_center_px / h
            bbox_w = bbox_w_px / w
            bbox_h = bbox_h_px / h
            area = bbox_w * bbox_h

            detections.append({
                "class_id": cls_id,
                "confidence": conf,
                "x1": int(x1),
                "y1": int(y1),
                "x2": int(x2),
                "y2": int(y2),
                "x_center": x_center,
                "y_center": y_center,
                "bbox_w": bbox_w,
                "bbox_h": bbox_h,
                "area": area
            })

    return detections

def compute_command(state, x_error, y_error, area, stable_count):
    surge = 0.0
    heave = 0.0
    yaw = 0.0
    action = "idle"

    if state == SEARCH:
        if stable_count >= REQUIRED_STABLE_FRAMES:
            return ALIGN, 0.0, 0.0, 0.0, "target_locked"
        else:
            return SEARCH, 0.0, 0.0, SEARCH_YAW, "search_scan"

    elif state == ALIGN:
        aligned_x = abs(x_error) < X_THRESHOLD
        aligned_y = abs(y_error) < Y_THRESHOLD

        if aligned_x and aligned_y:
            return APPROACH, 0.0, 0.0, 0.0, "aligned"

        yaw = clamp(K_YAW * x_error, -MAX_YAW, MAX_YAW)
        heave = clamp(K_HEAVE * y_error, -MAX_HEAVE, MAX_HEAVE)
        action = "aligning"
        return ALIGN, surge, heave, yaw, action

    elif state == APPROACH:
        if area >= AREA_CLOSE_THRESHOLD:
            return HANDOVER, 0.0, 0.0, 0.0, "close_enough_handover"

        yaw = clamp(K_YAW * x_error, -MAX_YAW, MAX_YAW)
        heave = clamp(K_HEAVE * y_error, -MAX_HEAVE, MAX_HEAVE)

        area_error = DESIRED_AREA - area
        surge = clamp(K_SURGE * area_error, 0.0, MAX_SURGE)

        action = "approaching"
        return APPROACH, surge, heave, yaw, action

    elif state == HANDOVER:
        return HANDOVER, 0.0, 0.0, 0.0, "handover_to_operator"

    return SEARCH, 0.0, 0.0, 0.0, "reset"

def draw_info(frame, state, action, target, x_error, y_error, area, surge, heave, yaw):
    h, w = frame.shape[:2]
    cx = w // 2
    cy = h // 2

    cv2.circle(frame, (cx, cy), 6, (255, 0, 0), -1)

    if target is not None:
        cv2.rectangle(frame, (target["x1"], target["y1"]), (target["x2"], target["y2"]), (0, 255, 0), 2)

        tx = int(target["x_center"] * w)
        ty = int(target["y_center"] * h)

        cv2.circle(frame, (tx, ty), 6, (0, 0, 255), -1)
        cv2.line(frame, (cx, cy), (tx, ty), (255, 255, 0), 2)

        cv2.putText(
            frame,
            f"Target detected conf={target['confidence']:.2f}",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )

        cv2.putText(
            frame,
            f"x_err={x_error:.3f} y_err={y_error:.3f} area={area:.4f}",
            (20, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2
        )
    else:
        cv2.putText(
            frame,
            "No object detected",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2
        )

    cv2.putText(frame, f"State: {state}", (20, 95),
                cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)

    cv2.putText(frame, f"Action: {action}", (20, 125),
                cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)

    cv2.putText(frame,
                f"surge={surge:.2f} heave={heave:.2f} yaw={yaw:.2f}",
                (20, 155), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 255, 200), 2)

def print_command(state, action, target, x_error, y_error, area, surge, heave, yaw):
    output = {
        "target_detected": target is not None,
        "state": state,
        "action": action,
        "command": {
            "surge": round(surge, 3),
            "heave": round(heave, 3),
            "yaw": round(yaw, 3)
        }
    }

    if target is not None:
        output["target"] = {
            "confidence": round(target["confidence"], 3),
            "x_error": round(x_error, 3),
            "y_error": round(y_error, 3),
            "area": round(area, 4)
        }

    print(output)

# =========================================================
# FRAME PROCESSOR
# =========================================================

class AutonomousTester:
    def __init__(self, model):
        self.model = model
        self.state = SEARCH
        self.stable_detection_count = 0
        self.lost_target_count = 0
        self.prev_x = None
        self.prev_y = None
        self.prev_area = None

    def process_frame(self, frame):
        detections = detect_objects(self.model, frame, CONF_THRESHOLD)
        target = choose_target(detections)

        if target is not None:
            self.lost_target_count = 0
            self.stable_detection_count += 1

            smoothed_x = smooth(target["x_center"], self.prev_x, ALPHA)
            smoothed_y = smooth(target["y_center"], self.prev_y, ALPHA)
            smoothed_area = smooth(target["area"], self.prev_area, ALPHA)

            self.prev_x = smoothed_x
            self.prev_y = smoothed_y
            self.prev_area = smoothed_area

            target["x_center"] = smoothed_x
            target["y_center"] = smoothed_y
            target["area"] = smoothed_area

            x_error = smoothed_x - 0.5
            y_error = smoothed_y - 0.5
            area = smoothed_area

            self.state, surge, heave, yaw, action = compute_command(
                self.state, x_error, y_error, area, self.stable_detection_count
            )

        else:
            self.stable_detection_count = 0
            self.lost_target_count += 1

            if self.lost_target_count >= LOST_TARGET_RESET_FRAMES:
                self.state = SEARCH
                self.prev_x = None
                self.prev_y = None
                self.prev_area = None

            x_error = 0.0
            y_error = 0.0
            area = 0.0

            if self.state == HANDOVER:
                surge, heave, yaw, action = 0.0, 0.0, 0.0, "handover_to_operator"
            else:
                surge, heave, yaw, action = 0.0, 0.0, SEARCH_YAW, "search_scan_no_target"

        draw_info(frame, self.state, action, target, x_error, y_error, area, surge, heave, yaw)
        print_command(self.state, action, target, x_error, y_error, area, surge, heave, yaw)

        return frame

# =========================================================
# RUN MODES
# =========================================================

def run_single_image(tester):
    frame = cv2.imread(IMAGE_PATH)
    if frame is None:
        raise FileNotFoundError(f"Could not read image: {IMAGE_PATH}")

    output = tester.process_frame(frame)

    cv2.imshow("Autonomous Test - Image", output)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def run_images_folder(tester):
    image_files = []
    for ext in ["*.jpg", "*.jpeg", "*.png", "*.bmp", "*.webp"]:
        image_files.extend(glob.glob(os.path.join(IMAGES_FOLDER, ext)))

    image_files.sort()

    if not image_files:
        raise FileNotFoundError(f"No images found in folder: {IMAGES_FOLDER}")

    for img_path in image_files:
        frame = cv2.imread(img_path)
        if frame is None:
            continue

        output = tester.process_frame(frame)
        cv2.imshow("Autonomous Test - Images", output)

        key = cv2.waitKey(0) & 0xFF
        if key == ord("q") or key == 27:
            break

    cv2.destroyAllWindows()

def run_video_or_webcam(tester, source):
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open source: {source}")

    writer = None

    if SAVE_OUTPUT_VIDEO:
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv2.CAP_PROP_FPS)
        if fps <= 1:
            fps = 20

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(OUTPUT_VIDEO_PATH, fourcc, fps, (width, height))

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        output = tester.process_frame(frame)

        if writer is not None:
            writer.write(output)

        cv2.imshow("Autonomous Test", output)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break

    cap.release()
    if writer is not None:
        writer.release()
    cv2.destroyAllWindows()

# =========================================================
# MAIN
# =========================================================

def main():
    model = YOLO(MODEL_PATH)
    tester = AutonomousTester(model)

    print("Model loaded.")
    print(f"Running mode: {MODE}")

    if MODE == "image":
        run_single_image(tester)
    elif MODE == "images":
        run_images_folder(tester)
    elif MODE == "video":
        run_video_or_webcam(tester, VIDEO_PATH)
    elif MODE == "webcam":
        run_video_or_webcam(tester, WEBCAM_INDEX)
    else:
        raise ValueError("MODE must be one of: image, images, video, webcam")

if __name__ == "__main__":
    main()