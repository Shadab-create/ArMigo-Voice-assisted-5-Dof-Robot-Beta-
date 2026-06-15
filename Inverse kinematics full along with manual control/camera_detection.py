import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import pickle

# -----------------------------
# Load camera calibration
# -----------------------------
with open("cameraMatrix.pkl", "rb") as f:
    camera_matrix = pickle.load(f)
with open("dist.pkl", "rb") as f:
    dist_coeffs = pickle.load(f)

# -----------------------------
# Initialize camera
# -----------------------------
def open_camera(max_index=5):
    for i in range(max_index):
        cap = cv.VideoCapture(i, cv.CAP_DSHOW)
        if cap.isOpened():
            print(f"[Camera] Opened at index {i}")
            return cap
    raise RuntimeError("[Camera] No camera found!")

cap = open_camera()

# -----------------------------
# ArUco setup
# -----------------------------
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
parameters = aruco.DetectorParameters()

if hasattr(aruco, "ArucoDetector"):
    detector = aruco.ArucoDetector(aruco_dict, parameters)
else:
    detector = None

# Marker configuration
marker_sizes = {0: 0.08, 4: 0.08}  # Marker 0 (base), Marker 4 (object)
default_marker_length = 0.08

# IDs
base_id = 0
object_id = 4

# Robot base offset relative to Marker 0 (in cm)
robot_offset = np.array([33.5, 43.5, 0.0])

# -----------------------------
# Drawing helper
# -----------------------------
def draw_axis_manual(frame, camera_matrix, dist_coeffs, rvec, tvec, length=0.03):
    axis_points = np.float32([
        [0, 0, 0],
        [length, 0, 0],
        [0, length, 0],
        [0, 0, length]
    ])
    imgpts, _ = cv.projectPoints(axis_points, rvec, tvec, camera_matrix, dist_coeffs)
    imgpts = np.int32(imgpts).reshape(-1, 2)
    cv.line(frame, tuple(imgpts[0]), tuple(imgpts[1]), (0, 0, 255), 2)
    cv.line(frame, tuple(imgpts[0]), tuple(imgpts[2]), (0, 255, 0), 2)
    cv.line(frame, tuple(imgpts[0]), tuple(imgpts[3]), (255, 0, 0), 2)

# -----------------------------
# Smoothing buffers
# -----------------------------
base_history = []
object_history = []

def smooth_base(origin, R_base, max_history=5):
    global base_history
    base_history.append((origin, R_base))
    if len(base_history) > max_history:
        base_history.pop(0)
    origins = np.array([b[0] for b in base_history])
    Rs = np.array([b[1] for b in base_history])
    return np.mean(origins, axis=0), np.mean(Rs, axis=0)

def smooth_object(P_cm, alpha=0.5):
    global object_history
    if len(object_history) == 0:
        object_history.append(P_cm)
        return P_cm
    P_smooth = alpha * P_cm + (1 - alpha) * object_history[-1]
    object_history.append(P_smooth)
    if len(object_history) > 10:
        object_history.pop(0)
    return P_smooth

# -----------------------------
# Main computation
# -----------------------------
def get_object_coords():
    ret, frame = cap.read()
    if not ret:
        print("[Camera] Failed to grab frame.")
        return None, None, frame

    frame_undistorted = cv.undistort(frame, camera_matrix, dist_coeffs)
    gray = cv.cvtColor(frame_undistorted, cv.COLOR_BGR2GRAY)

    if detector:
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is None:
        print("[Camera] No markers detected")
        return None, None, frame_undistorted

    ids = ids.flatten()
    tvecs = {}
    rvecs = {}

    # Pose estimation for detected markers
    for i, marker_id in enumerate(ids):
        marker_length = marker_sizes.get(marker_id, default_marker_length)
        obj_pts = np.array([
            [0, 0, 0],
            [marker_length, 0, 0],
            [marker_length, marker_length, 0],
            [0, marker_length, 0]
        ], dtype=np.float32)
        img_pts = corners[i].reshape(-1, 2).astype(np.float32)
        success, rvec, tvec = cv.solvePnP(obj_pts, img_pts, camera_matrix, dist_coeffs)
        if success:
            tvecs[marker_id] = tvec.reshape(3,)
            rvecs[marker_id] = rvec.reshape(3,)
            color = (0, 255, 0) if marker_id == object_id else (0, 0, 255)
            cv.polylines(frame_undistorted, [corners[i].astype(np.int32)], True, color, 2)
            draw_axis_manual(frame_undistorted, camera_matrix, dist_coeffs, rvec, tvec)

    if base_id not in tvecs:
        print("[Camera] Base marker not detected")
        return None, None, frame_undistorted

    # -----------------------------
    # Base frame (Marker 0)
    # -----------------------------
    R_base, _ = cv.Rodrigues(rvecs[base_id])
    t_base = tvecs[base_id]
    origin_base_cm = t_base * 100.0
    origin_base_cm[2] = 0.0  # Force Z = 0
    origin_base_cm, R_base = smooth_base(origin_base_cm, R_base)

    # -----------------------------
    # Robot frame (fixed offset from Marker 0)
    # -----------------------------
    robot_position_cm = robot_offset

    # -----------------------------
    # Object position relative to robot (2D)
    # -----------------------------
    object_relative_cm = None
    if object_id in tvecs:
        P_obj_cam = tvecs[object_id]
        # Transform to Marker0 frame
        P_obj_base = np.dot(R_base.T, (P_obj_cam - t_base)) * 100.0
        # Relative to robot
        object_relative_cm = P_obj_base - robot_offset
        object_relative_cm[2] = 0.0  # Force Z = 0 for 2D
        object_relative_cm = smooth_object(object_relative_cm)

    # -----------------------------
    # Display info
    # -----------------------------
    cv.putText(frame_undistorted,
               f"Base frame (cm): X={origin_base_cm[0]:.1f}, Y={origin_base_cm[1]:.1f}, Z={origin_base_cm[2]:.1f}",
               (30, 40), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    cv.putText(frame_undistorted,
               f"Robot frame (cm): X={robot_position_cm[0]:.1f}, Y={robot_position_cm[1]:.1f}, Z={robot_position_cm[2]:.1f}",
               (30, 70), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    if object_relative_cm is not None:
        cv.putText(frame_undistorted,
                   f"Object rel to robot (cm): X={object_relative_cm[0]:.1f}, Y={object_relative_cm[1]:.1f}, Z={object_relative_cm[2]:.1f}",
                   (30, 100), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

    # Print
    print(f"Base frame (cm): X={origin_base_cm[0]:.1f}, Y={origin_base_cm[1]:.1f}, Z={origin_base_cm[2]:.1f}")
    print(f"Robot frame (cm): X={robot_position_cm[0]:.1f}, Y={robot_position_cm[1]:.1f}, Z={robot_position_cm[2]:.1f}")
    if object_relative_cm is not None:
        print(f"Object rel to robot (cm): X={object_relative_cm[0]:.1f}, Y={object_relative_cm[1]:.1f}, Z={object_relative_cm[2]:.1f}")

    return origin_base_cm, object_relative_cm, frame_undistorted

# -----------------------------
# Cleanup
# -----------------------------
def close_camera():
    if cap and cap.isOpened():
        cap.release()
        print("[Camera] Released successfully.")
