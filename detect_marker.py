import cv2
import numpy as np
import yaml
import pyrealsense2 as rs
from pick_and_place_system import ArucoDetector

def rotation_matrix_to_euler_angles(R):
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0
    return np.array([x, y, z])

# --- Configuration ---
INTRINSICS_FILE = 'intrinsics.yaml'
EYE_TO_HAND_CALIB_FILE = 'TF_matrixs.yaml'

# Single marker parameters
ARUCO_DICT = cv2.aruco.DICT_4X4_50  # Adjust based on your marker type
MARKER_ID = 1  # ID of the marker you want to detect
MARKER_LENGTH = 0.011  # Length of marker side in meters (adjust to your marker size)

# --- Load Camera Intrinsics and Eye-to-Hand Transform ---
try:
    detector = ArucoDetector(INTRINSICS_FILE)
    camera_matrix = detector.camera_matrix
    dist_coeffs = detector.dist_coeffs
except FileNotFoundError:
    print(f"Error: {INTRINSICS_FILE} not found. Please run camera intrinsic calibration.")
    exit()
except Exception as e:
    print(f"Error initializing ArucoDetector: {e}")
    exit()

try:
    with open(EYE_TO_HAND_CALIB_FILE, 'r') as f:
        calib_data = yaml.safe_load(f)
        if 'T_calibrated_transform' in calib_data:
            T_base2camera = np.array(calib_data['T_calibrated_transform'])
        elif 'T_base_to_camera' in calib_data:
            T_base2camera = np.array(calib_data['T_base_to_camera'])
        elif 'T_base2camera' in calib_data:
            T_base2camera = np.array(calib_data['T_base2camera'])
        else:
            print(f"Error: Could not find transformation key in {EYE_TO_HAND_CALIB_FILE}")
            exit()
except FileNotFoundError:
    print(f"Error: {EYE_TO_HAND_CALIB_FILE} not found. Please run eye-to-hand calibration.")
    exit()
except Exception as e:
    print(f"Error loading eye-to-hand transform: {e}")
    exit()

print("Successfully loaded camera intrinsics and eye-to-hand transform.")

# --- Initialize ArUco detector for single marker ---
aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
parameters = cv2.aruco.DetectorParameters()

print(f"Looking for single ArUco marker with ID: {MARKER_ID}")

# --- Initialize RealSense Camera ---
pipeline = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
try:
    pipeline.start(cfg)
except Exception as e:
    print(f"Error starting RealSense pipeline: {e}")
    print("Make sure the camera is connected and not in use by another process.")
    exit()

print("RealSense pipeline started.")

# --- Detection and Transformation ---
cv2.namedWindow("Single Marker Detection", cv2.WINDOW_NORMAL)

def detect_single_marker(image, marker_id, camera_matrix, dist_coeffs, marker_length):
    """
    Detect a specific single ArUco marker and return its pose
    """
    corners, ids, rejected = cv2.aruco.detectMarkers(image, aruco_dict, parameters=parameters)
    
    if ids is not None:
        # Find the specific marker we're looking for
        if marker_id in ids:
            idx = np.where(ids == marker_id)[0][0]
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                [corners[idx]], marker_length, camera_matrix, dist_coeffs
            )
            return True, rvec[0][0], tvec[0][0], corners[idx]
    
    return False, None, None, None

try:
    print(f"\nLooking for single ArUco marker (ID: {MARKER_ID})...")
    while True:
        frames = pipeline.wait_for_frames()
        color = frames.get_color_frame()
        img = np.asanyarray(color.get_data())

        # Detect single marker
        ok, rvec_target_cam, tvec_target_cam, corners = detect_single_marker(
            img, MARKER_ID, camera_matrix, dist_coeffs, MARKER_LENGTH
        )
        
        display_img = img.copy()

        if ok:
            # Draw detection
            cv2.aruco.drawDetectedMarkers(display_img, [corners], np.array([[MARKER_ID]]))
            cv2.drawFrameAxes(display_img, camera_matrix, dist_coeffs, 
                            rvec_target_cam, tvec_target_cam, MARKER_LENGTH/2)

            # Ensure proper array shapes
            rvec_target_cam = np.array(rvec_target_cam).flatten()
            tvec_target_cam = np.array(tvec_target_cam).flatten()

            # Create T_target2cam transformation matrix
            R_target2cam, _ = cv2.Rodrigues(rvec_target_cam)
            T_target2cam = np.eye(4)
            T_target2cam[:3, :3] = R_target2cam
            T_target2cam[:3, 3] = tvec_target_cam

            # Transform target pose to robot base frame
            T_base2target = T_base2camera @ T_target2cam

            # Extract position and rotation
            pos_base = T_base2target[:3, 3]
            
            # Convert rotation matrix to Euler angles
            R_base2target = T_base2target[:3, :3]
            rot_base_rad = rotation_matrix_to_euler_angles(R_base2target)
            rot_base_deg = np.degrees(rot_base_rad)

            # Also get camera frame values for display
            pos_cam = tvec_target_cam
            R_target_cam, _ = cv2.Rodrigues(rvec_target_cam)
            rot_cam_rad = rotation_matrix_to_euler_angles(R_target_cam)
            rot_cam_deg = np.degrees(rot_cam_rad)

            # Display results
            text_cam_pos = f"Marker in Cam (Pos): X:{pos_cam[0]:.4f} Y:{pos_cam[1]:.4f} Z:{pos_cam[2]:.4f}"
            text_cam_rot = f"Marker in Cam (Rot): Rx:{rot_cam_deg[0]:.2f} Ry:{rot_cam_deg[1]:.2f} Rz:{rot_cam_deg[2]:.2f} deg"
            text_base_pos = f"Marker in Base (Pos): X:{pos_base[0]:.4f} Y:{pos_base[1]:.4f} Z:{pos_base[2]:.4f}"
            text_base_rot = f"Marker in Base (Rot): Rx:{rot_base_deg[0]:.2f} Ry:{rot_base_deg[1]:.2f} Rz:{rot_base_deg[2]:.2f} deg"
            text_marker_id = f"Marker ID: {MARKER_ID}"

            cv2.putText(display_img, text_marker_id, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
            cv2.putText(display_img, text_cam_pos, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(display_img, text_cam_rot, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(display_img, text_base_pos, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(display_img, text_base_rot, (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(display_img, "MARKER DETECTED", (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        else:
            # Show all detected markers (for debugging)
            corners, ids, rejected = cv2.aruco.detectMarkers(img, aruco_dict, parameters=parameters)
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(display_img, corners, ids)
                detected_ids = ", ".join(map(str, ids.flatten()))
                cv2.putText(display_img, f"Detected IDs: {detected_ids}", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
                cv2.putText(display_img, f"Looking for ID: {MARKER_ID}", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
            else:
                cv2.putText(display_img, "NO MARKERS DETECTED", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow('Single Marker Detection', display_img)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # Press 'Esc' to exit
            break
        elif key == ord(' '):  # Space bar to print current pose
            if ok:
                print(f"\nMarker {MARKER_ID} Pose in Base Frame:")
                print(f"Position: X={pos_base[0]:.4f}, Y={pos_base[1]:.4f}, Z={pos_base[2]:.4f}")
                print(f"Rotation: Rx={rot_base_deg[0]:.2f}, Ry={rot_base_deg[1]:.2f}, Rz={rot_base_deg[2]:.2f} deg")

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    print("Camera stopped.")
