import cv2
import numpy as np
import yaml
import pyrealsense2 as rs
import time
from pick_and_place_system import ArucoDetector

# --- Configuration ---
# Assuming intrinsics_v2.yaml was used for intrinsic calibration
INTRINSICS_YAML = 'intrinsics_v2.yaml'
# Assuming TF_matrixs.yaml contains the T_base2camera transform
TF_MATRIX_YAML = 'TF_matrixs.yaml'

# Define an offset in the robot's base frame (in meters)
# You can adjust these values to correct for systematic errors.
OFFSET_X = -1.1098  # Offset in X direction
OFFSET_Y = -0.073  # Offset in Y direction
OFFSET_Z = 0.1215  # Offset in Z direction
BASE_FRAME_OFFSET = np.array([OFFSET_X, OFFSET_Y, OFFSET_Z], dtype=np.float64)


# Initialize ArucoDetector with intrinsics
try:
    detector = ArucoDetector(INTRINSICS_YAML)
    print(f"ArucoDetector initialized with {INTRINSICS_YAML}")
except FileNotFoundError:
    print(f"Error: {INTRINSICS_YAML} not found. Please run camera intrinsic calibration first.")
    exit()
except Exception as e:
    print(f"Error initializing ArucoDetector: {e}")
    exit()

# Load the Base to Camera transform (T_base2camera)
T_base2camera = None
try:
    with open(TF_MATRIX_YAML, 'r') as f:
        tf_data = yaml.safe_load(f)
        # Assuming the key used in do_hand_eye_calib.py is 'T_calibrated_transform'
        if 'T_calibrated_transform' in tf_data:
             T_base2camera = np.array(tf_data['T_calibrated_transform'], dtype=np.float64)
             print(f"Loaded T_base2camera transform from {TF_MATRIX_YAML}")
             print("T_base2camera:")
             print(T_base2camera)
        else:
             print(f"Error: 'T_calibrated_transform' key not found in {TF_MATRIX_YAML}.")
             print("Please ensure do_hand_eye_calib.py saved the matrix with this key.")
             exit()
except FileNotFoundError:
    print(f"Error: {TF_MATRIX_YAML} not found.")
    print("Please run calibration_eye_hand_data.py and do_hand_eye_calib.py first.")
    exit()
except Exception as e:
    print(f"An error occurred loading {TF_MATRIX_YAML}: {e}")
    exit()


# Calculate the inverse transform (Camera to Base) which will be used frequently
T_camera2base = np.linalg.inv(T_base2camera)
print("Calculated T_camera2base:")
print(T_camera2base)


# --- Initialisation RealSense ---
pipeline = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
try:
    pipeline.start(cfg)
    print("RealSense pipeline started.")
except Exception as e:
    print(f"Error starting RealSense pipeline: {e}")
    print("Make sure the camera is connected and not in use by another process.")
    exit()

cv2.namedWindow("Target Pose in Base Frame", cv2.WINDOW_NORMAL)

print("\nShowing camera feed. Press 'Esc' to quit.")

try:
    while True:
        frames = pipeline.wait_for_frames()
        color = frames.get_color_frame()
        if not color:
            continue

        img = np.asanyarray(color.get_data())

        # Detect the Charuco board
        # det.detect returns T_target2cam (rvec, tvec)
        ok, rvec_target_cam, tvec_target_cam = detector.detect(img)
        display_img = img.copy() # Draw on a copy

        # Draw detections (markers and charuco corners)
        display_img = detector.draw_detection(display_img)


        if ok:
            # Ensure rvec and tvec are valid before proceeding
            if rvec_target_cam is None or tvec_target_cam is None:
                 print("Warning: Detection OK but rvec or tvec is None!")
                 cv2.putText(display_img, "DETECTION ERROR", (10, 60),
                             cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                 ok = False # Treat as not OK if pose is invalid
            else:
                # Draw axes for target in camera frame
                cv2.drawFrameAxes(display_img, detector.camera_matrix, detector.dist_coeffs,
                                  rvec_target_cam, tvec_target_cam, 0.05) # axis length in meters

                # Display pose of target in camera frame (T_target2cam)
                position_text_cam = f"Target in Cam: X:{tvec_target_cam[0][0]:.3f} Y:{tvec_target_cam[1][0]:.3f} Z:{tvec_target_cam[2][0]:.3f}"
                cv2.putText(display_img, position_text_cam, (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                # --- Calculate Target Pose in Robot Base Frame ---
                try:
                    # Build T_target2cam homogeneous transform
                    R_tc, _ = cv2.Rodrigues(rvec_target_cam)
                    T_tc = np.eye(4); T_tc[:3,:3] = R_tc; T_tc[:3,3] = tvec_target_cam.flatten()

                    # Calculate T_target2base = T_camera2base @ T_target2cam
                    T_target2base = T_camera2base @ T_tc

                    # Apply the defined offset to the translation part in the base frame
                    pos_target_base = T_target2base[:3, 3] + BASE_FRAME_OFFSET

                    R_target2base = T_target2base[:3,:3]
                    rvec_target2base, _ = cv2.Rodrigues(R_target2base)

                    # Display pose of target in base frame (with offset applied to position)
                    position_text_base = f"Target in Base (Trans + Offset): X:{pos_target_base[0]:.3f} Y:{pos_target_base[1]:.3f} Z:{pos_target_base[2]:.3f}"
                    orientation_text_base = f"Target in Base (Rot): Rx:{rvec_target2base[0][0]:.3f} Ry:{rvec_target2base[1][0]:.3f} Rz:{rvec_target2base[2][0]:.3f}"

                    cv2.putText(display_img, "TARGET DETECTED", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(display_img, position_text_base, (10, 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                    cv2.putText(display_img, orientation_text_base, (10, 120),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

                     # Display the original calculated position for comparison
                    original_pos_text = f"Original Base Pos: X:{T_target2base[0,3]:.3f} Y:{T_target2base[1,3]:.3f} Z:{T_target2base[2,3]:.3f}"
                    cv2.putText(display_img, original_pos_text, (10, 150),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)


                except Exception as e:
                     cv2.putText(display_img, f"Base frame calc error: {e}", (10, 90),
                                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)


        else:
            # Indicate incomplete detection
            detected_markers = len(detector.corners) if detector.corners is not None else 0
            cv2.putText(display_img, f"Markers: {detected_markers}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(display_img, "DETECTION INCOMPLETE", (10, 60),
                         cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)


        # Affichage de l’image
        cv2.imshow("Target Pose in Base Frame", display_img)
        key = cv2.waitKey(1) & 0xFF

        if key == 27: # ESC key
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    print("Camera stream stopped.")
