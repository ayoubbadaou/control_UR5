import yaml
import numpy as np
from pick_and_place_system import calibrate_hand_eye

def main():
    # 1) Load raw data for eye-to-hand
    try:
        with open('Eyetohand_data.yaml', 'r') as f:
            data = yaml.safe_load(f)
    except FileNotFoundError:
        print("Error: hand_eye_data_eye_to_hand.yaml not found.")
        print("Please run calibration_hand_eye_data.py first to collect data.")
        return
    except KeyError as e:
        print(f"Error: Missing key in hand_eye_data_eye_to_hand.yaml: {e}.")
        print("Ensure the data collection script saved the data correctly.")
        return

    R_base2gripper_list = data['R_base2gripper_list']
    t_base2gripper_list = data['t_base2gripper_list']
    R_target2cam_list = data['R_target2cam_list']
    t_target2cam_list = data['t_target2cam_list']

    print(f"Number of poses collected: {len(R_base2gripper_list)}")

    if len(R_base2gripper_list) < 5:
        print("Warning: Less than 5 poses collected. Calibration may be inaccurate.")

    # 2) Prepare data for calibrateHandEye (AX=XB) - EYE-TO-HAND
    # For eye-to-hand calibration using OpenCV's calibrateHandEye:
    # A = T_gripper2base (robot motion)
    # B = T_target2cam (camera observation of target motion)
    # X = T_base2camera (what we want to find)
    
    # Convert A from T_base2gripper to T_gripper2base
    R_gripper2base_list = []
    t_gripper2base_list = []

    print("Processing robot poses (A = T_gripper2base)...")
    for R_bg, t_bg in zip(R_base2gripper_list, t_base2gripper_list):
        # Create homogeneous transformation T_base2gripper
        T_bg = np.eye(4)
        T_bg[:3, :3] = np.array(R_bg)
        T_bg[:3, 3] = np.array(t_bg).flatten()
        
        # Invert to get T_gripper2base
        T_gb = np.linalg.inv(T_bg)
        
        R_gripper2base_list.append(T_gb[:3, :3])
        t_gripper2base_list.append(T_gb[:3, 3])

    # B = T_target2cam (use directly from detection)
    print("Using camera observations (B = T_target2cam)...")
    R_target2cam_list_for_calib = [np.array(R) for R in R_target2cam_list]
    t_target2cam_list_for_calib = [np.array(t).reshape(3, 1) for t in t_target2cam_list]

    # 3) Perform hand-eye calibration
    print("Performing hand-eye calibration...")
    T_base2camera = calibrate_hand_eye(
        R_gripper2base_list,          # A (Gripper->Base) - robot motion
        t_gripper2base_list,          # A (Gripper->Base) - robot motion  
        R_target2cam_list_for_calib,  # B (Target->Camera) - camera observations
        t_target2cam_list_for_calib,  # B (Target->Camera) - camera observations
        'TF_matrixs.yaml'
    )

    # Display results
    print("\n=== EYE-TO-HAND CALIBRATION RESULTS ===")
    print("Transformation: Base → Camera (T_base2camera)")
    print(f"Translation: X={T_base2camera[0,3]:.4f}m, Y={T_base2camera[1,3]:.4f}m, Z={T_base2camera[2,3]:.4f}m")
    print("Rotation matrix:")
    print(np.array2string(T_base2camera[:3,:3], precision=4, suppress_small=True))
    
    # Also show as rotation vector for easier interpretation
    rvec, _ = cv2.Rodrigues(T_base2camera[:3,:3])
    print(f"Rotation vector: Rx={rvec[0][0]:.4f}, Ry={rvec[1][0]:.4f}, Rz={rvec[2][0]:.4f}")
    
    # Calculate the inverse transformation (camera to base) for reference
    T_camera2base = np.linalg.inv(T_base2camera)
    print(f"\nInverse transformation: Camera → Base")
    print(f"Translation: X={T_camera2base[0,3]:.4f}m, Y={T_camera2base[1,3]:.4f}m, Z={T_camera2base[2,3]:.4f}m")

if __name__ == "__main__":
    # Import cv2 here to avoid dependency issues in the main library
    import cv2
    main()
