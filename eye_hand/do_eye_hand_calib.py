import yaml
import numpy as np
from pick_and_place_system import calibrate_hand_eye

# S'assurer que numpy est importé dans ce script aussi # Ensure numpy is also imported in this script

# 1) Charge les données brutes pour eye-to-hand # Load raw data for eye-to-hand
try:
    with open('hand_eye_data_eye_to_hand.yaml', 'r') as f:
        data = yaml.safe_load(f)
except FileNotFoundError:
    print("Error: hand_eye_data_eye_to_hand.yaml not found.")
    print("Please run calibration_hand_eye_data.py first to collect data.")
    exit()
except KeyError as e:
    print(f"Error: Missing key in hand_eye_data_eye_to_hand.yaml: {e}.")
    print("Ensure the data collection script saved the data correctly.")
    exit()


R_base2gripper_list = data['R_base2gripper_list']
t_base2gripper_list = data['t_base2gripper_list']
R_cam2target_list   = data['R_cam2target_list']
t_cam2target_list   = data['t_cam2target_list']


print(f"Nombre de poses collectées: {len(R_base2gripper_list)}")

if len(R_base2gripper_list) < 5:
     print("Warning: Less than 5 poses collected. Calibration may be inaccurate.")


# 2) Préparer les données pour calibrateHandEye (AX=XB) - EYE-TO-HAND # Prepare data for calibrateHandEye (AX=XB) - EYE-TO-HAND
# For eye-to-hand: A = T_gripper2base, B = T_target2cam, X = T_base2camera # For eye-to-hand: A = T_gripper2base, B = T_target2cam, X = T_base2camera

# Convert A from T_base2gripper to T_gripper2base # Convert A from T_base2gripper to T_gripper2base
R_gripper2base_list = []
t_gripper2base_list = []

for R_bg, t_bg in zip(R_base2gripper_list, t_base2gripper_list):
    T_bg = np.eye(4)
    T_bg[:3, :3] = np.array(R_bg)
    T_bg[:3, 3] = np.array(t_bg).flatten()

    T_gb = np.linalg.inv(T_bg)  # Invert to get T_gripper2base

    R_gripper2base_list.append(T_gb[:3, :3].tolist())
    t_gripper2base_list.append(T_gb[:3, 3].tolist())

# B = T_target2cam (same as before) # B = T_target2cam (same as before)
R_target2cam_list_for_calib = []
t_target2cam_list_for_calib = []

for R_ct, t_ct in zip(R_cam2target_list, t_cam2target_list):
    T_ct = np.eye(4)
    T_ct[:3, :3] = np.array(R_ct)
    T_ct[:3, 3] = np.array(t_ct).flatten()

    T_tc = np.linalg.inv(T_ct)  # Invert T_cam2target to get T_target2cam

    R_target2cam_list_for_calib.append(T_tc[:3, :3].tolist())
    t_target2cam_list_for_calib.append(T_tc[:3, 3].tolist())

# 3) Calibre en mode Tsai–Lenz (AX=XB, X is T_base2camera for eye-to-hand) # Calibrate in Tsai–Lenz mode (AX=XB, X is T_base2camera for eye-to-hand)
# A = T_gripper2base, B = T_target2cam, X = T_base2camera # A = T_gripper2base, B = T_target2cam, X = T_base2camera
T_base2camera = calibrate_hand_eye(
    R_gripper2base_list,         # A (Gripper->Base)
    t_gripper2base_list,         # A (Gripper->Base)
    R_target2cam_list_for_calib, # B (Target->Camera)
    t_target2cam_list_for_calib, # B (Target->Camera)
    'TF_matrixs.yaml'
)

# Affichage des résultats # Display results
print("\nTransformation base → caméra (Eye-to-Hand):")
print(f"Translation: X={T_base2camera[0,3]:.4f}, Y={T_base2camera[1,3]:.4f}, Z={T_base2camera[2,3]:.4f}")
print("Matrice de rotation:")
print(T_base2camera[:3,:3])
