import cv2
import numpy as np
import yaml
import pyrealsense2 as rs
import rtde_receive
import time
from pick_and_place_system import ArucoDetector

# --- Configuration ---
ROBOT_IP = "10.2.30.60"
# Initialize ArucoDetector with intrinsics.yaml
try:
    detector = ArucoDetector('intrinsics.yaml')
except FileNotFoundError:
    print("Error: intrinsics.yaml not found. Please run camera intrinsic calibration first.")
    exit()
except Exception as e:
    print(f"Error initializing ArucoDetector: {e}")
    exit()


try:
    rtde_recv = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
except Exception as e:
    print(f"Error initializing RTDE interface: {e}")
    print("Make sure the robot is reachable at IP address {ROBOT_IP} and RTDE is enabled.")
    exit()


# --- Initialisation RealSense ---
pipeline = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
try:
    pipeline.start(cfg)
except Exception as e:
    print(f"Error starting RealSense pipeline: {e}")
    print("Make sure the camera is connected and not in use by another process.")
    exit()


# --- Listes pour stocker les poses (for eye-to-hand) ---
# A = T_base2gripper (robot motion)
# B = T_cam2target (camera motion relative to target)
R_base2gripper_list = []
t_base2gripper_list = []
R_cam2target_list = []
t_cam2target_list = []


# --- Instructions utilisateur ---
print("\n=== Eye–to–hand calibration data collection ===\n")
print("CONSEILS POUR UNE BONNE CALIBRATION:")
print("1. Déplacez le robot pour voir le CharucoBoard sous différents angles")
print("2. Collectez au moins 20 positions différentes")
print("3. Variez les rotations ET translations du robot")
print("4. Assurez-vous que la planche est bien détectée à chaque capture\n")

print("COMMANDES:")
print("  'c' - capturer un échantillon")
print("  'd' - supprimer le dernier échantillon")
print("  's' - sauvegarder et quitter")
print("  'q' - quitter sans sauvegarder\n")

cv2.namedWindow("Calibration Capture", cv2.WINDOW_NORMAL)

try:
    sample_count = 0
    last_capture_time = 0

    while True:
        frames = pipeline.wait_for_frames()
        color = frames.get_color_frame()
        img = np.asanyarray(color.get_data())

        # Détection de la planche ArUco
        # det.detect returns T_target2cam (rvec, tvec)
        ok, rvec_target_cam, tvec_target_cam = detector.detect(img)
        display_img = detector.draw_detection(img)

        # Affichage nombre d'échantillons et messages
        cv2.putText(display_img, f"Echantillons: {sample_count}/20+", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        if ok:
            # Draw axes for target in camera frame
            cv2.drawFrameAxes(display_img, detector.camera_matrix, detector.dist_coeffs,
                              rvec_target_cam, tvec_target_cam, 0.05) # axis length in meters

            # Display pose of target in camera frame (T_target2cam)
            position_text_cam = f"Target in Cam: X:{tvec_target_cam[0][0]:.3f} Y:{tvec_target_cam[1][0]:.3f} Z:{tvec_target_cam[2][0]:.3f}"
            cv2.putText(display_img, position_text_cam, (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            cv2.putText(display_img, "DETECTION OK", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
             cv2.putText(display_img, "DETECTION INCOMPLETE", (10, 60),
                         cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)


        # Instructions de commande clavier
        y_offset = 120
        for cmd, desc in zip(['c', 'd', 's', 'q'],
                             ["capturer", "supprimer dernier", "sauvegarder & quitter", "quitter sans sauvegarder"]):
            cv2.putText(display_img, f"{cmd} - {desc}", (10, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y_offset += 25

        # Affichage position actuelle du TCP robot (T_base2gripper)
        tcp = rtde_recv.getActualTCPPose() # This is T_base2gripper
        cv2.putText(display_img, f"TCP (Base): X={tcp[0]:.3f} Y={tcp[1]:.3f} Z={tcp[2]:.3f}", (10, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        y_offset += 25
        cv2.putText(display_img, f"Rot (Base): Rx={tcp[3]:.3f} Ry={tcp[4]:.3f} Rz={tcp[5]:.3f}", (10, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)


        # Affichage de l’image
        cv2.imshow("Calibration Capture", display_img)
        key = cv2.waitKey(1) & 0xFF
        current_time = time.time()

        # --- Capture échantillon ---
        if key == ord('c') and ok and (current_time - last_capture_time > 1.0):
            # Robot pose: T_base2gripper (this is what getActualTCPPose returns)
            # We need R_base2gripper and t_base2gripper
            R_base2gripper, _ = cv2.Rodrigues(np.array(tcp[3:6]))
            t_base2gripper = np.array(tcp[:3])

            # Camera pose relative to target: T_cam2target
            # We detected T_target2cam, so we need its inverse.
            R_target2cam, _ = cv2.Rodrigues(rvec_target_cam)
            t_target2cam = tvec_target_cam.flatten()
            T_target2cam = np.eye(4)
            T_target2cam[:3,:3] = R_target2cam
            T_target2cam[:3,3] = t_target2cam

            T_cam2target = np.linalg.inv(T_target2cam)
            R_cam2target = T_cam2target[:3,:3]
            t_cam2target = T_cam2target[:3,3]


            R_base2gripper_list.append(R_base2gripper.tolist())
            t_base2gripper_list.append(t_base2gripper.tolist())
            R_cam2target_list.append(R_cam2target.tolist())
            t_cam2target_list.append(t_cam2target.tolist())

            sample_count += 1
            last_capture_time = current_time
            print(f"[+] Échantillon {sample_count} capturé")

        # --- Supprimer dernier échantillon ---
        elif key == ord('d') and sample_count > 0:
            R_base2gripper_list.pop()
            t_base2gripper_list.pop()
            R_cam2target_list.pop()
            t_cam2target_list.pop()
            sample_count -= 1
            print(f"[-] Échantillon supprimé, reste {sample_count}")

        # --- Sauvegarder et quitter ---
        elif key == ord('s'):
            if sample_count >= 5:
                break
            else:
                print("Collectez au moins 5 échantillons avant de sauvegarder")

        # --- Quitter sans sauvegarder ---
        elif key == ord('q'):
            if sample_count > 0:
                confirm = input("Quitter sans sauvegarder? (o/N): ").lower()
                if confirm == 'o':
                    print("Sortie sans sauvegarde")
                    exit(0)
            else:
                print("Sortie sans sauvegarde")
                exit(0)

finally:
    pipeline.stop()
    cv2.destroyAllWindows()

# --- Vérification et sauvegarde YAML ---
if sample_count < 5:
    print("\nPas assez d'échantillons collectés (minimum 5). Données non sauvegardées.")
    exit(1)

# Save data for eye-to-hand calibration
data = {
    'R_base2gripper_list': R_base2gripper_list, # A in AX=XB
    't_base2gripper_list': t_base2gripper_list, # A in AX=XB
    'R_cam2target_list': R_cam2target_list,   # B in AX=XB
    't_cam2target_list': t_cam2target_list    # B in AX=XB
}

with open('hand_eye_data_eye_to_hand.yaml', 'w') as f:
    yaml.dump(data, f)

print(f"\nCalibration data saved to hand_eye_data_eye_to_hand.yaml ({sample_count} échantillons)")
print("\nPour effectuer la calibration, exécutez maintenant le script do_hand_eye_calib.py")
