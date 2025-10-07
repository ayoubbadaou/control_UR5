"""
pick_and_place_system.py - Capture et traite les image pour la calibration.
Renvoie deux fichiers yaml : hand_eye_data.yaml et intrinsics.yaml

Auteur : Alban CASELLA et Suzanne-Léonore GIRARD-JOLLET
Date : Juin 2025
Description : Ce script utilise la caméra pour capturer des images d'une grille d'ArUco pour calibrer la caméra et déterminer les paramètres intrasects de la caméra.
"""
import cv2
from cv2 import aruco
import numpy as np
import yaml
import glob
import pyrealsense2 as rs

# Hand-eye calibration and camera intrinsic calibration utilities
def calibrate_camera_intrinsics(image_files, charuco_board, aruco_dict, output_yaml):
    all_corners = []
    all_ids = []
    image_size = None
    for fname in glob.glob(image_files):
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict)
        if len(corners) > 0:
            ret, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                corners, ids, gray, charuco_board)
            if charuco_corners is not None and charuco_ids is not None and len(charuco_corners) > 3:
                all_corners.append(charuco_corners)
                all_ids.append(charuco_ids)
                image_size = gray.shape[::-1]
    # calibrate
    ret, camera_matrix, dist_coeffs, _, _ = cv2.aruco.calibrateCameraCharuco(
        all_corners, all_ids, charuco_board, image_size, None, None)
    # save
    with open(output_yaml, 'w') as f:
        yaml.dump({'camera_matrix': camera_matrix.tolist(),
                    'dist_coeffs': dist_coeffs.tolist()}, f)
    print(f"Intrinsics saved to {output_yaml}")

    print(f"Erreur de reprojection : {ret:.4f}")

    return camera_matrix, dist_coeffs


def calibrate_hand_eye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, output_yaml):
    """
    Calibration hand-eye avec la méthode de Tsai.

    Args:
        R_gripper2base: Liste des matrices de rotation A (base->gripper for eye-to-hand, gripper->base for hand-in-eye)
        t_gripper2base: Liste des vecteurs de translation A
        R_target2cam: Liste des matrices de rotation B (target->cam for hand-in-eye, cam->target for eye-to-hand)
        t_target2cam: Liste des vecteurs de translation B
        output_yaml: Chemin où sauvegarder la transformation (T_cam->flange for hand-in-eye, T_base->cam for eye-to-hand)

    Returns:
        T: Matrice de transformation homogène (T_cam->flange for hand-in-eye, T_base->cam for eye-to-hand)
    """
    # Convertir les listes en tableaux NumPy
    R_a_list = [np.array(R, dtype=np.float64) for R in R_gripper2base]
    t_a_list = [np.array(t, dtype=np.float64).reshape(3, 1) for t in t_gripper2base]
    R_b_list = [np.array(R, dtype=np.float64) for R in R_target2cam]
    t_b_list = [np.array(t, dtype=np.float64).reshape(3, 1) for t in t_target2cam]


    # Tsai method (solves AX=XB)
    # For hand-in-eye: A = T_gripper2base, B = T_target2cam, X = T_cam2gripper (T_cam->flange)
    # For eye-to-hand: A = T_base2gripper, B = T_target2cam, X = T_base2cam
    # However, the OpenCV function assumes A is the *robot* motion and B is the *camera* motion relative to the target.
    # So for eye-to-hand (camera fixed, target on robot), the arguments should be:
    # A = T_base2gripper (robot motion)
    # B = T_target2cam (target motion relative to camera)
    # The function then calculates X in AX=XB, where X is T_base2cam (base to camera).

    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_a_list, t_a_list,  # Robot motion (Base->Gripper for eye-to-hand)
        R_b_list, t_b_list,  # Camera motion relative to target (Target->Cam for eye-to-hand)
        method=cv2.CALIB_HAND_EYE_TSAI)

    T = np.eye(4)
    T[:3,:3] = R_cam2gripper
    T[:3, 3] = t_cam2gripper.flatten()

    print(f"Calibrated Transformation (result of AX=XB):")
    print(T)

    with open(output_yaml, 'w') as f:
        yaml.dump({'T_calibrated_transform': T.tolist()}, f) # Use a generic key name
    print(f"Calibrated transform saved to {output_yaml}")

    return T


# Charuco detection
def load_charuco(board_squares_x, board_squares_y, square_length, marker_length, aruco_dict):
    dict_attr = getattr(cv2.aruco, aruco_dict)
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_attr)

    # Version-agnostic CharucoBoard creation
    try:
        # OpenCV 4.7.0+ style
        board = cv2.aruco.CharucoBoard(
            size=(board_squares_x, board_squares_y),
            squareLength=square_length,
            markerLength=marker_length,
            dictionary=aruco_dict
        )
    except TypeError:
        # OpenCV 4.5.x fallback
        board = cv2.aruco.CharucoBoard_create(
            squaresX=board_squares_x,
            squaresY=board_squares_y,
            squareLength=square_length,
            markerLength=marker_length,
            dictionary=aruco_dict
        )
    return board

class ArucoDetector:
    def __init__(self, config_file):
        cfg = yaml.safe_load(open(config_file))
        print("=== YAML CONFIG LOADED ===")
        print(cfg)

        self.camera_matrix = np.array(cfg['camera_matrix'], dtype=np.float32)
        self.dist_coeffs   = np.array(cfg['dist_coeffs'], dtype=np.float32)
        p = cfg['charuco_params']
        self.board      = load_charuco(
            p['board_squares_x'], p['board_squares_y'],
            p['square_length'], p['marker_length'],
            p['aruco_dict']
        )
        dict_attr = getattr(cv2.aruco, p['aruco_dict'])
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_attr)
        self.detector      = cv2.aruco.ArucoDetector(self.aruco_dict, cv2.aruco.DetectorParameters())
        self.corners = None
        self.ids = None
        self.charuco_corners = None
        self.charuco_ids = None

    def detect(self, img):
        """
        Détecte le CharucoBoard et renvoie (ok, rvec, tvec).
        Utilise estimatePoseCharucoBoard plutôt qu'estimatePoseBoard.
        rvec, tvec represent the transformation from target to camera (T_target2cam).
        """
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        self.corners, self.ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict) # Pass dictionary explicitly
        # self.corners, self.ids, _ = self.detector.detectMarkers(gray) # Use detector object

        # Vérifiez d'abord si des marqueurs ont été détectés
        if self.ids is None or len(self.ids) == 0:
            self.charuco_corners = None
            self.charuco_ids = None
            return False, None, None

        # Raffiner la détection Charuco
        retval, self.charuco_corners, self.charuco_ids = cv2.aruco.interpolateCornersCharuco(
            self.corners, self.ids, gray, self.board)

        # Vérifiez si charuco_ids est None avant d'appeler len()
        # Also check if retval is True (success)
        if retval and self.charuco_ids is not None and len(self.charuco_ids) > 3:
            # Crée les tableaux de sortie attendus par le binding Python
            rvec = np.zeros((3, 1), dtype=np.float64)
            tvec = np.zeros((3, 1), dtype=np.float64)

            # Appel avec sortie explicite
            ok = cv2.aruco.estimatePoseCharucoBoard(
                self.charuco_corners, self.charuco_ids, self.board,
                self.camera_matrix, self.dist_coeffs,
                rvec, tvec
            )

            if ok:
                # Vérifier que les valeurs renvoyées sont raisonnables
                if np.any(np.isnan(rvec)) or np.any(np.isnan(tvec)):
                    print("Warning: NaN values detected in pose estimation!")
                    return False, None, None

                # rvec et tvec sont remplis correctement (T_target2cam)
                return True, rvec, tvec

        return False, None, None

    def draw_detection(self, img):
        """
        Dessine les détections sur l'image.
        Renvoie l'image avec les visuels de détection.
        """
        display_img = img.copy()

        # 1. Dessiner les marqueurs individuels s'ils sont détectés
        if self.corners is not None and len(self.corners) > 0:
            cv2.aruco.drawDetectedMarkers(display_img, self.corners, self.ids)

        # 2. Dessiner les coins Charuco s'ils sont détectés
        if self.charuco_corners is not None and len(self.charuco_corners) > 0:
            # Dessiner les corners Charuco en vert
            for corner in self.charuco_corners:
                cv2.circle(display_img, tuple(corner.astype(int).flatten()),
                          5, (0, 255, 0), -1)

        return display_img



# UR5 control via RTDE
def load_hand_eye_transform(yaml_file):
    data = yaml.safe_load(open(yaml_file))
    # Assuming the YAML file now contains the T_base->camera transformation
    return np.array(data['T_calibrated_transform'])

class UR5Controller:
    def __init__(self, robot_ip, eye_to_hand_yaml):
        import rtde_control, rtde_receive
        self.rtde_c = rtde_control.RTDEControlInterface(robot_ip)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
        # Load the T_base->camera transform
        self.T_base2camera = load_hand_eye_transform(eye_to_hand_yaml)

    def move_to_pose(self, rvec_target_cam, tvec_target_cam, offset=(0,0,0), speed=0.1, accel=0.1):
        """
        Move the robot such that the target (Charuco) is at a desired pose relative to the camera.
        This requires the eye-to-hand transform (T_base->camera).

        Args:
            rvec_target_cam: Rotation vector of the target in the camera frame (T_target2cam).
            tvec_target_cam: Translation vector of the target in the camera frame (T_target2cam).
            offset: Optional offset in the target's frame (x, y, z).
        """
        # Build transform target->camera (T_target2cam)
        R_tc, _ = cv2.Rodrigues(rvec_target_cam)
        T_tc = np.eye(4); T_tc[:3,:3] = R_tc; T_tc[:3,3] = tvec_target_cam.flatten()

        # Build desired offset transform in target frame (T_offset)
        T_offset = np.eye(4)
        T_offset[:3, 3] = np.array(offset)

        # Calculate the desired pose of the robot's gripper (which holds the target) in the base frame (T_base2gripper).
        # We know T_base->camera and T_cam->target. We want T_base->gripper.
        # T_base->gripper = T_base->camera @ T_camera->target @ T_target->gripper
        # Assuming the target (Charuco) is rigidly attached to the gripper, T_target->gripper is a constant transform.
        # However, typically in this setup, we want to move the *gripper* to a pose such that the *target* is at a specific pose relative to the camera.
        # Let T_cam_desired_target be the desired pose of the target in the camera frame.
        # We want to find T_base_desired_gripper such that:
        # T_base_desired_gripper @ T_gripper_target = T_base_camera @ T_camera_desired_target
        # T_base_desired_gripper = T_base_camera @ T_cam_desired_target @ (T_gripper_target)^-1
        # This still requires T_gripper_target.

        # A simpler interpretation for eye-to-hand control:
        # The robot's TCP (gripper) has a pose T_base2gripper.
        # The target has a pose relative to the camera T_target2cam.
        # We know T_base2camera.
        # The relationship is: T_base2camera @ T_camera2target = T_base2gripper @ T_gripper2target
        # where T_camera2target is the inverse of T_target2cam.

        # If we want to move the gripper such that the *current* detected T_target2cam corresponds to a desired TCP pose:
        # This means the current TCP pose (T_base2gripper) corresponds to the current detected T_target2cam.
        # T_base2camera @ T_camera2target_current = T_base2gripper_current @ T_gripper2target

        # Let's assume the goal is to move the *TCP* to a specific pose in the base frame. This doesn't directly use the target detection for movement.

        # A more practical use case for eye-to-hand after calibration is to command the robot to place the *target* at a specific pose in the *camera's* frame or the *base's* frame.
        # To move the *target* (mounted on the gripper) to a desired pose T_cam_desired_target in the camera frame:
        # We need to find the required T_base_desired_gripper.
        # T_base_desired_gripper = T_base_camera @ T_camera_desired_target @ (T_gripper_target)^-1
        # Still need T_gripper_target.

        # Let's reconsider the provided arguments `rvec` and `tvec` from `det.detect(img)`, which are T_target2cam.
        # If the goal is to move the robot relative to the *currently detected* target pose:
        # Move the TCP from its current pose to a new pose based on the detected target.
        # For example, move the TCP such that the target's X, Y, Z in the camera frame changes by a certain amount.
        # This implies moving the gripper by that amount transformed into the base frame.

        # Let's assume the goal is to move the *gripper* to a specific pose in the *base* frame, but the pose is *derived* from the target's detected pose in the camera frame.
        # This is similar to hand-in-eye pick and place, but the transform T_base->camera is used differently.

        # Let's assume the intention is to move the *TCP* to a calculated pose based on the detected target pose and the eye-to-hand transform.
        # A common use case is to calculate the pose of the target in the base frame and then calculate a desired TCP pose relative to that.
        # T_target2base = T_camera2base @ T_target2cam = (T_base2camera)^-1 @ T_target2cam
        T_camera2base = np.linalg.inv(self.T_base2camera)

        R_tc, _ = cv2.Rodrigues(rvec_target_cam)
        T_tc = np.eye(4); T_tc[:3,:3] = R_tc; T_tc[:3,3] = tvec_target_cam.flatten()

        T_target2base = T_camera2base @ T_tc

        # Now, we have the pose of the target in the base frame.
        # If the target is mounted on the gripper with a fixed T_gripper2target transform,
        # then T_base2gripper = T_target2base @ (T_gripper2target)^-1
        # However, we don't have T_gripper2target from this calibration process directly.

        # Let's assume the provided `offset` is a desired pose of the *target* relative to the *camera*, and we want to move the robot's TCP (gripper) to achieve this.
        # This doesn't make sense for eye-to-hand with a fixed camera.

        # Let's interpret `rvec_target_cam` and `tvec_target_cam` as the *current* detected pose of the target relative to the camera (T_target2cam).
        # And the `offset` is a desired displacement *of the gripper* in the *base* frame relative to the current gripper pose. This also doesn't use the eye-to-hand calibration effectively.

        # A more likely use case for eye-to-hand: given a desired pose of the *target* in the *base* frame (T_base_desired_target), calculate the required *gripper* pose in the base frame (T_base_desired_gripper).
        # T_base_desired_gripper = T_base_desired_target @ (T_gripper_target)^-1
        # Still need T_gripper_target.

        # Let's assume a different use case: move the robot such that the *target* is at a desired pose relative to the *camera*.
        # Let T_cam_desired_target be the desired pose of the target in the camera frame.
        # We need to find T_base_desired_gripper such that:
        # T_base_desired_gripper @ T_gripper_target = T_base_camera @ T_cam_desired_target
        # T_base_desired_gripper = T_base_camera @ T_cam_desired_target @ (T_gripper_target)^-1
        # Still need T_gripper_target.

        # Perhaps the `move_to_pose` function should take a desired pose of the *target* in the *base* frame.
        # Let's redefine `move_to_pose` to take `rvec_desired_target_base`, `tvec_desired_target_base`.
        # Then T_base_desired_target is built from these.
        # T_base_desired_gripper = T_base_desired_target @ (T_gripper_target)^-1
        # Still need T_gripper_target.

        # Let's go back to the original structure and assume `rvec_target_cam`, `tvec_target_cam` are *detected* current poses.
        # And `offset` is a desired displacement *of the target* in the *camera* frame.
        # Desired T_cam_desired_target = T_cam_current_target @ T_offset_in_camera_frame
        # T_cam_current_target is inverse of T_target2cam.
        # T_cam_current_target = np.linalg.inv(T_tc)
        # T_cam_desired_target = T_cam_current_target @ T_offset (where offset is in camera frame)

        # Required T_base_desired_gripper = T_base_camera @ T_cam_desired_target @ (T_gripper_target)^-1
        # Again, T_gripper_target is needed.

        # It seems the original `move_to_pose` logic is for hand-in-eye, where you move the gripper relative to the detected target in the camera frame.
        # For eye-to-hand, the control strategy changes. You usually want to move the target (on the gripper) to a specific pose in the base or camera frame.

        # Let's assume the simplest adaptation: use the calculated T_base->camera to transform the detected target pose (T_target2cam) into the base frame (T_target2base).
        # And then perhaps define a target gripper pose relative to this target pose in the base frame.
        # T_target2base = T_camera2base @ T_target2cam

        # Let's modify `move_to_pose` to take `rvec_target_cam`, `tvec_target_cam` (detected current pose) and an optional `offset_in_base` (displacement for the gripper in base frame). This still doesn't fully utilize eye-to-hand calibration for precise target placement.

        # A better approach for eye-to-hand: Define a desired pose for the *target* in the *camera* frame (T_cam_desired_target).
        # Then calculate the required *gripper* pose in the *base* frame (T_base_desired_gripper).
        # T_base_desired_gripper = T_base_camera @ T_cam_desired_target @ T_target_gripper
        # We need T_target_gripper, which is the fixed transform from the target (Charuco) to the gripper. This transform is *not* calculated by standard hand-eye calibration. It needs to be determined separately (e.g., by measuring).

        # Given the original function structure, let's try to adapt it assuming the `offset` is a desired displacement of the *target* in the *camera* frame, and we want to move the *gripper* to achieve this.
        # Current T_cam_current_target = np.linalg.inv(T_tc)
        # Desired T_cam_desired_target = T_cam_current_target @ T_offset_in_camera_frame (offset is applied in camera frame)
        # Required T_base_desired_gripper = T_base_camera @ T_cam_desired_target @ (T_gripper_target)^-1
        # This still requires T_gripper_target.

        # Let's assume the `offset` is a desired pose *of the target* relative to the *base frame* (T_base_desired_offset).
        # Then the desired target pose in the base frame is T_base_desired_target = T_target2base_current + offset_translation_in_base
        # Or maybe the user wants to specify a desired *gripper* pose relative to the detected *target* in the base frame.

        # Let's assume the user wants to move the gripper to a pose relative to the detected target in the base frame.
        # T_target2base = (T_base2camera)^-1 @ T_target2cam
        # Let T_target_gripper_offset be the desired offset of the gripper relative to the target in the base frame.
        # T_base_desired_gripper = T_target2base @ T_target_gripper_offset
        # This still requires knowing T_target_gripper_offset.

        # Given the constraints, let's make a simplifying assumption for the `move_to_pose` function in the eye-to-hand context:
        # Assume `rvec_target_cam` and `tvec_target_cam` are the *detected* pose of the target in the camera frame.
        # Assume `offset` is a desired *translation* offset in the *base* frame, which will be applied to the calculated target pose in the base frame.
        # T_target2base = (T_base2camera)^-1 @ T_target2cam
        # Desired T_base_desired_target_pos = T_target2base[:3, 3] + np.array(offset)
        # To maintain orientation, we'll keep the rotation of T_target2base.
        # Desired T_base_desired_target = np.eye(4); T_base_desired_target[:3,:3] = T_target2base[:3,:3]; T_base_desired_target[:3,3] = Desired T_base_desired_target_pos

        # Now, how to get the gripper pose? We need T_gripper2target (or its inverse).
        # Without T_gripper2target, we cannot precisely calculate the required gripper pose for a desired target pose.

        # Let's modify `move_to_pose` to take `rvec_target_cam`, `tvec_target_cam` (detected pose) and calculate the target pose in the base frame.
        # Then, the robot can move its TCP to this calculated pose in the base frame as a simple example, ignoring the gripper-target offset. This is not truly placing the target, but moving the gripper to the target's location in the base frame.

        print("Warning: move_to_pose logic for eye-to-hand needs a fixed T_gripper->target transform which is not calibrated here.")
        print("The following move_to_pose implementation will move the TCP to the detected target's position in the base frame, maintaining current TCP orientation.")

        T_camera2base = np.linalg.inv(self.T_base2camera)

        R_tc, _ = cv2.Rodrigues(rvec_target_cam)
        T_tc = np.eye(4); T_tc[:3,:3] = R_tc; T_tc[:3,3] = tvec_target_cam.flatten()

        # Pose of the target in the base frame
        T_target2base = T_camera2base @ T_tc

        # Desired TCP pose in the base frame (for simplicity, match target position, keep current TCP orientation)
        desired_pos_base = T_target2base[:3, 3] + np.array(offset) # Apply offset in base frame

        # Get current TCP orientation
        tcp = self.rtde_r.getActualTCPPose()
        current_ori_tcp = tcp[3:6] # Keep current Rx, Ry, Rz

        # Desired TCP pose (position from target in base frame + offset, orientation from current TCP)
        pose = [*desired_pos_base.tolist(), *current_ori_tcp]

        print(f"Moving TCP to target position in base frame: {pose}")
        self.rtde_c.moveL(pose, speed, accel)


# Example usage
if __name__ == '__main__':
     # ------------------------------------------------------------------
    # 1) Définition du CharucoBoard
    #    (doit correspondre EXACTEMENT à ta planche imprimée)
    # Ensure these match the board mounted on the robot's hand.
    aruco_dict_name = cv2.aruco.DICT_4X4_50
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_name)
    board_squares_x = 3
    board_squares_y = 5
    square_length = 0.04
    marker_length = 0.02

    board = cv2.aruco.CharucoBoard(
        size=(board_squares_x, board_squares_y),
        squareLength=square_length,
        markerLength=marker_length,
        dictionary=aruco_dict
    )

    # ------------------------------------------------------------------
    # 2) Calibration de l'intrinsèque de la caméra
    #    Assurez-vous que 'intrinsics.yaml' est déjà généré par une calibration.
    #    Décommente si tu as besoin de re-calibrer l'intrinsèque avec la caméra fixe.
    # camera_matrix, dist_coeffs = calibrate_camera_intrinsics(
    #     image_files='/path/to/your/intrinsic/calibration/images/*.jpg', # Update path
    #     charuco_board=board, # Use the same board definition if used for intrinsic calib
    #     aruco_dict=aruco_dict, # Use the same dict
    #     square_length=square_length,
    #     marker_length=marker_length,
    #     output_yaml='intrinsics.yaml') # Save to intrinsics.yaml

    # Load existing intrinsics
    try:
        with open('intrinsics.yaml', 'r') as f:
             intrinsic_data = yaml.safe_load(f)
             camera_matrix = np.array(intrinsic_data['camera_matrix'])
             dist_coeffs = np.array(intrinsic_data['dist_coeffs'])
             print("Loaded intrinsics from intrinsics.yaml")
    except FileNotFoundError:
        print("Error: intrinsics.yaml not found. Please run camera intrinsic calibration first.")
        exit()
    except KeyError:
        print("Error: Invalid intrinsics.yaml format. Missing 'camera_matrix' or 'dist_coeffs'.")
        exit()


    # ------------------------------------------------------------------
    # 3) Calibration eye–to–hand (base→caméra)
    #    Requires running `calibration_hand_eye_data.py` first to generate `hand_eye_data_eye_to_hand.yaml`
    #    Then run `do_hand_eye_calib.py` to process it and generate `hand_eye_base_to_camera.yaml`

    # Example of how to run eye-to-hand calibration after data collection (for reference)
    # You would typically run do_hand_eye_calib.py separately after data collection.
    # This section is commented out as it's done by do_hand_eye_calib.py
    # try:
    #     with open('hand_eye_data_eye_to_hand.yaml', 'r') as f:
    #         data = yaml.safe_load(f)
    #
    #     R_base2gripper_list = data['R_base2gripper_list']
    #     t_base2gripper_list = data['t_base2gripper_list']
    #     R_cam2target_list   = data['R_cam2target_list']
    #     t_cam2target_list   = data['t_cam2target_list']
    #
    #     # Invert cam->target to get target->cam for calibrateHandEye input B
    #     R_target2cam_list_inv = []
    #     t_target2cam_list_inv = []
    #     for R_ct, t_ct in zip(R_cam2target_list, t_cam2target_list):
    #         T_ct = np.eye(4)
    #         T_ct[:3, :3] = np.array(R_ct)
    #         T_ct[:3, 3] = np.array(t_ct).flatten()
    #         T_tc = np.linalg.inv(T_ct)
    #         R_target2cam_list_inv.append(T_tc[:3, :3].tolist())
    #         t_target2cam_list_inv.append(T_tc[:3, 3].tolist())
    #
    #     T_base2camera = calibrate_hand_eye(
    #         R_base2gripper_list,  # A (base->gripper)
    #         t_base2gripper_list,  # A (base->gripper)
    #         R_target2cam_list_inv, # B (target->camera)
    #         t_target2cam_list_inv, # B (target->camera)
    #         'hand_eye_base_to_camera.yaml' # Output for T_base->camera
    #     )
    # except FileNotFoundError:
    #      print("Warning: hand_eye_data_eye_to_hand.yaml not found. Skipping eye-to-hand calibration example run.")
    # except KeyError as e:
    #      print(f"Warning: Missing key in hand_eye_data_eye_to_hand.yaml: {e}. Skipping eye-to-hand calibration example run.")
    # except Exception as e:
    #      print(f"An error occurred during eye-to-hand calibration example run: {e}. Skipping.")


    # ------------------------------------------------------------------
    # 4) Initialisation des flux RealSense et des modules de détection & contrôle
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    # Use depth stream if needed for 3D point clouds later
    # cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    try:
        pipeline.start(cfg)
    except Exception as e:
        print(f"Error starting RealSense pipeline: {e}")
        print("Make sure the camera is connected and not in use by another process.")
        exit()


    # Charger les YAML générés (intrinsics + eye-to-hand T_base->camera)
    # The ArucoDetector needs intrinsics.
    # The UR5Controller needs the T_base->camera transform.
    try:
        det_params = {
            'camera_matrix': camera_matrix.tolist(),
            'dist_coeffs': dist_coeffs.tolist(),
            'charuco_params': {
                'board_squares_x': board_squares_x,
                'board_squares_y': board_squares_y,
                'square_length': square_length,
                'marker_length': marker_length,
                'aruco_dict': 'DICT_4X4_50' # Use the string name
            }
        }
        # Temporarily save detector params to a file for ArucoDetector to load
        with open('detector_params.yaml', 'w') as f:
            yaml.dump(det_params, f)

        det  = ArucoDetector('detector_params.yaml') # Initialize with intrinsics and board info
    except Exception as e:
        print(f"Error initializing ArucoDetector: {e}")
        exit()


    # Initialize UR5 Controller with the eye-to-hand transform (T_base->camera)
    try:
        # Ensure 'hand_eye_base_to_camera.yaml' exists after running do_hand_eye_calib.py
        ctrl = UR5Controller('10.2.30.60', 'hand_eye_base_to_camera.yaml')
        print("UR5Controller initialized with eye-to-hand transform.")
    except FileNotFoundError:
        print("Error: hand_eye_base_to_camera.yaml not found.")
        print("Please run calibration_hand_eye_data.py and then do_hand_eye_calib.py first.")
        ctrl = None # Set to None if calibration file is missing
    except ImportError:
         print("Error: RTDE libraries not found. Cannot initialize UR5Controller.")
         ctrl = None
    except Exception as e:
         print(f"An error occurred initializing UR5Controller: {e}")
         ctrl = None



    print("Appuie sur 'Esc' pour quitter.")
    print("Appuie sur 'm' pour tenter de déplacer le robot (requires successful detection and UR5Controller).")

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            img = np.asanyarray(color_frame.get_data())
            # det.detect returns T_target2cam (rvec, tvec)
            ok, rvec, tvec = det.detect(img)

            # Affiche les détections et les résultats visuels
            display_img = det.draw_detection(img)

            # If Charuco is detected completely, draw axes and show pose
            if ok:
                # Verify rvec and tvec are valid
                if rvec is None or tvec is None:
                    print("Warning: Detection OK but rvec or tvec is None!")
                    cv2.putText(display_img, "DETECTION ERROR", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    ok = False # Treat as not OK for further processing if pose is invalid
                else:
                    # Draw axes for target in camera frame
                    cv2.drawFrameAxes(
                        display_img,
                        det.camera_matrix,
                        det.dist_coeffs,
                        rvec, tvec,
                        0.05  # axis length in meters
                    )

                    # Display pose of target in camera frame (T_target2cam)
                    position_text_cam = f"Target in Cam: X:{tvec[0][0]:.3f} Y:{tvec[1][0]:.3f} Z:{tvec[2][0]:.3f}"
                    cv2.putText(display_img, position_text_cam, (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                    cv2.putText(display_img, "TARGET DETECTED", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                    # If UR5Controller is initialized, calculate and display target pose in base frame
                    if ctrl is not None:
                         try:
                            # Calculate T_target2cam homogeneous transform
                            R_tc, _ = cv2.Rodrigues(rvec)
                            T_tc = np.eye(4); T_tc[:3,:3] = R_tc; T_tc[:3,3] = tvec.flatten()

                            # Transform T_target2cam to T_base2target using T_base2camera
                            # Correct chain: T_base2target = T_base2camera @ T_camera2target
                            # T_camera2target = inverse of T_target2cam
                            T_camera2target = np.linalg.inv(T_tc)
                            T_base2target = ctrl.T_base2camera @ T_camera2target

                            pos_target_base = T_base2target[:3, 3]
                            # Get orientation in base frame as rotation vector
                            R_base2target = T_base2target[:3, :3]
                            rvec_base2target, _ = cv2.Rodrigues(R_base2target)

                            position_text_base = f"Target in Base (Trans): X:{pos_target_base[0]:.3f} Y:{pos_target_base[1]:.3f} Z:{pos_target_base[2]:.3f}"
                            orientation_text_base = f"Target in Base (Rot): Rx:{rvec_base2target[0][0]:.3f} Ry:{rvec_base2target[1][0]:.3f} Rz:{rvec_base2target[2][0]:.3f}"

                            cv2.putText(display_img, position_text_base, (10, 90),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                            cv2.putText(display_img, orientation_text_base, (10, 120),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

                         except Exception as e:
                              cv2.putText(display_img, f"Base frame calc error: {e}", (10, 90),
                                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)


            else:
                # Indicate incomplete detection
                detected_markers = len(det.corners) if det.corners is not None else 0
                cv2.putText(display_img, f"Markers: {detected_markers}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            cv2.imshow('Aruco Detection (Eye-to-Hand)', display_img)
            key = cv2.waitKey(1) & 0xFF

            # Move robot command
            if key == ord('m'):
                if ok and ctrl is not None:
                     print("Attempting to move robot to target position in base frame...")
                     # Example move: move TCP to the detected target's position in the base frame with a small Z offset
                     # This is a very basic example. Real pick and place needs more sophisticated logic.
                     try:
                        # Call the modified move_to_pose
                        # Using the detected rvec/tvec (T_target2cam) and a base frame offset (e.g., 0,0,0 for position match)
                        ctrl.move_to_pose(rvec, tvec, offset=(0,0,0.1)) # Move TCP to target's base position + 10cm in Z
                        print("Move command sent.")
                     except Exception as e:
                          print(f"Error sending move command: {e}")
                elif not ok:
                    print("Cannot move: Target not detected.")
                elif ctrl is None:
                    print("Cannot move: UR5Controller not initialized (check calibration file and robot connection).")


            if key == 27: # ESC key
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
