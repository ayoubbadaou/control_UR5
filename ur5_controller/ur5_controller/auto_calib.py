#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
import numpy as np
import time
import rtde_io
import rtde_receive
import rtde_control
from moveit_msgs.msg import RobotState, Constraints, JointConstraint, PositionConstraint, OrientationConstraint
from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.msg import PlanningOptions
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_msgs.msg import CollisionObject, PlanningScene, AttachedCollisionObject # Import PlanningScene and AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import threading # Import threading for waiting on futures
from scipy.spatial.transform import Rotation as R # Import scipy for axis-angle to quaternion conversion
import signal # Import signal for handling interrupts
import sys # Import sys for exiting cleanly

import cv2
import yaml
import pyrealsense2 as rs
import os # Import the os module
from moveit_msgs.msg import RobotTrajectory # Import RobotTrajectory

# Vérifiez si pick_and_place_system est disponible
try:
    from ur5_controller.pick_and_place_system import ArucoDetector
    aruco_detector_available = True
except ImportError:
    aruco_detector_available = False
    print("ERREUR : Le module 'pick_and_place_system' n'a pas été trouvé.")
    print("Veuillez vous assurer que le package est installé et sourcé dans votre environnement ROS2.")


class URMoveitRTDENode(Node):
    def __init__(self):
        super().__init__('ur_moveit_rtde_node_calibration') # Donnez-lui un nom différent
        self.get_logger().info("Démarrage du URMoveitRTDENode pour la calibration")

        # Initialiser l'interface RTDE
        self.robot_ip = "10.2.30.60" # Remplacez par l'adresse IP de votre robot
        self.rtde_control = None # Le contrôle RTDE n'est pas nécessaire pour l'exécution de MoveIt
        self.rtde_receive = None

        try:
            self.rtde_receive = rtde_receive.RTDEReceiveInterface(self.robot_ip)
            self.get_logger().info(f"Connecté au robot UR à {self.robot_ip} pour la réception RTDE.")

        except Exception as e:
            self.get_logger().error(f"Échec de la connexion au robot UR à {self.robot_ip} pour la réception RTDE : {e}")



        # Initialiser les clients d'action pour MoveIt 2
        self._move_group_action_client = ActionClient(self, MoveGroup, '/move_action')
        self._execute_trajectory_action_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

        self.get_logger().info("Attente du serveur d'action MoveGroup...")
        self._move_group_action_client.wait_for_server()
        self.get_logger().info("Serveur d'action MoveGroup connecté.")

        self.get_logger().info("Attente du serveur d'action ExecuteTrajectory...")
        self._execute_trajectory_action_client.wait_for_server()
        self.get_logger().info("Serveur d'action ExecuteTrajectory connecté.")


        # Optionnel : Client du service IK (si vous prévoyez d'utiliser des objectifs de pose)
        # self._get_ik_client = self.create_client(GetPositionIK, '/compute_ik')
        # while not self._get_ik_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service IK non disponible, attente...')
        # self.get_logger().info("Service IK connecté.")

        # Publicateur pour l'interface Planning Scene
        # Nous utilisons un publicateur de type latching pour garantir que la scène est reçue
        self.planning_scene_publisher = self.create_publisher(PlanningScene, '/planning_scene', 10) # Utilisation du message PlanningScene
        self.get_logger().info("Publicateur Planning Scene créé.")

        # Futures pour les clients d'action et les gestionnaires d'objectifs
        self._send_plan_goal_future = None
        self._get_plan_result_future = None
        self._send_execute_goal_future = None
        self._get_execute_result_future = None
        self._execute_goal_handle = None # Stocker le gestionnaire d'objectif pour l'annulation


    def add_collision_object(self, object_id, shape_type, dimensions, pose, frame_id="world"):
        """
        Crée et renvoie un message CollisionObject.
        """
        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.id = object_id

        primitive = SolidPrimitive()
        primitive.type = shape_type
        primitive.dimensions = dimensions
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD

        self.get_logger().info(f"Définition de CollisionObject créée pour : {object_id} dans le frame {frame_id}")
        return collision_object


    def remove_collision_object(self, object_id):
        """
         Supprime un objet de collision de la scène de planification.

        :param object_id: L'ID (string) de l'objet à supprimer.
        """
        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"  # Le Frame ID n'a pas d'importance stricte pour la suppression par ID, mais correspond souvent au frame d'ajout
        collision_object.id = object_id
        collision_object.operation = CollisionObject.REMOVE

        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.world.collision_objects.append(collision_object)

        self.planning_scene_publisher.publish(planning_scene)
        self.get_logger().info(f"Objet de collision supprimé : {object_id}")
        # Ajouter un petit délai pour permettre au Planning Scene Monitor de MoveIt de traiter la mise à jour
        time.sleep(0.1) # Ajuster si nécessaire


    def setup_planning_scene(self, fixed_obstacles=None, attached_objects=None, fixed_frame_id="world", attached_link=""):
        """
        Configure la scène de planification en ajoutant des obstacles fixes prédéfinis et des objets attachés.
        """
        self.get_logger().info("Configuration de la scène de planification...")
        planning_scene = PlanningScene()
        planning_scene.is_diff = True

        if fixed_obstacles:
            self.get_logger().info(f"Ajout d'obstacles fixes au monde dans le frame {fixed_frame_id}...")
            for obstacle in fixed_obstacles:
                if obstacle["type"] == "box":
                    obstacle_pose = Pose()
                    obstacle_pose.position.x = float(obstacle["pose"][0])
                    obstacle_pose.position.y = float(obstacle["pose"][1])
                    obstacle_pose.position.z = float(obstacle["pose"][2])
                    obstacle_pose.orientation.x = float(obstacle["pose"][3])
                    obstacle_pose.orientation.y = float(obstacle["pose"][4])
                    obstacle_pose.orientation.z = float(obstacle["pose"][5])
                    obstacle_pose.orientation.w = float(obstacle["pose"][6])
                    obstacle_dimensions = obstacle["size"]
                    col_obj = self.add_collision_object(obstacle["name"], SolidPrimitive.BOX, obstacle_dimensions, obstacle_pose, fixed_frame_id)
                    planning_scene.world.collision_objects.append(col_obj)

        if attached_objects and attached_link:
             self.get_logger().info(f"Ajout d'objets attachés à {attached_link}...")
             for obj in attached_objects:
                 if obj["type"] == "box":
                     obj_pose = Pose()
                     obj_pose.position.x = float(obj["pose"][0])
                     obj_pose.position.y = float(obj["pose"][1])
                     obj_pose.position.z = float(obj["pose"][2])
                     obj_pose.orientation.x = float(obj["pose"][3])
                     obj_pose.orientation.y = float(obj["pose"][4])
                     obj_pose.orientation.z = float(obj["pose"][5])
                     obj_pose.orientation.w = float(obj["pose"][6])
                     obj_dimensions = obj["size"]
                     # Utiliser "tool0" comme frame_id pour les primitive_poses de l'objet attaché
                     col_obj = self.add_collision_object(obj["name"], SolidPrimitive.BOX, obj_dimensions, obj_pose, attached_link) # frame_id pour CollisionObject doit correspondre au lien attaché

                     attached_col_obj = AttachedCollisionObject()
                     attached_col_obj.link_name = attached_link
                     attached_col_obj.object = col_obj
                     # S'assurer que la pose de l'objet attaché est relative au lien auquel il est attaché
                     # La fonction add_collision_object définit déjà la pose dans le frame spécifié (attached_link)
                     planning_scene.robot_state.attached_collision_objects.append(attached_col_obj)


        self.planning_scene_publisher.publish(planning_scene)
        # Ajouter un petit délai pour permettre au Planning Scene Monitor de MoveIt de traiter la mise à jour
        time.sleep(1.0) # Augmenter si nécessaire
        self.get_logger().info("Configuration de la scène de planification terminée.")


    def plan_and_execute_pose_goal(self, goal_pose: Pose, goal_frame_id="base"):
        """
        Planifie et exécute une trajectoire vers l'objectif de pose spécifié (orientation en quaternion).

        :param goal_pose: La pose cible (geometry_msgs.msg.Pose).
        :param goal_frame_id: Le frame de coordonnées dans lequel l'objectif de pose est défini (par défaut : "base_link").
        """
        self.get_logger().info(f"Planification et exécution de la trajectoire vers l'objectif de pose dans le frame {goal_frame_id}...")

        goal_msg = MoveGroup.Goal()
        request = MotionPlanRequest()

        # Définir le groupe de planification
        request.group_name = "ur_manipulator" # En supposant que "ur_manipulator" est le nom de votre groupe de planification

        # Définir l'état de départ à l'état actuel du robot lu depuis RTDE
        if self.rtde_receive and self.rtde_receive.isConnected():
            # Ajouter un petit délai pour s'assurer que l'interface de réception RTDE est prête
            time.sleep(0.1) # Ajuster si nécessaire
            current_joint_state_list = self.rtde_receive.getActualQ()
            self.get_logger().info(f"RTDE getActualQ() a retourné : {current_joint_state_list}") # Log de débogage
            self.get_logger().info(f"Longueur du résultat de getActualQ() : {len(current_joint_state_list) if current_joint_state_list is not None else 'None'}") # Log de débogage


            if current_joint_state_list is not None and len(current_joint_state_list) == 6: # 6 articulations pour UR5
                robot_state = RobotState()
                # L'ordre est :
                # ['shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'shoulder_pan_joint']
                # Utiliser cet ordre pour définir l'état de départ, en supposant que RTDE getActualQ() correspond ou que MoveIt peut gérer cela.
                # Une manière plus robuste serait d'utiliser un dictionnaire de mappage si l'ordre est réellement incohérent.
                # Pour l'instant, nous nous en tenons à l'ordre observé /joint_states pour les positions RTDE.
                rtde_joint_names_order = [
                    'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
                    'wrist_2_joint', 'wrist_3_joint', 'shoulder_pan_joint'
                ]

                robot_state.joint_state.name = rtde_joint_names_order
                robot_state.joint_state.position = list(current_joint_state_list) # Attribuer les positions basées sur cet ordre supposé
                request.start_state = robot_state
                self.get_logger().info(f"Définition de l'état de départ de la planification à l'état articulaire actuel du robot : {current_joint_state_list} avec l'ordre supposé {rtde_joint_names_order}")
            else:
                 self.get_logger().error("Impossible d'obtenir un état articulaire valide actuel depuis RTDE. La planification pourrait échouer ou utiliser l'état de départ par défaut.")
                 if current_joint_state_list is not None:
                     self.get_logger().error(f"État articulaire reçu (liste) : {current_joint_state_list}")
                 else:
                     self.get_logger().error("RTDE getActualQ() a retourné None.")


        else:
             self.get_logger().error("L'interface de réception RTDE n'est pas initialisée ou connectée. Impossible d'obtenir l'état actuel du robot pour la planification.")
             # Si RTDE n'est pas connecté, MoveIt utilisera probablement un état de départ par défaut.
             # Selon votre configuration, cela pourrait être acceptable pour la planification,
             # mais cela ne reflétera pas la position réelle du robot.


        # Définir les contraintes d'objectif
        goal_constraints = Constraints()

        self.get_logger().info(f"Définition des contraintes d'objectif de pose dans le frame {goal_frame_id}.")
        # En supposant que la pose est pour le lien de l'effecteur final (tool0 ou similaire)
        end_effector_link = "tool0" # Remplacez par le nom du lien de l'effecteur final de votre robot

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = goal_frame_id # Définir frame_id basé sur l'entrée
        position_constraint.link_name = end_effector_link
        position_constraint.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01])) # Une petite sphère autour du point cible
        position_constraint.constraint_region.primitive_poses.append(goal_pose)
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = goal_frame_id # Définir frame_id basé sur l'entrée
        orientation_constraint.link_name = end_effector_link
        orientation_constraint.orientation = goal_pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.01 # Ajuster la tolérance si nécessaire
        orientation_constraint.absolute_y_axis_tolerance = 0.01 # Ajuster la tolérance si nécessaire
        orientation_constraint.absolute_z_axis_tolerance = 0.01 # Ajuster la tolérance si nécessaire
        orientation_constraint.weight = 1.0

        goal_constraints.position_constraints.append(position_constraint)
        goal_constraints.orientation_constraints.append(orientation_constraint)


        request.goal_constraints.append(goal_constraints)

        # Définir les options de planification
        request.num_planning_attempts = 10
        request.planner_id = "RRTstar" # Planificateur d'exemple, ajuster en fonction de la configuration de MoveIt 2

        goal_msg.request = request

        self.get_logger().info("Envoi de l'objectif de planification au serveur d'action MoveGroup...")
        self._send_plan_goal_future = self._move_group_action_client.send_goal_async(goal_msg)
        # Attendre que l'objectif de planification soit accepté ou rejeté
        rclpy.spin_until_future_complete(self, self._send_plan_goal_future)
        goal_handle = self._send_plan_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Objectif de planification rejeté.")
            return False # Retourner False si la planification est rejetée

        self.get_logger().info("Objectif de planification accepté. Attente du résultat...")
        self._get_plan_result_future = goal_handle.get_result_async()
        # Attendre le résultat de la planification
        rclpy.spin_until_future_complete(self, self._get_plan_result_future)
        plan_result = self._get_plan_result_future.result().result

        if plan_result.error_code.val == plan_result.error_code.SUCCESS:
            self.get_logger().info("Planification réussie.")
            planned_trajectory = plan_result.planned_trajectory.joint_trajectory
            if planned_trajectory.points:
                 self.get_logger().info(f"La trajectoire planifiée a {len(planned_trajectory.points)} points.")
                 # Envoyer la trajectoire planifiée au serveur d'action ExecuteTrajectory
                 return self.execute_trajectory(planned_trajectory) # Retourner le résultat de l'exécution
            else:
                 self.get_logger().warn("La trajectoire planifiée n'a pas de points.")
                 return False # Retourner False si la trajectoire planifiée est vide

        else:
            self.get_logger().error(f"La planification a échoué avec le code d'erreur : {plan_result.error_code.val}")
            return False # Retourner False si la planification échoue


    def plan_and_execute_tcp_pose_goal(self, tcp_pose: list, goal_frame_id="base"):
        """
        Planifie et exécute une trajectoire vers l'objectif de pose TCP spécifié (position et orientation en axe-angle).

        :param tcp_pose: Une liste [x, y, z, rx, ry, rz] représentant la pose TCP cible.
                         Position en mètres, orientation en axe-angle (radians).
        :param goal_frame_id: Le frame de coordonnées dans lequel l'objectif de pose est défini (par défaut : "base_link").
        """
        if len(tcp_pose) != 6:
            self.get_logger().error("tcp_pose doit être une liste de 6 valeurs : [x, y, z, rx, ry, rz]")
            return False # Retourner False en cas d'erreur

        x, y, z, rx, ry, rz = tcp_pose

        # Convertir l'axe-angle [rx, ry, rz] en quaternion [x, y, z, w] en utilisant scipy
        try:
            # Créer un objet Rotation à partir du vecteur axe-angle
            rotation = R.from_rotvec([rx, ry, rz])
            # Obtenir le quaternion au format [x, y, z, w]
            quaternion = rotation.as_quat()
        except Exception as e:
            self.get_logger().error(f"Échec de la conversion de l'axe-angle en quaternion en utilisant scipy : {e}")
            return False # Retourner False en cas d'erreur

        # Créer le message geometry_msgs/Pose
        goal_pose_quat = Pose()
        goal_pose_quat.position.x = float(x)
        goal_pose_quat.position.y = float(y)
        goal_pose_quat.position.z = float(z)
        goal_pose_quat.orientation.x = float(quaternion[0])
        goal_pose_quat.orientation.y = float(quaternion[1])
        goal_pose_quat.orientation.z = float(quaternion[2])
        goal_pose_quat.orientation.w = float(quaternion[3])

        self.get_logger().info(f"Pose TCP convertie [x,y,z,rx,ry,rz] : [{x}, {y}, {z}, {rx}, {ry}, {rz}]")
        self.get_logger().info(f"En Quaternion [x,y,z,w] : [{quaternion[0]}, {quaternion[1]}, {quaternion[2]}, {quaternion[3]}]")


        # Appeler la fonction existante plan_and_execute_pose_goal avec la pose en quaternion
        return self.plan_and_execute_pose_goal(goal_pose_quat, goal_frame_id)


    def execute_trajectory(self, trajectory: JointTrajectory):
        """
        Envoie la trajectoire planifiée au serveur d'action ExecuteTrajectory pour exécution.
        Renvoie True si l'exécution est réussie, False sinon.
        """
        if not self._execute_trajectory_action_client.server_is_ready():
            self.get_logger().error("Le serveur d'action ExecuteTrajectory n'est pas prêt.")
            return False

        self.get_logger().info(f"Envoi de la trajectoire avec {len(trajectory.points)} points pour exécution.")

        execute_goal_msg = ExecuteTrajectory.Goal()
        # Envelopper la JointTrajectory dans un message RobotTrajectory
        robot_trajectory = RobotTrajectory()
        robot_trajectory.joint_trajectory = trajectory
        execute_goal_msg.trajectory = robot_trajectory

        self._send_execute_goal_future = self._execute_trajectory_action_client.send_goal_async(execute_goal_msg)
        # Attendre que l'objectif d'exécution soit accepté ou rejeté
        rclpy.spin_until_future_complete(self, self._send_execute_goal_future)
        goal_handle = self._send_execute_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Objectif d'exécution rejeté.")
            self._execute_goal_handle = None # Effacer le gestionnaire d'objectif s'il est rejeté
            return False # Retourner False si l'exécution est rejetée

        self.get_logger().info("Objectif d'exécution accepté. Attente du résultat...")
        self._execute_goal_handle = goal_handle # Stocker le gestionnaire d'objectif
        self._get_execute_result_future = goal_handle.get_result_async()

        # Attendre la fin de l'exécution
        timeout_sec = 60.0 # Définir un délai d'attente pour l'exécution
        start_time = time.time()
        # Attendre que le future soit terminé ou qu'un délai d'attente soit atteint
        # Maintenant, self._get_execute_result_future est garanti de ne pas être None ici
        while rclpy.ok() and not self._get_execute_result_future.done() and (time.time() - start_time < timeout_sec):
             rclpy.spin_once(self, timeout_sec=0.1)

        self._execute_goal_handle = None # Effacer le gestionnaire d'objectif après la tentative d'exécution

        if self._get_execute_result_future and self._get_execute_result_future.done():
            result = self._get_execute_result_future.result().result
            if result.error_code.val == result.error_code.SUCCESS:
                self.get_logger().info("Exécution de la trajectoire réussie.")
                return True # Retourner True en cas de succès
            else:
                self.get_logger().error(f"L'exécution de la trajectoire a échoué avec le code d'erreur : {result.error_code.val}")
                return False # Retourner False en cas d'échec
        elif time.time() - start_time >= timeout_sec:
            self.get_logger().error("Exécution expirée.")
            self.cancel_execution_goal() # Tenter d'annuler l'objectif expiré
            return False # Retourner False en cas d'expiration
        else:
            # Ce cas ne devrait idéalement pas être atteint
            self.get_logger().error("Résultat de l'exécution non disponible après attente ou expiration.")
            return False


    def cancel_execution_goal(self):
        """
        Envoie une requête d'annulation à l'objectif d'action ExecuteTrajectory actif.
        """
        if self._execute_goal_handle is not None and self._execute_goal_handle.status in [
            self._execute_goal_handle.STATUS_ACCEPTED,
            self._execute_goal_handle.STATUS_EXECUTING
        ]:
            self.get_logger().info("Tentative d'annulation de l'objectif d'exécution actif...")
            cancel_future = self._execute_goal_handle.cancel_goal_async()
            # Pas besoin d'un rappel séparé, attendre simplement que l'annulation soit terminée
            rclpy.spin_until_future_complete(self, self, cancel_future) # Appel corrigé de spin_until_future_complete
            cancel_response = cancel_future.result()
            if len(cancel_response.goals_canceling) > 0:
                self.get_logger().info("Objectif d'exécution annulé avec succès.")
            else:
                self.get_logger().warn("Échec de l'annulation de l'objectif d'exécution.")
            self._execute_goal_handle = None # Effacer le gestionnaire d'objectif après la tentative d'annulation
        else:
            self.get_logger().info("Aucun objectif d'exécution actif à annuler.")


# Instance globale du nœud pour le gestionnaire de signal
node_instance = None

# Gestionnaire de signal pour un arrêt propre
def signal_handler(sig, frame):
    global node_instance
    if node_instance is not None:
        node_instance.get_logger().info("Ctrl+C reçu. Arrêt du nœud et annulation des objectifs actifs...")
        node_instance.cancel_execution_goal() # Tenter d'annuler l'objectif MoveIt actif
    rclpy.shutdown()
    sys.exit(0)


# --- Logique de calibration et d'automatisation (basée sur la deuxième cellule) ---

def main(args=None):
    rclpy.init(args=args)

    # Vérifier si ArucoDetector est disponible avant de continuer
    if not aruco_detector_available:
        rclpy.shutdown()
        sys.exit(1)

    global node_instance
    node_instance = URMoveitRTDENode()

    # Enregistrer le gestionnaire de signal pour Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    # Définir les obstacles (à partir du main de la première cellule)
    fixed_obstacles = [
        {
            "type": "box",
            "name": "pb_box_1",
            "pose": [-0.205, 0.011, -0.011, 0, 0, 0, 1], # Signes corrigés
            "size": [0.65, 0.65, 0.01]
        },
        {
            "type": "box",
            "name": "pb_box_2",
            "pose": [-0.66, 0.30, 0.3, 0, 0, 0, 1], # Signes corrigés
            "size": [0.055, 0.055, 0.81]
        },
        {
            "type": "box",
            "name": "pb_box_3",
            "pose": [-0.551, 0.30, 0.73, 0, 0, 0, 1], # Signes corrigés
            "size": [0.26, 0.045, 0.045]
        },
        {
            "type": "box",
            "name": "pb_box_4",
            "pose": [-0.46, 0.24, 0.7325, 0, 0, 0, 1], # Signes corrigés
            "size": [0.9, 0.9, 0.027]
        },
        {
            "type": "box",
            "name": "pb_box_5",
            "pose": [0.135, 0.25, 0.085, 0, 0, 0, 1], # Signes corrigés
            "size": [0.015, 0.085, 0.20]
        },
        {
            "type": "box",
            "name": "pb_box_6",
            "pose": [-0.36, 0.03, 0.115, 0, 0, 0, 1], # Signes corrigés
            "size": [0.025, 0.025, 0.185]
        },
        {
            "type": "box",
            "name": "pb_box_7",
            "pose": [-0.36, -0.27, 0.115, 0, 0, 0, 1], # Signes corrigés
            "size": [0.025, 0.025, 0.185]
        },
        {
            "type": "box",
            "name": "pb_box_3_1",
            "pose": [-0.615, 0.30, 0.445, 0, 0, 0, 1], # Signes corrigés
            "size": [0.045, 0.045, 0.045]
        },
        {
            "type": "box",
            "name": "mur",
            "pose": [0.2, 0.0, 0.445, 0, 0, 0, 1], # Signes corrigés
            "size": [0.045, 0.9, 0.9]
        },
        {
            "type": "box",
            "name": "box_8",
            "pose": [-0.45, -0.075, 0.025, 0, 0, 0, 1], # Signes corrigés
            "size": [0.18, 0.39, 0.05]
        }
    ]
     # Objets attachés à l'effecteur final (par exemple, carte Charuco, caméra)
    # La pose ici est relative au lien 'tool0'
    attached_objects = [
        {
            "type": "box", # En supposant que le support de la carte Charuco est une boîte
            "name": "charuco_board_mount",
            "pose": [0.075, 0.0, 0.022, 0.0, 0.0, 0.0, 1.0],
            "size": [0.13, 0.1, 0.04]
        },
        # Ajouter un autre objet pour la carte Charuco elle-même si nécessaire, relatif au support ou à tool0
         {
            "type": "box", # En supposant que la carte Charuco elle-même est une boîte fine
            "name": "charuco_board", # ID unique pour la carte Charuco
            # Pose relative au lien 'tool0' (x, y, z, qx, qy, z, qw) - Corrigé de qz
            # Cette pose serait la pose de l'origine de la carte relative à tool0
            "pose": [0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 1.0], # Exemple : légèrement plus loin que le support
            "size": [0.24, 0.14, 0.01] # Dimensions d'exemple de la carte elle-même
        }
    ]
    end_effector_link_name = "tool0" # Remplacez par le nom du lien de l'effecteur final de votre robot

    # Ajouter des obstacles à la scène de planification
    node_instance.setup_planning_scene(fixed_obstacles=fixed_obstacles, fixed_frame_id="base")
    node_instance.setup_planning_scene(attached_objects=attached_objects, attached_link=end_effector_link_name)

    # Laisser au Planning Scene Monitor de MoveIt le temps de traiter les objets de collision ajoutés
    time.sleep(2.0) # Délai augmenté

    # --- Configuration de la collecte de données de calibration (basée sur la deuxième cellule) ---
    ROBOT_IP = "10.2.30.60"
    # Chemin mis à jour vers intrinsics.yaml - Assurez-vous que ce chemin est correct pour votre caméra fixe
    detector = ArucoDetector('/home/robot/Bureau/ros_moveit/src/ur5_controller/data/intrinsics.yaml')
    # rtde_recv est déjà initialisé dans URMoveitRTDENode

    # Configurer le pipeline RealSense pour la caméra fixe
    pipeline = rs.pipeline()
    cfg = rs.config()
    # En supposant que la caméra fixe est le périphérique RealSense par défaut.
    # Si vous avez plusieurs caméras, vous devrez peut-être spécifier le numéro de série :
    # cfg.enable_device("VOTRE_NUMERO_DE_SERIE_CAMERA")
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    # Si vous utilisez la profondeur, activez-la ici :
    # cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    profile = pipeline.start(cfg)

    # Stockage des données de calibration
    R_base2gripper_list = [] # Rotation de la pose TCP du robot (main par rapport à la base)
    t_base2gripper_list = [] # Translation de la pose TCP du robot (main par rapport à la base)
    R_cam2target_list = [] # Rotation de la pose de la carte (carte par rapport à la caméra fixe)
    t_cam2target_list = [] # Translation de la pose de la carte (carte par rapport à la caméra fixe)


    print("\n=== Collecte automatisée de données de calibration Œil-Main ===\n")
    print("Le robot se déplacera automatiquement pour collecter des points de données.")
    print("La caméra fixe doit observer la carte ChArUco tenue par le robot.")
    print("Assurez-vous que la carte est visible dans le champ de vision de la caméra pour chaque pose.")
    print("Appuyez sur Ctrl+C pour arrêter.\n")

    cv2.namedWindow("Capture de Calibration", cv2.WINDOW_NORMAL)

    sample_count = 0
    min_samples = 15 # Nombre cible d'échantillons pour une bonne calibration
    max_attempts_per_sample = 10 # Nombre maximum de tentatives pour détecter la carte pour chaque échantillon


    # Obtenir la pose TCP initiale à partir de la position actuelle du robot via RTDE pour démarrer la boucle
    starting_tcp_pose = None
    if node_instance.rtde_receive and node_instance.rtde_receive.isConnected():
        # Attendre brièvement que l'interface de réception RTDE soit prête
        time.sleep(0.5)
        starting_tcp_pose = node_instance.rtde_receive.getActualTCPPose()
        if starting_tcp_pose is not None and len(starting_tcp_pose) == 6:
            print(f"Début de la collecte de données de calibration à partir de la pose TCP actuelle du robot : {starting_tcp_pose}")
        else:
            node_instance.get_logger().error("Impossible d'obtenir une pose TCP valide actuelle depuis RTDE. Impossible de démarrer la calibration.")
            pipeline.stop()
            cv2.destroyAllWindows()
            rclpy.shutdown()
            sys.exit(1)
    else:
        node_instance.get_logger().error("Interface de réception RTDE non initialisée ou connectée. Impossible d'obtenir la pose actuelle du robot.")
        pipeline.stop()
        cv2.destroyAllWindows()
        rclpy.shutdown()
        sys.exit(1)

    try:
        # Utiliser la pose TCP actuelle comme base pour le calcul du premier mouvement
        # Cette variable sera mise à jour avec la pose réelle après chaque mouvement réussi
        current_tcp_pose_np = np.array(starting_tcp_pose)


        while rclpy.ok() and sample_count < min_samples:
            attempts = 0
            board_detected = False
            rvec_board_cam, tvec_board_cam = None, None # Variables pour la pose de la carte par rapport à la caméra

            # Tenter de détecter la carte depuis la position actuelle
            while attempts < max_attempts_per_sample and not board_detected:
                frames = pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    node_instance.get_logger().warning("Échec de l'obtention du frame couleur de la caméra.")
                    attempts += 1
                    time.sleep(0.1) # Petit délai avant de réessayer
                    continue

                img = np.asanyarray(color_frame.get_data())

                ok, rvec_board_cam, tvec_board_cam = detector.detect(img)
                display_img = detector.draw_detection(img)

                cv2.putText(display_img, f"Échantillons Collectés : {sample_count}/{min_samples}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

                if ok:
                    cv2.drawFrameAxes(display_img, detector.camera_matrix, detector.dist_coeffs,
                                      rvec_board_cam, tvec_board_cam, 0.05)
                    cv2.putText(display_img, "CARTE DÉTECTÉE (OK)", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    board_detected = True
                else:
                    cv2.putText(display_img, f"CARTE NON DÉTECTÉE (Tentative {attempts + 1})", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    # Si la détection échoue, essayer un petit mouvement correctif ou attendre
                    # Pour simplifier, nous allons juste attendre et réessayer dans cette boucle.
                    # Une approche plus avancée impliquerait un schéma de recherche ou un déplacement vers une pose "sûre" connue.
                    time.sleep(0.5)

                # Afficher la pose TCP actuelle depuis RTDE
                tcp_display = node_instance.rtde_receive.getActualTCPPose()
                if tcp_display is not None and len(tcp_display) == 6:
                     cv2.putText(display_img, f"TCP: X={tcp_display[0]:.3f} Y={tcp_display[1]:.3f} Z={tcp_display[2]:.3f}", (10, 90),
                                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                     cv2.putText(display_img, f"Rot: Rx={tcp_display[3]:.3f} Ry={tcp_display[4]:.3f} Rz={tcp_display[5]:.3f}", (10, 115),
                                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                else:
                     cv2.putText(display_img, "Attente de RTDE TCP...", (10, 90),
                                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)


                cv2.imshow("Capture de Calibration", display_img)
                key = cv2.waitKey(1) & 0xFF # Permettre la vérification de Ctrl+C
                if key == ord('q'):
                    print("Demande d'arrêt.")
                    raise KeyboardInterrupt # Utiliser une exception pour une sortie propre

                rclpy.spin_once(node_instance, timeout_sec=0.01) # Traiter les callbacks

                attempts += 1

            if board_detected:
                # Carte détectée, collecter les données
                tcp_pose = node_instance.rtde_receive.getActualTCPPose()
                if tcp_pose is not None and len(tcp_pose) == 6:
                    # Convertir la pose TCP du robot (axe-angle) en matrice de rotation et vecteur de translation
                    R_hand2base, _ = cv2.Rodrigues(np.array(tcp_pose[3:6]))
                    t_hand2base = np.array(tcp_pose[:3])

                    # Convertir la pose de la carte par rapport à la caméra (rvec, tvec) en matrice de rotation et vecteur de translation
                    R_board2cam, _ = cv2.Rodrigues(rvec_board_cam)
                    t_board2cam = tvec_board_cam.flatten() # S'assurer que tvec est un tableau 1D

                    R_base2gripper_list.append(R_hand2base.tolist())
                    t_base2gripper_list.append(t_hand2base.tolist())
                    R_cam2target_list.append(R_board2cam.tolist())
                    t_cam2target_list.append(t_board2cam.tolist())

                    sample_count += 1
                    print(f"[+] Échantillon {sample_count} collecté")

                    # --- Calculer la prochaine pose basée sur la pose TCP réelle du robot ---
                    # Obtenir la pose TCP actuelle réelle du robot depuis RTDE
                    current_tcp_pose = node_instance.rtde_receive.getActualTCPPose()
                    if current_tcp_pose is None or len(current_tcp_pose) != 6:
                         node_instance.get_logger().error("Impossible d'obtenir une pose TCP valide actuelle depuis RTDE pour le calcul du prochain mouvement.")
                         continue # Passer à l'itération suivante si la pose est invalide

                    current_tcp_pose_np = np.array(current_tcp_pose)


                    # Définir un petit déplacement dans le frame de base du robot
                    # Faire varier aléatoirement les translations et rotations dans une petite plage
                    # Ces valeurs seront ajoutées à la pose TCP actuelle
                    delta_x = np.random.uniform(-0.01, 0.01) # Réduit +/- 2 cm en X
                    delta_y = np.random.uniform(-0.01, 0.01) # Réduit +/- 2 cm en Y
                    delta_z = np.random.uniform(-0.01, 0.01) # Réduit +/- 2 cm en Z
                    delta_rx = np.random.uniform(-np.deg2rad(5), np.deg2rad(5)) # Réduit +/- 5 degrés autour de Rx
                    delta_ry = np.random.uniform(-np.deg2rad(5), np.deg2rad(5)) # Réduit +/- 5 degrés autour de Ry
                    delta_rz = np.random.uniform(-np.deg2rad(5), np.deg2rad(5)) # Réduit +/- 5 degrés autour de Rz


                    delta_pose = np.array([delta_x, delta_y, delta_z, delta_rx, delta_ry, delta_rz])

                    # Calculer la nouvelle pose TCP cible en ajoutant le delta à la pose actuelle
                    target_tcp_pose = current_tcp_pose_np + delta_pose


                    # Optionnel : Ajouter des limites à target_tcp_pose pour rester dans une zone de travail sûre
                    # Exemple de limites (ajuster en fonction de l'espace de travail et de la configuration de votre robot)
                    x_limit = [-0.6, 0.6] # Limites d'exemple
                    y_limit = [-0.6, 0.6] # Limites d'exemple
                    z_limit = [0.1, 0.8] # Limites d'exemple (Éviter de toucher la table ou d'aller trop haut)
                    target_tcp_pose[0] = np.clip(target_tcp_pose[0], x_limit[0], x_limit[1])
                    target_tcp_pose[1] = np.clip(target_tcp_pose[1], y_limit[0], y_limit[1])
                    target_tcp_pose[2] = np.clip(target_tcp_pose[2], z_limit[0], z_limit[1])
                    # Les limites articulaires sont gérées par la planification de MoveIt, mais les limites cartésiennes peuvent aider à guider le planificateur

                    print(f"Tentative de déplacement vers la prochaine pose cible : {target_tcp_pose.tolist()}")
                    success = node_instance.plan_and_execute_tcp_pose_goal(target_tcp_pose.tolist(), goal_frame_id="base")

                    if not success:
                         node_instance.get_logger().warning(f"Échec du déplacement vers la pose {target_tcp_pose.tolist()}. Tentative de retrouver la carte depuis la position actuelle.")
                         # Si le mouvement échoue, la boucle essaiera de détecter à nouveau la carte depuis la position actuelle
                         # Nous pourrions vouloir implémenter une stratégie de récupération plus robuste ici,
                         # par exemple, revenir à une pose légèrement différente ou demander une intervention manuelle.
                         # Pour l'instant, nous laissons la boucle continuer et réessayer la détection.
                    else:
                        # Ajouter un court délai après un mouvement réussi pour laisser le robot se stabiliser
                        time.sleep(0.5)


                else:
                    node_instance.get_logger().error("Impossible d'obtenir une pose TCP valide depuis RTDE après le mouvement.")

            else:
                # Carte non détectée après le nombre maximum de tentatives depuis la position actuelle
                node_instance.get_logger().warning("Carte non détectée après plusieurs tentatives depuis la position actuelle. Impossible de collecter l'échantillon.")
                # Si la détection échoue à plusieurs reprises, essayer de se déplacer vers une pose légèrement différente aléatoirement
                node_instance.get_logger().info("Tentative d'un petit déplacement aléatoire pour retrouver la carte.")

                # Obtenir la pose TCP actuelle réelle du robot depuis RTDE
                current_tcp_pose = node_instance.rtde_receive.getActualTCPPose()
                if current_tcp_pose is None or len(current_tcp_pose) != 6:
                     node_instance.get_logger().error("Impossible d'obtenir une pose TCP valide actuelle depuis RTDE pour le mouvement de récupération.")
                     print("La collecte automatisée a échoué en raison de l'incapacité à détecter la carte et à obtenir la pose actuelle.")
                     break # Sortir de la boucle si nous ne pouvons même pas obtenir la pose actuelle

                current_tcp_pose_np = np.array(current_tcp_pose)

                # Définir un déplacement aléatoire légèrement plus important pour la récupération
                delta_x = np.random.uniform(-0.03, 0.03)
                delta_y = np.random.uniform(-0.03, 0.03)
                delta_z = np.random.uniform(-0.03, 0.03)
                delta_rx = np.random.uniform(-np.deg2rad(7), np.deg2rad(7))
                delta_ry = np.random.uniform(-np.deg2rad(7), np.deg2rad(7))
                delta_rz = np.random.uniform(-np.deg2rad(7), np.deg2rad(7))

                recovery_delta_pose = np.array([delta_x, delta_y, delta_z, delta_rx, delta_ry, delta_rz])
                recovery_target_tcp_pose = current_tcp_pose_np + recovery_delta_pose

                # Appliquer également les limites à la pose de récupération
                x_limit = [-0.6, 0.6]
                y_limit = [-0.6, 0.6]
                z_limit = [0.1, 0.8]
                recovery_target_tcp_pose[0] = np.clip(recovery_target_tcp_pose[0], x_limit[0], x_limit[1])
                recovery_target_tcp_pose[1] = np.clip(recovery_target_tcp_pose[1], y_limit[0], y_limit[1])
                recovery_target_tcp_pose[2] = np.clip(recovery_target_tcp_pose[2], z_limit[0], z_limit[1])


                print(f"Tentative de mouvement de récupération vers la pose : {recovery_target_tcp_pose.tolist()}")
                recovery_success = node_instance.plan_and_execute_tcp_pose_goal(recovery_target_tcp_pose.tolist(), goal_frame_id="base")

                if not recovery_success:
                     node_instance.get_logger().error("Le mouvement de récupération a échoué. Arrêt de la collecte automatisée.")
                     print("La collecte automatisée a échoué après la tentative de récupération.")
                     break # Sortir de la boucle si le mouvement de récupération échoue


    except KeyboardInterrupt:
        print("Interruption clavier reçue. Arrêt de la collecte de données.")
    except Exception as e:
        node_instance.get_logger().error(f"Une erreur est survenue pendant la collecte de données de calibration : {e}")
    finally:
        print("\nCollecte de données de calibration terminée.")
        pipeline.stop()
        cv2.destroyAllWindows()

    # --- Enregistrer les données ---
    if sample_count >= min_samples: # Enregistrer si suffisamment d'échantillons sont collectés
        data = {
            'R_base2gripper_list': R_base2gripper_list, # Rotation de la pose TCP du robot (main par rapport à la base)
            't_base2gripper_list': t_base2gripper_list, # Translation de la pose TCP du robot (main par rapport à la base)
            'R_cam2target_list': R_cam2target_list, # Rotation de la pose de la carte (carte par rapport à la caméra fixe)
            't_cam2target_list': t_cam2target_list  # Translation de la pose de la carte (carte par rapport à la caméra fixe)
        }

        # Définir le répertoire de sortie et le nom du fichier
        output_dir = '/home/robot/Bureau/ros_moveit/src/ur5_controller/data/'
        output_filename = 'eye_to_hand_calibration_data.yaml' # Nom du fichier modifié
        output_filepath = os.path.join(output_dir, output_filename)

        # S'assurer que le répertoire existe
        os.makedirs(output_dir, exist_ok=True)


        try:
            with open(output_filepath, 'w') as f:
                yaml.dump(data, f)
            print(f"\nDonnées de calibration enregistrées dans {output_filepath} ({sample_count} échantillons)")
            print("\nPour effectuer la calibration œil-main en utilisant ces données, vous aurez besoin d'une fonction qui prend ces listes en entrée.")
            print("Par exemple, en supposant que vous ayez une fonction 'calibrate_eye_to_hand' :")
            print("from pick_and_place_system import calibrate_eye_to_hand")
            print("import yaml\n")
            print(f"with open('{output_filepath}', 'r') as f:")
            print("    data = yaml.safe_load(f)\n")
            print("calibrate_eye_to_hand(")
            print("    data['R_base2gripper_list'],")
            print("    data['t_base2gripper_list'],")
            print("    data['R_cam2target_list'],")
            print("    data['t_cam2target_list'],")
            print(f"    '{output_filepath.replace('_data.yaml', '.yaml')}' # Sortir le résultat de la calibration dans un nouveau fichier")
            print(")")


        except IOError as e:
            node_instance.get_logger().error(f"Échec de l'enregistrement des données de calibration dans {output_filepath} : {e}")


    else:
        print(f"\nPas assez d'échantillons collectés ({sample_count}). Minimum {min_samples} requis pour l'enregistrement. Données non enregistrées.")


    # Nettoyer les connexions RTDE (géré par le destructeur de URMoveitRTDENode si nécessaire, mais explicite est plus sûr)
    if node_instance.rtde_receive and node_instance.rtde_receive.isConnected():
        node_instance.rtde_receive.disconnect()

    if node_instance.rtde_control and node_instance.rtde_control.isConnected():
        node_instance.rtde_control.servoStop() # S'assurer que le mode servo est arrêté s'il a été utilisé
        node_instance.rtde_control.disconnect()


    node_instance.destroy_node()
    # rclpy.shutdown() # rclpy.shutdown est maintenant appelé dans le gestionnaire de signal si le signal est intercepté

if __name__ == '__main__':
    main()
