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
from moveit_msgs.msg import CollisionObject, PlanningScene, AttachedCollisionObject # Importer PlanningScene et AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import threading # Importer threading pour attendre les futurs
from scipy.spatial.transform import Rotation as R # Importer scipy pour la conversion axe-angle en quaternion
import signal # Importer signal pour gérer les interruptions
import sys # Importer sys pour sortir proprement


class URMoveitRTDENode(Node):
    def __init__(self):
        super().__init__('ur_moveit_rtde_node')
        self.get_logger().info("Starting URMoveitRTDENode")

        # Initialiser l'interface RTDE
        # RTDE receive est toujours nécessaire pour obtenir l'état initial du robot pour la planification
        self.robot_ip = "10.2.30.60" # Remplacer par l'adresse IP de votre robot
        self.rtde_control = None # Le contrôle RTDE n'est pas nécessaire pour l'exécution MoveIt
        self.rtde_receive = None

        try:
            self.rtde_receive = rtde_receive.RTDEReceiveInterface(self.robot_ip)
            self.get_logger().info(f"Connected to UR robot at {self.robot_ip} for RTDE receive.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to UR robot at {self.robot_ip} for RTDE receive: {e}")
            # Permettre au nœud de démarrer, mais la planification peut échouer sans l'état actuel


        # Initialiser les clients d'action pour MoveIt 2
        self._move_group_action_client = ActionClient(self, MoveGroup, '/move_action')
        self._execute_trajectory_action_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

        self.get_logger().info("Waiting for MoveGroup action server...")
        self._move_group_action_client.wait_for_server()
        self.get_logger().info("MoveGroup action server connected.")

        self.get_logger().info("Waiting for ExecuteTrajectory action server...")
        self._execute_trajectory_action_client.wait_for_server()
        self.get_logger().info("ExecuteTrajectory action server connected.")


        # Client de service IK optionnel (si vous prévoyez d'utiliser des objectifs de pose)
        # self._get_ik_client = self.create_client(GetPositionIK, '/compute_ik')
        # while not self._get_ik_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('IK service not available, waiting again...')
        # self.get_logger().info("IK service connected.")

        # Éditeur pour l'interface de scène de planification
        # Nous utilisons un éditeur latching pour garantir que la scène est reçue
        self.planning_scene_publisher = self.create_publisher(PlanningScene, '/planning_scene', 10) # Utilisation du message PlanningScene
        self.get_logger().info("Planning Scene publisher created.")

        # Futurs pour les clients d'action et les gestionnaires d'objectifs
        self._send_plan_goal_future = None
        self._get_plan_result_future = None
        self._send_execute_goal_future = None
        self._get_execute_result_future = None
        self._execute_goal_handle = None # Stocker le gestionnaire d'objectifs pour l'annulation


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

        self.get_logger().info(f"Created CollisionObject definition for: {object_id} in frame {frame_id}")
        return collision_object


    def remove_collision_object(self, object_id):
        """
         Supprime un objet de collision de la scène de planification.

        :param object_id: L'ID de chaîne de l'objet à supprimer.
        """
        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"  # L'ID de trame n'a pas strictement d'importance pour la suppression par ID, mais correspond souvent à la trame d'ajout
        collision_object.id = object_id
        collision_object.operation = CollisionObject.REMOVE

        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.world.collision_objects.append(collision_object)

        self.planning_scene_publisher.publish(planning_scene)
        self.get_logger().info(f"Removed collision object: {object_id}")
        # Ajouter un petit délai pour permettre au moniteur de scène de planification de MoveIt de traiter la mise à jour
        time.sleep(0.1) # Ajuster si nécessaire


    def setup_planning_scene(self, fixed_obstacles=None, attached_objects=None, fixed_frame_id="world", attached_link=""):
        """
        Configure la scène de planification en ajoutant des obstacles fixes prédéfinis et des objets attachés.
        """
        self.get_logger().info("Setting up planning scene...")
        planning_scene = PlanningScene()
        planning_scene.is_diff = True

        if fixed_obstacles:
            self.get_logger().info(f"Adding fixed obstacles to the world in {fixed_frame_id} frame...")
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
             self.get_logger().info(f"Adding objects attached to {attached_link}...")
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
                     # La fonction add_collision_object définit déjà la pose dans la trame spécifiée (attached_link)
                     planning_scene.robot_state.attached_collision_objects.append(attached_col_obj)


        self.planning_scene_publisher.publish(planning_scene)
        # Ajouter un petit délai pour permettre au moniteur de scène de planification de MoveIt de traiter la mise à jour
        time.sleep(1.0) # Ajuster si nécessaire
        self.get_logger().info("Planning scene setup complete.")


    def plan_and_execute_pose_goal(self, goal_pose: Pose, goal_frame_id="base"):
        """
        Planifie et exécute une trajectoire vers l'objectif de pose spécifié (orientation quaternion).

        :param goal_pose: La pose cible (geometry_msgs.msg.Pose).
        :param goal_frame_id: La trame de coordonnées dans laquelle l'objectif de pose est défini (par défaut : "base_link").
        """
        self.get_logger().info(f"Planning and executing trajectory to pose goal in {goal_frame_id} frame...")

        goal_msg = MoveGroup.Goal()
        request = MotionPlanRequest()

        # Définir le groupe de planification
        request.group_name = "ur_manipulator" # En supposant que "ur_manipulator" est le nom de votre groupe de planification

        # Définir l'état de départ sur l'état actuel du robot lu par RTDE
        if self.rtde_receive and self.rtde_receive.isConnected():
            # Ajouter un petit délai pour s'assurer que l'interface de réception RTDE est prête
            time.sleep(0.1) # Ajuster si nécessaire
            current_joint_state_list = self.rtde_receive.getActualQ()
            self.get_logger().info(f"RTDE getActualQ() returned: {current_joint_state_list}") # Debug log
            self.get_logger().info(f"Length of getActualQ() result: {len(current_joint_state_list) if current_joint_state_list is not None else 'None'}") # Debug log


            if current_joint_state_list is not None and len(current_joint_state_list) == 6: # En supposant 6 articulations pour UR5
                robot_state = RobotState()
                # Basé sur la sortie de votre /joint_states, l'ordre est :
                # ['shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'shoulder_pan_joint']
                # Utiliser cet ordre pour définir l'état de départ, en supposant que RTDE getActualQ() correspond ou que MoveIt peut gérer cela.
                # Une manière plus robuste serait d'utiliser un dictionnaire de mappage si l'ordre est réellement incohérent.
                # Pour l'instant, s'en tenir à l'ordre observé des /joint_states pour les positions RTDE.
                rtde_joint_names_order = [
                    'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
                    'wrist_2_joint', 'wrist_3_joint', 'shoulder_pan_joint'
                ]

                robot_state.joint_state.name = rtde_joint_names_order
                robot_state.joint_state.position = list(current_joint_state_list) # Attribuer les positions en fonction de cet ordre supposé
                request.start_state = robot_state
                self.get_logger().info(f"Set planning start state to current robot joint state: {current_joint_state_list} with assumed order {rtde_joint_names_order}")
            else:
                 self.get_logger().error("Could not get valid current joint state from RTDE. Planning may fail or use default start state.")
                 if current_joint_state_list is not None:
                     self.get_logger().error(f"Received joint state (list): {current_joint_state_list}")
                 else:
                     self.get_logger().error("RTDE getActualQ() returned None.")
                 # Optionnellement, vous pourriez retourner ici si l'obtention de l'état de départ de RTDE est critique
                 # return


        else:
             self.get_logger().error("RTDE receive interface not initialized or connected. Cannot get current robot state for planning.")
             # Si RTDE n'est pas connecté, MoveIt utilisera probablement un état de départ par défaut.
             # Selon votre configuration, cela pourrait être acceptable pour la planification,
             # mais cela ne reflétera pas la position réelle du robot.


        # Définir les contraintes de l'objectif
        goal_constraints = Constraints()

        self.get_logger().info(f"Setting pose goal constraints in {goal_frame_id} frame.")
        # En supposant que la pose est pour le lien de l'effecteur final (tool0 ou similaire)
        end_effector_link = "tool0" # Remplacer par le nom du lien de l'effecteur final de votre robot

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = goal_frame_id # Définir le frame_id en fonction de l'entrée
        position_constraint.link_name = end_effector_link
        position_constraint.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01])) # Une petite sphère autour du point cible
        position_constraint.constraint_region.primitive_poses.append(goal_pose)
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = goal_frame_id # Définir le frame_id en fonction de l'entrée
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
        # request.planning_time = 5.0 # Cet attribut n'existe pas dans MotionPlanRequest de Humble
        request.planner_id = "RRTstar" # Planificateur exemple, ajuster en fonction de votre configuration MoveIt 2
        # request.allowed_planning_time = 5.0 # Utiliser allowed_planning_time à la place si disponible

        goal_msg.request = request

        self.get_logger().info("Sending planning goal to MoveGroup action server...")
        self._send_plan_goal_future = self._move_group_action_client.send_goal_async(goal_msg)
        self._send_plan_goal_future.add_done_callback(self.plan_goal_response_callback)


    def plan_and_execute_tcp_pose_goal(self, tcp_pose: list, goal_frame_id="base"):
        """
        Planifie et exécute une trajectoire vers l'objectif de pose TCP spécifié (position et orientation axe-angle).

        :param tcp_pose: Une liste [x, y, z, rx, ry, rz] représentant la pose TCP cible.
                         Position en mètres, orientation en axe-angle (radians).
        :param goal_frame_id: La trame de coordonnées dans laquelle l'objectif de pose est défini (par défaut : "base_link").
        """
        if len(tcp_pose) != 6:
            self.get_logger().error("tcp_pose must be a list of 6 values: [x, y, z, rx, ry, rz]")
            return

        x, y, z, rx, ry, rz = tcp_pose

        # Convertir axe-angle [rx, ry, rz] en quaternion [x, y, z, w] en utilisant scipy
        try:
            # Créer un objet Rotation à partir du vecteur axe-angle
            rotation = R.from_rotvec([rx, ry, rz])
            # Obtenir le quaternion au format [x, y, z, w]
            quaternion = rotation.as_quat()
        except Exception as e:
            self.get_logger().error(f"Failed to convert axis-angle to quaternion using scipy: {e}")
            return

        # Créer le message geometry_msgs/Pose
        goal_pose_quat = Pose()
        goal_pose_quat.position.x = float(x)
        goal_pose_quat.position.y = float(y)
        goal_pose_quat.position.z = float(z)
        goal_pose_quat.orientation.x = float(quaternion[0])
        goal_pose_quat.orientation.y = float(quaternion[1])
        goal_pose_quat.orientation.z = float(quaternion[2])
        goal_pose_quat.orientation.w = float(quaternion[3])

        self.get_logger().info(f"Converted TCP pose [x,y,z,rx,ry,rz]: [{x}, {y}, {z}, {rx}, {ry}, {rz}]")
        self.get_logger().info(f"To Quaternion [x,y,z,w]: [{quaternion[0]}, {quaternion[1]}, {quaternion[2]}, {quaternion[3]}]")


        # Appeler la méthode existante plan_and_execute_pose_goal avec la pose quaternion
        self.plan_and_execute_pose_goal(goal_pose_quat, goal_frame_id)


    def plan_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Planning goal rejected.')
            return

        self.get_logger().info('Planning goal accepted. Waiting for result...')
        self._get_plan_result_future = goal_handle.get_result_async()
        # Attendre le résultat et le traiter directement
        rclpy.spin_until_future_complete(self, self._get_plan_result_future)
        self.plan_result_callback(self._get_plan_result_future)


    def plan_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == result.error_code.SUCCESS:
            self.get_logger().info("Planning successful.")
            planned_trajectory = result.planned_trajectory.joint_trajectory
            if planned_trajectory.points:
                 self.get_logger().info(f"Planned trajectory has {len(planned_trajectory.points)} points.")
                 # Envoyer la trajectoire planifiée au serveur d'action ExecuteTrajectory
                 self.execute_trajectory(planned_trajectory)
            else:
                 self.get_logger().warn("Planned trajectory has no points.")

        else:
            self.get_logger().error(f"Planning failed with error code: {result.error_code.val}")


    def execute_trajectory(self, trajectory: JointTrajectory):
        """
        Envoie la trajectoire planifiée au serveur d'action ExecuteTrajectory pour exécution.
        """
        if not self._execute_trajectory_action_client.server_is_ready():
            self.get_logger().error("ExecuteTrajectory action server is not ready.")
            return

        self.get_logger().info(f"Sending trajectory with {len(trajectory.points)} points for execution.")

        execute_goal_msg = ExecuteTrajectory.Goal()
        execute_goal_msg.trajectory = trajectory

        self._send_execute_goal_future = self._execute_trajectory_action_client.send_goal_async(execute_goal_msg)
        self._send_execute_goal_future.add_done_callback(self.execute_goal_response_callback)


    def execute_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Execution goal rejected.')
            self._execute_goal_handle = None # Effacer le gestionnaire d'objectifs si rejeté
            return

        self.get_logger().info('Execution goal accepted. Waiting for result...')
        self._execute_goal_handle = goal_handle # Stocker le gestionnaire d'objectifs
        self._get_execute_result_future = goal_handle.get_result_async()
        # Attendre le résultat et le traiter directement
        rclpy.spin_until_future_complete(self, self._get_execute_result_future)
        self.execute_result_callback(self._get_execute_result_future)


    def execute_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == result.error_code.SUCCESS:
            self.get_logger().info("Trajectory execution successful.")
        else:
            self.get_logger().error(f"Trajectory execution failed with error code: {result.error_code.val}")
        self._execute_goal_handle = None # Effacer le gestionnaire d'objectifs après l'exécution


    def cancel_execution_goal(self):
        """
        Envoie une requête d'annulation à l'objectif d'action ExecuteTrajectory actif.
        """
        if self._execute_goal_handle is not None and self._execute_goal_handle.status in [
            self._execute_goal_handle.STATUS_ACCEPTED,
            self._execute_goal_handle.STATUS_EXECUTING
        ]:
            self.get_logger().info("Attempting to cancel active execution goal...")
            cancel_future = self._execute_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_execution_response_callback)
        else:
            self.get_logger().info("No active execution goal to cancel.")

    def cancel_execution_response_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("Execution goal successfully cancelled.")
        else:
            self.get_logger().warn("Failed to cancel execution goal.")
        self._execute_goal_handle = None # Effacer le gestionnaire d'objectifs après la tentative d'annulation


# Instance de nœud globale pour le gestionnaire de signaux
node_instance = None

# Gestionnaire de signaux pour une fermeture propre
def signal_handler(sig, frame):
    global node_instance
    if node_instance is not None:
        node_instance.get_logger().info("Ctrl+C received. Shutting down node and cancelling active goals...")
        node_instance.cancel_execution_goal() # Tenter d'annuler l'objectif MoveIt actif
    rclpy.shutdown()
    sys.exit(0)


def main(args=None):
    rclpy.init(args=args)

    global node_instance
    node_instance = URMoveitRTDENode()

    # Enregistrer le gestionnaire de signaux pour Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)


    # Définir les obstacles
    fixed_obstacles = [
        {
            "type": "box",
            "name": "pb_box_1",
            "pose": [-0.205, 0.011, -0.011, 0, 0, 0, 1],
            "size": [0.65, 0.65, 0.01]
        },
        {
            "type": "box",
            "name": "pb_box_2",
            "pose": [-0.66, 0.30, 0.3, 0, 0, 0, 1],
            "size": [0.045, 0.045, 0.81]
        },
        {
            "type": "box",
            "name": "pb_box_3",
            "pose": [-0.551, 0.30, 0.73, 0, 0, 0, 1],
            "size": [0.26, 0.045, 0.045]
        },
        {
            "type": "box",
            "name": "pb_box_4",
            "pose": [-0.46, 0.24, 0.7325, 0, 0, 0, 1],
            "size": [0.9, 0.9, 0.027]
        },
        {
            "type": "box",
            "name": "pb_box_5",
            "pose": [0.135, 0.25, 0.085, 0, 0, 0, 1],
            "size": [0.015, 0.085, 0.20]
        },
        {
            "type": "box",
            "name": "pb_box_6",
            "pose": [-0.36, 0.03, 0.115, 0, 0, 0, 1],
            "size": [0.025, 0.025, 0.185]
        },
        {
            "type": "box",
            "name": "pb_box_7",
            "pose": [-0.36, -0.27, 0.115, 0, 0, 0, 1],
            "size": [0.025, 0.025, 0.185]
        },
        {
            "type": "box",
            "name": "pb_box_3_1",
            "pose": [-0.615, 0.30, 0.445, 0, 0, 0, 1],
            "size": [0.045, 0.045, 0.045]
        },
        {
            "type": "box",
            "name": "mur",
            "pose": [0.35, 0.0, 0.445, 0, 0, 0, 1],
            "size": [0.045, 0.9, 0.9]
        }
    ]
     # Objets attachés à l'effecteur final (par exemple, carte Charuco, caméra)
    # La pose ici est relative au lien 'tool0'
    attached_objects = [
        {
            "type": "box", # En supposant que le support de la carte Charuco est une boîte
            "name": "charuco_board_mount", # ID unique pour l'objet attaché
            # Pose relative au lien 'tool0' (x, y, z, qx, qy, qz, qw)
            # Vous devrez déterminer cette pose en fonction de votre configuration physique
            "pose": [0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 1.0], # Exemple : 10 cm dans la direction Z par rapport à tool0
            "size": [0.2, 0.1, 0.02] # Dimensions exemples du support
        },
         {
            "type": "box", # En supposant que la carte Charuco elle-même est une boîte fine
            "name": "charuco_board", # ID unique pour la carte Charuco
            # Pose relative au lien 'tool0' (x, y, z, qx, qy, z, qw) - Corrigé de qz
            # Cette pose serait la pose de l'origine de la carte par rapport à tool0
            "pose": [0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 1.0], # Exemple : légèrement plus loin que le support
            "size": [0.2, 0.15, 0.005] # Dimensions exemples de la carte elle-même
        }
    ]
    end_effector_link_name = "tool0" # Remplacer par le nom du lien de l'effecteur final de votre robot


    # Ajouter des obstacles à la scène de planification
    # Utiliser la trame "world" pour les obstacles s'ils sont fixes dans l'environnement
    # Utiliser la trame "base_link" s'ils se déplacent avec la base du robot ou sont définis par rapport à elle
    node_instance.setup_planning_scene(fixed_obstacles=fixed_obstacles, fixed_frame_id="base")
    # Si vous avez des obstacles définis par rapport à base_link :
    node_instance.setup_planning_scene(attached_objects=attached_objects, attached_link=end_effector_link_name)



    # Laisser le temps au moniteur de scène de planification de MoveIt de traiter les objets de collision ajoutés
    time.sleep(1.0) # Ajuster ce délai si nécessaire

    # --- Demande de planification et d'exécution ---

    # Exemple d'utilisation de la nouvelle méthode avec la pose TCP au format [x, y, z, rx, ry, rz]
    # Remplacer par les valeurs de pose TCP souhaitées depuis Polyscope
    target_tcp_pose = [-0.2, 0.4, 0.4, -1.57, 3.14, 0.0] # Valeurs exemples

    # Planifier et exécuter la trajectoire en utilisant l'entrée de pose TCP
    node_instance.plan_and_execute_tcp_pose_goal(target_tcp_pose, goal_frame_id="base")


    # Faire tourner le nœud avec une boucle qui vérifie rclpy.ok()
    try:
        while rclpy.ok():
            rclpy.spin_once(node_instance, timeout_sec=0.1) # Traiter les rappels périodiquement
    except KeyboardInterrupt:
        pass # Ctrl+C sera géré par le gestionnaire de signaux


    # Optionnel : Supprimer les obstacles lorsque vous avez terminé
    #node_instance.remove_collision_object("base_link_box_1")
    # node_instance.remove_collision_object("pb_box_2")
    # node_instance.remove_collision_object("pb_box_3")
    # node_instance.remove_collision_object("pb_box_4")


    # Nettoyer la connexion de réception RTDE
    if node_instance.rtde_receive and node_instance.rtde_receive.isConnected():
        node_instance.rtde_receive.disconnect()

    # Nettoyer la connexion de contrôle RTDE si elle a été utilisée
    if node_instance.rtde_control and node_instance.rtde_control.isConnected():
        node_instance.rtde_control.servoStop() # S'assurer que le mode servo est arrêté s'il a été utilisé
        node_instance.rtde_control.disconnect()


    node_instance.destroy_node()

if __name__ == '__main__':
    main()
