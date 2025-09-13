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
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import threading # Import threading for waiting on futures

class URMoveitRTDENode(Node):
    def __init__(self):
        super().__init__('ur_moveit_rtde_node')
        self.get_logger().info("Starting URMoveitRTDENode")

        # Initialize RTDE interface
        self.robot_ip = "10.2.30.60" # Replace with your robot's IP address
        self.rtde_control = None
        self.rtde_receive = None

        try:
            self.rtde_control = rtde_control.RTDEControlInterface(self.robot_ip)
            self.rtde_receive = rtde_receive.RTDEReceiveInterface(self.robot_ip)
            self.get_logger().info(f"Connected to UR robot at {self.robot_ip}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to UR robot at {self.robot_ip}: {e}")
            # Don't shutdown immediately, allow node to start and potentially attempt reconnection
            # rclpy.shutdown()
            # return

        # Initialize Action Clients for MoveIt 2
        self._move_group_action_client = ActionClient(self, MoveGroup, '/move_action')
        self._execute_trajectory_action_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

        self.get_logger().info("Waiting for MoveGroup action server...")
        self._move_group_action_client.wait_for_server()
        self.get_logger().info("MoveGroup action server connected.")

        self.get_logger().info("Waiting for ExecuteTrajectory action server...")
        self._execute_trajectory_action_client.wait_for_server()
        self.get_logger().info("ExecuteTrajectory action server connected.")


        # Optional: IK Service Client (if you plan to use pose goals)
        # self._get_ik_client = self.create_client(GetPositionIK, '/compute_ik')
        # while not self._get_ik_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('IK service not available, waiting again...')
        # self.get_logger().info("IK service connected.")

        # Publisher for the Planning Scene Interface
        # We use a latching publisher to ensure the scene is received
        self.planning_scene_publisher = self.create_publisher(CollisionObject, '/collision_object', 10) # Qos profile might need adjustment for latching if default isn't sufficient
        self.get_logger().info("Planning Scene publisher created.")

        # Futures for action clients
        self._send_plan_goal_future = None
        self._get_plan_result_future = None
        self._send_execute_goal_future = None
        self._get_execute_result_future = None

    def add_collision_object(self, object_id, shape_type, dimensions, pose):
        """
        Adds a collision object to the planning scene.

        :param object_id: A unique string ID for the object.
        :param shape_type: The type of shape (e.g., SolidPrimitive.BOX, SolidPrimitive.SPHERE).
        :param dimensions: A list of floats representing the dimensions of the shape (e.g., [x, y, z] for a box).
        :param pose: A geometry_msgs.msg.Pose object for the object's position and orientation.
        """
        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"  # Or your robot's base frame
        collision_object.id = object_id

        primitive = SolidPrimitive()
        primitive.type = shape_type
        primitive.dimensions = dimensions
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)

        collision_object.operation = CollisionObject.ADD

        self.planning_scene_publisher.publish(collision_object)
        self.get_logger().info(f"Added collision object: {object_id}")
        # Add a small delay to allow MoveIt's Planning Scene Monitor to process the update
        time.sleep(0.1) # Adjust if needed


    def remove_collision_object(self, object_id):
        """
         Removes a collision object from the planning scene.

        :param object_id: The string ID of the object to remove.
        """
        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"  # Or your robot's base frame
        collision_object.id = object_id
        collision_object.operation = CollisionObject.REMOVE

        self.planning_scene_publisher.publish(collision_object)
        self.get_logger().info(f"Removed collision object: {object_id}")
        # Add a small delay to allow MoveIt's Planning Scene Monitor to process the update
        time.sleep(0.1) # Adjust if needed


    def plan_trajectory(self, goal_pose=None, joint_goal=None):
        self.get_logger().info("Planning trajectory...")

        goal_msg = MoveGroup.Goal()
        request = MotionPlanRequest()

        # Set planning group
        request.group_name = "ur_manipulator" # Assuming "ur_manipulator" is your planning group name

        # Set the start state to the robot's current state
        if self.rtde_receive:
            # Add a small delay to ensure RTDE receive interface is ready
            time.sleep(0.05) # Adjust as needed
            current_joint_state = self.rtde_receive.getActualQ()
            if current_joint_state:
                robot_state = RobotState()
                # Assuming your robot's joint names are these. Adjust if necessary.
                robot_state.joint_state.name = [
                    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
                ]
                robot_state.joint_state.position = list(current_joint_state)
                request.start_state = robot_state
                self.get_logger().info("Set planning start state to current robot joint state.")
            else:
                 self.get_logger().warn("Could not get current joint state from RTDE. Planning may use default start state.")
        else:
             self.get_logger().warn("RTDE receive interface not initialized. Cannot get current robot state for planning.")


        # Set goal constraints
        goal_constraints = Constraints()

        if joint_goal is not None:
            self.get_logger().info("Setting joint goal constraints.")
            joint_constraints = []
            joint_names = [
                'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
            ] # Replace with your robot's joint names if different

            for i, joint_name in enumerate(joint_names):
                constraint = JointConstraint()
                constraint.joint_name = joint_name
                constraint.position = joint_goal[i]
                constraint.tolerance_above = 0.01  # Adjust tolerance as needed
                constraint.tolerance_below = 0.01  # Adjust tolerance as needed
                constraint.weight = 1.0
                joint_constraints.append(constraint)

            goal_constraints.joint_constraints = joint_constraints

        elif goal_pose is not None:
            self.get_logger().info("Setting pose goal constraints.")
            # Assuming the pose is for the end-effector link (tool0 or similar)
            end_effector_link = "tool0" # Replace with your robot's end-effector link name

            position_constraint = PositionConstraint()
            position_constraint.header.frame_id = "world" # Or your robot's base frame
            position_constraint.link_name = end_effector_link
            position_constraint.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01])) # A small sphere around the target point
            position_constraint.constraint_region.primitive_poses.append(goal_pose)
            position_constraint.weight = 1.0

            orientation_constraint = OrientationConstraint()
            orientation_constraint.header.frame_id = "world" # Or your robot's base frame
            orientation_constraint.link_name = end_effector_link
            orientation_constraint.orientation = goal_pose.orientation
            orientation_constraint.absolute_x_axis_tolerance = 0.01 # Adjust tolerance as needed
            orientation_constraint.absolute_y_axis_tolerance = 0.01 # Adjust tolerance as needed
            orientation_constraint.absolute_z_axis_tolerance = 0.01 # Adjust tolerance as needed
            orientation_constraint.weight = 1.0

            goal_constraints.position_constraints.append(position_constraint)
            goal_constraints.orientation_constraints.append(orientation_constraint)

        else:
            self.get_logger().error("No goal (joint_goal or goal_pose) provided for planning.")
            return

        request.goal_constraints.append(goal_constraints)

        # Set planning options
        request.num_planning_attempts = 10
        # request.planning_time = 5.0 # This attribute does not exist in Humble's MotionPlanRequest
        request.planner_id = "RRTstar" # Example planner, adjust based on your MoveIt 2 setup
        # request.allowed_planning_time = 5.0 # Use allowed_planning_time instead if available

        goal_msg.request = request

        self.get_logger().info("Sending planning goal to MoveGroup action server...")
        self._send_plan_goal_future = self._move_group_action_client.send_goal_async(goal_msg)
        self._send_plan_goal_future.add_done_callback(self.plan_goal_response_callback)

    def plan_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Planning goal rejected.')
            return

        self.get_logger().info('Planning goal accepted. Waiting for result...')
        self._get_plan_result_future = goal_handle.get_result_async()
        # We will wait for the result in the main execution flow or a separate thread
        # self._get_plan_result_future.add_done_callback(self.plan_result_callback) # Remove direct callback


    def plan_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == result.error_code.SUCCESS:
            self.get_logger().info("Planning successful.")
            trajectory = result.planned_trajectory.joint_trajectory
            if trajectory.points:
                 self.get_logger().info(f"Executing trajectory via RTDE servoj with {len(trajectory.points)} points...")
                 self.execute_trajectory_rtde(trajectory)
            else:
                 self.get_logger().warn("Planned trajectory has no points.")

        else:
            self.get_logger().error(f"Planning failed with error code: {result.error_code.val}")
            # You might want to print more details about the error if available
            # self.get_logger().error(f"Planning error: {result.error_code.text}")


    def execute_trajectory_rtde(self, trajectory: JointTrajectory):
        if not self.rtde_control:
            self.get_logger().error("RTDE control interface not initialized.")
            return

        self.get_logger().info(f"Executing {len(trajectory.points)} waypoints...")

        # Send waypoints using servoj
        # Note: servoj requires a time parameter for each point.
        # We will use the time_from_start from the trajectory points
        # and calculate the time difference between points.

        if not trajectory.points:
            self.get_logger().warn("Trajectory has no points to execute.")
            return

        # Get current joint state for initial point reference if needed
        # current_joint_state = self.rtde_receive.getActualQ()
        # self.get_logger().info(f"Current joint state: {current_joint_state}")


        for i, point in enumerate(trajectory.points):
            if i == 0:
                # For the first point, the time to reach it is its time_from_start
                dt = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            else:
                # For subsequent points, calculate the time difference from the previous point
                prev_point = trajectory.points[i-1]
                dt = (point.time_from_start.sec - prev_point.time_from_start.sec) + \
                     (point.time_from_start.nanosec - prev_point.time_from_start.nanasec) / 1e9

            # Ensure dt is positive and reasonable
            if dt <= 0:
                self.get_logger().warn(f"Calculated dt is non-positive for point {i+1}. Skipping or adjusting.")
                dt = 1 # Assign a small positive value to avoid issues, adjust as needed

            # Use velocities and accelerations from the trajectory if available, otherwise use defaults
            velocity = 0.01 # Default velocity if not provided
            acceleration = 0.005 # Default acceleration if not provided
            if point.velocities:
                 # Assuming velocities are provided for all joints
                 velocity = np.mean(np.abs(point.velocities)) # Example: Use mean absolute velocity

            # blend parameter (arc blend) - usually 0.0 for point-to-point movement
            blend = 0.0

            # servoj takes joint angles in radians, velocity, acceleration, dt, blend
            servoj_command = list(point.positions) + [velocity, acceleration, dt, blend]

            try:
                self.get_logger().info(f"Sending servoj command for waypoint {i+1}/{len(trajectory.points)} with dt: {dt:.4f}")
                self.rtde_control.servoj(servoj_command)
                # Note: servoj is a non-blocking command. The sleep here is
                # a simplified way to space out commands. A more robust
                # implementation would involve monitoring robot state or
                # using RTDE synchronization features.
                time.sleep(dt * 0.8) # Sleep slightly less than dt to account for communication and robot execution time
            except Exception as e:
                self.get_logger().error(f"Error sending servoj command for waypoint {i+1}: {e}")
                break

        self.get_logger().info("Trajectory execution finished.")


def main(args=None):
    rclpy.init(args=args)
    node = URMoveitRTDENode()

    # Define obstacles
    obstacles = [
        {
            "type": "box",
            "name": "pb_box_1",
            "pose": [-0.205, 0.0, -0.011, 0, 0, 0, 1],
            "size": [0.65, 0.65, 0.01]
        },
        {
            "type": "box",
            "name": "pb_box_2",
            "pose": [-0.8, 0.30, 0.4, 0, 0, 0, 1],
            "size": [0.02 * 2, 0.04, 0.4 * 2]
        },
        {
            "type": "box",
            "name": "pb_box_3",
            "pose": [-0.675, 0.30, 0.82, 0, 0, 0, 1],
            "size": [0.25, 0.04, 0.04]
        },
        {
            "type": "box",
            "name": "pb_box_4",
            "pose": [-0.675, 0.24, 0.8325, 0, 0, 0, 1],
            "size": [0.09, 0.025, 0.026]
        }
    ]
    
    """
    obstacles = [
        {
            "type": "box",
            "name": "pb_box_1",
            "pose": [0.205, 0.0, -0.02, 0, 0, 0, 1],
            "size": [0.325 * 2, 0.325 * 2, 0.01 * 2]
        },
        {
            "type": "box",
            "name": "pb_box_2",
            "pose": [0.8, -0.30, 0.4, 0, 0, 0, 1],
            "size": [0.04, 0.04, 0.4 * 2]
        },
        {
            "type": "box",
            "name": "pb_box_3",
            "pose": [0.675, -0.30, 0.82, 0, 0, 0, 1],
            "size": [0.25, 0.02 * 2, 0.02 * 2]
        },
        {
            "type": "box",
            "name": "pb_box_4",
            "pose": [0.675, -0.24, 0.8325, 0, 0, 0, 1],
            "size": [0.09, 0.025, 0.026]
        }
    ]
    """
    
    # Add obstacles to the planning scene
    for obstacle in obstacles:
        if obstacle["type"] == "box":
            obstacle_pose = Pose()
            obstacle_pose.position.x = float(obstacle["pose"][0])
            obstacle_pose.position.y = float(obstacle["pose"][1])
            obstacle_pose.position.z = float(obstacle["pose"][2])
            # Assuming pose is [x, y, z, qx, qy, qz, qw]
            obstacle_pose.orientation.x = float(obstacle["pose"][3])
            obstacle_pose.orientation.y = float(obstacle["pose"][4])
            obstacle_pose.orientation.z = float(obstacle["pose"][5])
            obstacle_pose.orientation.w = float(obstacle["pose"][6])
            obstacle_dimensions = obstacle["size"]
            node.add_collision_object(obstacle["name"], SolidPrimitive.BOX, obstacle_dimensions, obstacle_pose)
        # Add more shapes here if needed (e.g., if obstacle["type"] == "sphere": ...)

    # Give MoveIt's Planning Scene Monitor time to process the added collision objects
    time.sleep(1.0) # Adjust this delay if necessary

    # Example: Plan to a pose goal (will now consider the added obstacles)
    # Replace with your desired pose goal
    goal_pose = Pose()
    goal_pose.position.x = -0.073
    goal_pose.position.y = 0.6
    goal_pose.position.z = 0.5
    goal_pose.orientation.x = 2.6
    goal_pose.orientation.y = 2.0
    goal_pose.orientation.z = 2.10
    goal_pose.orientation.w = 1.0 # Identity quaternion for no rotation

    # Instead of a direct callback, wait for the planning result here
    node.plan_trajectory(goal_pose=goal_pose)

    # Wait for the planning goal to be accepted
    if node._send_plan_goal_future:
        rclpy.spin_until_future_complete(node, node._send_plan_goal_future)
        goal_handle = node._send_plan_goal_future.result()
        if goal_handle and goal_handle.accepted:
            node.get_logger().info("Planning goal accepted, waiting for result...")
            # Wait for the planning result
            if node._get_plan_result_future:
                 rclpy.spin_until_future_complete(node, node._get_plan_result_future)
                 node.plan_result_callback(node._get_plan_result_future) # Process the result
        else:
             node.get_logger().error("Planning goal was not accepted.")


    rclpy.spin(node)

    # Optional: Remove obstacles when done
    # for obstacle in obstacles:
    #     node.remove_collision_object(obstacle["name"])

    # Clean up RTDE connection
    if node.rtde_control:
        node.rtde_control.disconnect()
    if node.rtde_receive:
        node.rtde_receive.disconnect()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
