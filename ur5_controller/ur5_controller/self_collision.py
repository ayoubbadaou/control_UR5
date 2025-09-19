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
# import transforms3d # Import for axis-angle to quaternion conversion - Replaced with scipy
from scipy.spatial.transform import Rotation as R # Import scipy for axis-angle to quaternion conversion
import signal # Import signal for handling interrupts
import sys # Import sys for exiting cleanly


class URMoveitRTDENode(Node):
    def __init__(self):
        super().__init__('ur_moveit_rtde_node')
        self.get_logger().info("Starting URMoveitRTDENode")

        # Initialize RTDE interface
        # RTDE receive is still needed to get the initial robot state for planning
        self.robot_ip = "10.2.30.60" # Replace with your robot's IP address
        self.rtde_control = None # RTDE control not needed for MoveIt execution
        self.rtde_receive = None

        try:
            self.rtde_receive = rtde_receive.RTDEReceiveInterface(self.robot_ip)
            self.get_logger().info(f"Connected to UR robot at {self.robot_ip} for RTDE receive.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to UR robot at {self.robot_ip} for RTDE receive: {e}")
            # Allow node to start, but planning may fail without current state


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

        # Futures for action clients and goal handles
        self._send_plan_goal_future = None
        self._get_plan_result_future = None
        self._send_execute_goal_future = None
        self._get_execute_result_future = None
        self._execute_goal_handle = None # Store the goal handle for cancellation


    def add_collision_object(self, object_id, shape_type, dimensions, pose, frame_id="base"):
        """
        Adds a collision object to the planning scene.

        :param object_id: A unique string ID for the object.
        :param shape_type: The type of shape (e.g., SolidPrimitive.BOX, SolidPrimitive.SPHERE).
        :param dimensions: A list of floats representing the dimensions of the shape (e.g., [x, y, z] for a box).
        :param pose: A geometry_msgs.msg.Pose object for the object's position and orientation.
        :param frame_id: The coordinate frame the pose is defined in (default: "world").
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

        self.planning_scene_publisher.publish(collision_object)
        self.get_logger().info(f"Added collision object: {object_id} in frame {frame_id}")
        # Add a small delay to allow MoveIt's Planning Scene Monitor to process the update
        time.sleep(0.1) # Adjust if needed


    def remove_collision_object(self, object_id):
        """
         Removes a collision object from the planning scene.

        :param object_id: The string ID of the object to remove.
        """
        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"  # Frame ID doesn't strictly matter for removal by ID, but often matches add frame
        collision_object.id = object_id
        collision_object.operation = CollisionObject.REMOVE

        self.planning_scene_publisher.publish(collision_object)
        self.get_logger().info(f"Removed collision object: {object_id}")
        # Add a small delay to allow MoveIt's Planning Scene Monitor to process the update
        time.sleep(0.1) # Adjust if needed

    def setup_planning_scene(self, obstacles, frame_id="world"):
        """
        Sets up the planning scene by adding predefined obstacles.

        :param obstacles: A list of dictionaries defining the obstacles.
        :param frame_id: The coordinate frame to define obstacles in (default: "world").
        """
        self.get_logger().info(f"Setting up planning scene with obstacles in {frame_id} frame...")
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
                self.add_collision_object(obstacle["name"], SolidPrimitive.BOX, obstacle_dimensions, obstacle_pose, frame_id)

        # Give MoveIt's Planning Scene Monitor time to process the added collision objects
        time.sleep(1.0) # Adjust this delay if necessary
        self.get_logger().info("Planning scene setup complete.")


    def plan_and_execute_pose_goal(self, goal_pose: Pose, goal_frame_id="base"):
        """
        Plans and executes a trajectory to the specified pose goal (quaternion orientation).

        :param goal_pose: The target pose (geometry_msgs.msg.Pose).
        :param goal_frame_id: The coordinate frame the goal pose is defined in (default: "base_link").
        """
        self.get_logger().info(f"Planning and executing trajectory to pose goal in {goal_frame_id} frame...")

        goal_msg = MoveGroup.Goal()
        request = MotionPlanRequest()

        # Set planning group
        request.group_name = "ur_manipulator" # Assuming "ur_manipulator" is your planning group name

        # Set the start state to the robot's current state read from RTDE
        if self.rtde_receive and self.rtde_receive.isConnected():
            # Add a small delay to ensure RTDE receive interface is ready
            time.sleep(0.1) # Adjust as needed
            current_joint_state_list = self.rtde_receive.getActualQ()
            self.get_logger().info(f"RTDE getActualQ() returned: {current_joint_state_list}") # Debug log
            self.get_logger().info(f"Length of getActualQ() result: {len(current_joint_state_list) if current_joint_state_list is not None else 'None'}") # Debug log


            if current_joint_state_list is not None and len(current_joint_state_list) == 6: # Assuming 6 joints for UR5
                robot_state = RobotState()
                # Based on your /joint_states output, the order is:
                # ['shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'shoulder_pan_joint']
                # Use this order for setting the start state, assuming RTDE getActualQ() matches or MoveIt can handle this.
                # A more robust way is to use a dictionary mapping if the order is truly inconsistent.
                # For now, sticking to the observed /joint_states order for RTDE positions.
                rtde_joint_names_order = [
                    'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
                    'wrist_2_joint', 'wrist_3_joint', 'shoulder_pan_joint'
                ]

                robot_state.joint_state.name = rtde_joint_names_order
                robot_state.joint_state.position = list(current_joint_state_list) # Assign positions based on this assumed order
                request.start_state = robot_state
                self.get_logger().info(f"Set planning start state to current robot joint state: {current_joint_state_list} with assumed order {rtde_joint_names_order}")
            else:
                 self.get_logger().error("Could not get valid current joint state from RTDE. Planning may fail or use default start state.")
                 if current_joint_state_list is not None:
                     self.get_logger().error(f"Received joint state (list): {current_joint_state_list}")
                 else:
                     self.get_logger().error("RTDE getActualQ() returned None.")
                 # Optionally, you could return here if getting start state from RTDE is critical
                 # return


        else:
             self.get_logger().error("RTDE receive interface not initialized or connected. Cannot get current robot state for planning.")
             # If RTDE is not connected, MoveIt will likely use a default start state.
             # Depending on your setup, this might be acceptable for planning,
             # but it won't reflect the actual robot's position.


        # Set goal constraints
        goal_constraints = Constraints()

        self.get_logger().info(f"Setting pose goal constraints in {goal_frame_id} frame.")
        # Assuming the pose is for the end-effector link (tool0 or similar)
        end_effector_link = "tool0" # Replace with your robot's end-effector link name

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = goal_frame_id # Set frame_id based on input
        position_constraint.link_name = end_effector_link
        position_constraint.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01])) # A small sphere around the target point
        position_constraint.constraint_region.primitive_poses.append(goal_pose)
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = goal_frame_id # Set frame_id based on input
        orientation_constraint.link_name = end_effector_link
        orientation_constraint.orientation = goal_pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.01 # Adjust tolerance as needed
        orientation_constraint.absolute_y_axis_tolerance = 0.01 # Adjust tolerance as needed
        orientation_constraint.absolute_z_axis_tolerance = 0.01 # Adjust tolerance as needed
        orientation_constraint.weight = 1.0

        goal_constraints.position_constraints.append(position_constraint)
        goal_constraints.orientation_constraints.append(orientation_constraint)


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


    def plan_and_execute_tcp_pose_goal(self, tcp_pose: list, goal_frame_id="base"):
        """
        Plans and executes a trajectory to the specified TCP pose goal (position and axis-angle orientation).

        :param tcp_pose: A list [x, y, z, rx, ry, rz] representing the target TCP pose.
                         Position in meters, orientation in axis-angle (radians).
        :param goal_frame_id: The coordinate frame the goal pose is defined in (default: "base_link").
        """
        if len(tcp_pose) != 6:
            self.get_logger().error("tcp_pose must be a list of 6 values: [x, y, z, rx, ry, rz]")
            return

        x, y, z, rx, ry, rz = tcp_pose

        # Convert axis-angle [rx, ry, rz] to quaternion [x, y, z, w] using scipy
        try:
            # Create a Rotation object from the axis-angle vector
            rotation = R.from_rotvec([rx, ry, rz])
            # Get the quaternion in [x, y, z, w] format
            quaternion = rotation.as_quat()
        except Exception as e:
            self.get_logger().error(f"Failed to convert axis-angle to quaternion using scipy: {e}")
            return

        # Create the geometry_msgs/Pose message
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


        # Call the existing plan_and_execute_pose_goal with the quaternion pose
        self.plan_and_execute_pose_goal(goal_pose_quat, goal_frame_id)


    def plan_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Planning goal rejected.')
            return

        self.get_logger().info('Planning goal accepted. Waiting for result...')
        self._get_plan_result_future = goal_handle.get_result_async()
        # Wait for the result and process it directly
        rclpy.spin_until_future_complete(self, self._get_plan_result_future)
        self.plan_result_callback(self._get_plan_result_future)


    def plan_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == result.error_code.SUCCESS:
            self.get_logger().info("Planning successful.")
            planned_trajectory = result.planned_trajectory.joint_trajectory
            if planned_trajectory.points:
                 self.get_logger().info(f"Planned trajectory has {len(planned_trajectory.points)} points.")
                 # Send the planned trajectory to the ExecuteTrajectory action server
                 self.execute_trajectory(planned_trajectory)
            else:
                 self.get_logger().warn("Planned trajectory has no points.")

        else:
            self.get_logger().error(f"Planning failed with error code: {result.error_code.val}")
            # You might want to print more details about the error if available
            # self.get_logger().error(f"Planning error: {result.error_code.text}")

    def execute_trajectory(self, trajectory: JointTrajectory):
        """
        Sends the planned trajectory to the ExecuteTrajectory action server for execution.
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
            self._execute_goal_handle = None # Clear goal handle if rejected
            return

        self.get_logger().info('Execution goal accepted. Waiting for result...')
        self._execute_goal_handle = goal_handle # Store the goal handle
        self._get_execute_result_future = goal_handle.get_result_async()
        # Wait for the result and process it directly
        rclpy.spin_until_future_complete(self, self._get_execute_result_future)
        self.execute_result_callback(self._get_execute_result_future)


    def execute_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == result.error_code.SUCCESS:
            self.get_logger().info("Trajectory execution successful.")
        else:
            self.get_logger().error(f"Trajectory execution failed with error code: {result.error_code.val}")
            # You might want to print more details about the error if available
            # self.get_logger().error(f"Execution error: {result.error_code.text}")
        self._execute_goal_handle = None # Clear goal handle after execution completes


    def cancel_execution_goal(self):
        """
        Sends a cancel request to the active ExecuteTrajectory action goal.
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
        self._execute_goal_handle = None # Clear goal handle after cancellation attempt


# Global node instance for signal handler
node_instance = None

# Signal handler for graceful shutdown
def signal_handler(sig, frame):
    global node_instance
    if node_instance is not None:
        node_instance.get_logger().info("Ctrl+C received. Shutting down node and cancelling active goals...")
        node_instance.cancel_execution_goal() # Attempt to cancel active MoveIt goal
    rclpy.shutdown()
    sys.exit(0)


def main(args=None):
    rclpy.init(args=args)

    global node_instance
    node_instance = URMoveitRTDENode()

    # Register signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)


    # Define obstacles
    obstacles = [
        {
            "type": "box",
            "name": "pb_box_1",
            "pose": [-0.205, 0.011, -0.011, 0, 0, 0, 1], # Corrected signs
            "size": [0.65, 0.65, 0.01]
        },
        {
            "type": "box",
            "name": "pb_box_2",
            "pose": [-0.7, 0.30, 0.4, 0, 0, 0, 1], # Corrected signs
            "size": [0.045, 0.045, 0.81]
        },
        {
            "type": "box",
            "name": "pb_box_3",
            "pose": [-0.575, 0.30, 0.82, 0, 0, 0, 1], # Corrected signs
            "size": [0.26, 0.045, 0.045]
        },
        {
            "type": "box",
            "name": "pb_box_4",
            "pose": [-0.575, 0.24, 0.8325, 0, 0, 0, 1], # Corrected signs
            "size": [0.095, 0.021, 0.027]
        },
        {
            "type": "box",
            "name": "pb_box_5",
            "pose": [0.135, 0.25, 0.085, 0, 0, 0, 1], # Corrected signs
            "size": [0.015, 0.085, 0.20]
        },
        {
            "type": "box",
            "name": "pb_box_6",
            "pose": [-0.37, 0.02, 0.115, 0, 0, 0, 1], # Corrected signs
            "size": [0.025, 0.025, 0.185]
        },
        {
            "type": "box",
            "name": "pb_box_7",
            "pose": [-0.37, -0.295, 0.115, 0, 0, 0, 1], # Corrected signs
            "size": [0.025, 0.025, 0.185]
        }
    ]

    # Example obstacles defined in "base_link" frame
    """
    obstacles_tool0 = [
        {
            "type": "box",
            "name": "base_link_box_1",
            "pose": [0.5, 0.5, 0.0, 0, 0, 0, 1], # Example pose in base_link
            "size": [0.2, 0.075, 0.04]
        },
    ]
    """


    # Add obstacles to the planning scene
    # Use "world" frame for obstacles if they are fixed in the environment
    # Use "base_link" frame if they move with the robot's base or are defined relative to it
    node_instance.setup_planning_scene(obstacles, frame_id="base")
    # If you have obstacles defined relative to base_link:
    # node_instance.setup_planning_scene(obstacles_base_link, frame_id="base_link")


    # Give MoveIt's Planning Scene Monitor time to process the added collision objects
    time.sleep(1.0) # Adjust this delay if necessary

    # --- Planning and Execution Request ---

    # Example of using the new method with TCP pose in [x, y, z, rx, ry, rz] format
    # Replace with your desired TCP pose values from Polyscope
    target_tcp_pose = [-0.2, 0.4, 0.4, -1.57, 3.14, 0.0] # Example values

    # Plan and execute the trajectory using the TCP pose input
    node_instance.plan_and_execute_tcp_pose_goal(target_tcp_pose, goal_frame_id="base")


    # Spin the node with a loop that checks rclpy.ok()
    try:
        while rclpy.ok():
            rclpy.spin_once(node_instance, timeout_sec=0.1) # Process callbacks periodically
            # Add any other logic you need to run in the main loop here
    except KeyboardInterrupt:
        pass # Ctrl+C will be handled by the signal handler


    # Optional: Remove obstacles when done
    #node_instance.remove_collision_object("base_link_box_1")
    # node_instance.remove_collision_object("pb_box_2")
    # node_instance.remove_collision_object("pb_box_3")
    # node_instance.remove_collision_object("pb_box_4")


    # Clean up RTDE receive connection
    if node_instance.rtde_receive and node_instance.rtde_receive.isConnected():
        node_instance.rtde_receive.disconnect()

    # Clean up RTDE control connection if it was used
    if node_instance.rtde_control and node_instance.rtde_control.isConnected():
        node_instance.rtde_control.servoStop() # Ensure servo mode is stopped if it was used
        node_instance.rtde_control.disconnect()


    node_instance.destroy_node()
    # rclpy.shutdown() # rclpy.shutdown is now called in the signal handler

if __name__ == '__main__':
    main()
