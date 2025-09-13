import rclpy
from rclpy.node import Node
import moveit_commander
import sys
import time
import rtde_io
import rtde_receive
import rtde_control

class URMoveitRTDENode(Node):
    def __init__(self):
        super().__init__('ur_moveit_rtde_node')
        self.get_logger().info("Starting URMoveitRTDENode")

        # Initialize MoveIt 2
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("ur_manipulator") # Assuming "ur_manipulator" is your planning group name from ur_move_config

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
            rclpy.shutdown()
            return

    def plan_and_execute(self):
        self.get_logger().info("Planning trajectory...")

        # Example: Plan to a joint goal
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = 0.0  # Modify joint values as needed
        joint_goal[1] = -1.57
        joint_goal[2] = 1.57
        joint_goal[3] = -1.57
        joint_goal[4] = -1.57
        joint_goal[5] = 0.0

        self.move_group.set_joint_value_target(joint_goal)

        plan = self.move_group.plan()

        if plan[0]:
            self.get_logger().info("Planning successful. Executing trajectory via RTDE servoj...")
            self.execute_trajectory_rtde(plan[1])
        else:
            self.get_logger().error("Planning failed!")

    def execute_trajectory_rtde(self, trajectory):
        if not self.rtde_control:
            self.get_logger().error("RTDE control interface not initialized.")
            return

        # Convert MoveIt trajectory to RTDE servoj format
        # This is a simplified conversion. You might need to adjust based on your needs (e.g., time synchronization)
        waypoints = []
        for point in trajectory.joint_trajectory.points:
            # Assuming joint_trajectory.points[i].positions contains the joint angles
            # and joint_trajectory.points[i].time_from_start contains the time from the start of the trajectory
            waypoints.append(list(point.positions))

        self.get_logger().info(f"Executing {len(waypoints)} waypoints...")

        # Send waypoints using servoj
        # Note: servoj requires a time parameter for each point.
        # This example uses a fixed time interval. You might need to calculate this based on the trajectory time.
        time_per_waypoint = 0.1  # Adjust as needed

        for i, waypoint in enumerate(waypoints):
            # Add velocities, accelerations, and time to the waypoint data for servoj
            # For simplicity, using default values. You might need to get these from the trajectory.
            velocity = 0.5
            acceleration = 0.5
            dt = time_per_waypoint
            blend = 0.0

            # servoj takes joint angles in radians
            servoj_command = waypoint + [velocity, acceleration, dt, blend]

            try:
                self.rtde_control.servoj(servoj_command)
                self.get_logger().info(f"Sent waypoint {i+1}/{len(waypoints)}")
                time.sleep(dt) # Wait for the robot to reach the waypoint (simplified)
            except Exception as e:
                self.get_logger().error(f"Error sending servoj command for waypoint {i+1}: {e}")
                break

        self.get_logger().info("Trajectory execution finished.")


def main(args=None):
    rclpy.init(args=args)
    node = URMoveitRTDENode()
    node.plan_and_execute()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
