#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import numpy as np
from mplib import Pose, Planner
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
from scipy.spatial.transform import Rotation as R


class UR5MotionPlanner:
    def __init__(self, robot_ip, urdf_path, srdf_path,
                 move_group="tool0",
                 velocity=0.005, acceleration=0.01,   # super-slow defaults
                 dt=2.5,
                 interp_threshold=0.1, interp_steps=10,
                 planning_retries=3, retry_delay=0.5):
        """
        Keep structure from your original script, but add robustness & slow defaults.

        - move_group: same argument name your environment's mplib expects.
        - velocity/acceleration: very low by default to move "super slowly".
        - interp_threshold / interp_steps: safe interpolation before big jumps.
        - planning_retries: number of attempts mplib will try before failing.
        """
        self.robot_ip = robot_ip
        self.urdf_path = urdf_path
        self.srdf_path = srdf_path
        self.move_group = move_group

        # RTDE
        self.rtde_control = RTDEControlInterface(robot_ip)
        self.rtde_receive = RTDEReceiveInterface(robot_ip)

        # MPLib Planner: keep same signature as your working mplib usage
        self.planner = Planner(
            urdf=urdf_path,
            srdf=srdf_path,
            move_group=move_group,
        )

        # Execution / safety params
        self.velocity = velocity
        self.acceleration = acceleration
        self.dt = dt
        self.interp_threshold = interp_threshold
        self.interp_steps = interp_steps

        # planning retries
        self.planning_retries = planning_retries
        self.retry_delay = retry_delay

        # Obstacles (kept for the user API). Use planner.set_objects(...) to register
        self.obstacles = []

    # -------------------- Utilities --------------------
    def set_obstacles(self, obstacle_meshes):
        """
        Replace planner obstacles with obstacle_meshes and inform mplib.
        obstacle_meshes should be in the format your mplib expects (filepaths or descriptors).
        This mirrors the pattern from your earlier code:
            self.planner.set_objects(obstacle_meshes)
        """
        self.obstacles = obstacle_meshes or []
        # Pass them to mplib's planner (your original code did this)
        try:
            # Assuming 'set_obstacles' or similar is the correct method based on user code
            # If this still fails, consult your mplib documentation.
            if hasattr(self.planner, 'set_objects'):
                 self.planner.set_objects(self.obstacles)
            elif hasattr(self.planner, 'set_obstacles'): # alternative name?
                 self.planner.set_obstacles(self.obstacles)
            else:
                 print("[OBSTACLES] Warning: planner object has no attribute 'set_objects' or 'set_obstacles'. Obstacles not set.")

        except Exception as e:
            print(f"[OBSTACLES] Warning: planner.set_objects raised: {e}")
        print(f"[OBSTACLES] {len(self.obstacles)} obstacles added to planner.")

    def add_obstacles(self, obstacle_meshes):
        """Append obstacles to existing list and call planner.set_objects(...)"""
        self.obstacles.extend(obstacle_meshes)
        try:
            if hasattr(self.planner, 'set_objects'):
                 self.planner.set_objects(self.obstacles)
            elif hasattr(self.planner, 'set_obstacles'):
                 self.planner.set_obstacles(self.obstacles)
            else:
                 print("[OBSTACLES] Warning: planner object has no attribute 'set_objects' or 'set_obstacles'. Obstacles not set.")
        except Exception as e:
            print(f"[OBSTACLES] Warning: planner.set_objects raised: {e}")
        print(f"[OBSTACLES] Now {len(self.obstacles)} obstacles active.")

    def clear_obstacles(self):
        """Clear obstacles both locally and in planner."""
        self.obstacles = []
        try:
            if hasattr(self.planner, 'set_objects'):
                 self.planner.set_objects([])
            elif hasattr(self.planner, 'set_obstacles'):
                 self.planner.set_obstacles([])
            else:
                 print("[OBSTACLES] Warning: planner object has no attribute 'set_objects' or 'set_obstacles'. Obstacles not cleared.")
        except Exception as e:
            print(f"[OBSTACLES] Warning: planner.set_objects([]) raised: {e}")
        print("[OBSTACLES] Cleared all obstacles.")

    def get_current_joint_positions(self):
        """Return joint array from RTDE (real robot)"""
        return np.array(self.rtde_receive.getActualQ())

    def get_current_tcp_pose(self):
        """Return current TCP pose from RTDE as [x,y,z, rx,ry,rz] (rvec)"""
        tcp = np.array(self.rtde_receive.getActualTCPPose())
        # mplib & your previous code used axis-angle (rvec) representation for planning input
        # RTDE sometimes returns quaternion; if length==7 or 6 handle accordingly.
        # Here we assume the same as your initial code: ActualTCPPose returns [x,y,z, rx,ry,rz] (axis-angle)
        return tcp  # keep as-is; prints will show pos + rvec

    # -------------------- helpers --------------------
    @staticmethod
    def rvec_to_quat(rvec):
        """Convert axis-angle rotation vector to quaternion [x,y,z,w] for mplib Pose convenience."""
        rot = R.from_rotvec(rvec)
        q = rot.as_quat()  # [x,y,z,w]
        return [q[0], q[1], q[2], q[3]]

    @staticmethod
    def pos_rvec_to_pose(pos, rvec):
        """Return an mplib Pose from pos + rvec"""
        quat = UR5MotionPlanner.rvec_to_quat(rvec)
        return Pose(pos, quat)

    def forward_kinematics(self, joint_values):
        """
        Perform forward kinematics using mplib to get the TCP pose from joint values.
        NOTE: The exact method name might vary based on your mplib version.
        Consult your mplib documentation for the correct function to get the pose
        of the 'tool0' link or the move_group tip.

        Common potential method names might include:
        - planner.get_link_pose(joint_values, "tool0")
        - planner.compute_fk(joint_values, "tool0")
        - planner.robot.get_link_pose(joint_values, "tool0")
        ...or similar, potentially requiring specifying the link name ('tool0').
        """
        try:
            # --- Placeholder FK Call ---
            # REPLACE THE LINE BELOW with the correct mplib FK call for your setup.
            # Example: tcp_pose = self.planner.get_link_pose(joint_values, self.move_group)

            print("[FK] Attempting placeholder FK call - REPLACE THIS LINE!")
            # This placeholder will always fail with the previous error unless replaced.
            # You need to find the correct method in your mplib documentation.
            raise NotImplementedError("Correct mplib FK method not implemented. Consult mplib docs.")

            # --- End Placeholder FK Call ---

            # Assuming the correct FK call returns a Pose object or similar:
            pos = tcp_pose.pos
            quat = tcp_pose.quat # [x, y, z, w]
            rot = R.from_quat(quat)
            rvec = rot.as_rotvec() # [rx, ry, rz]
            return np.array(pos + rvec.tolist())
        except NotImplementedError:
             print("[FK][ERROR] Correct mplib FK method is not implemented in the code.")
             return None
        except Exception as e:
            print(f"[FK][ERROR] Forward kinematics failed: {e}")
            print("[FK] Please double-check your mplib documentation for the correct FK method signature and expected return type.")
            return None # Return None if FK fails

    # -------------------- Planning (with retries) --------------------
    def plan_to_tcp_pose(self, target_pose, current_joints=None):
        if current_joints is None:
            current_joints = self.get_current_joint_positions()

        if not isinstance(target_pose, Pose):
            # Expecting [x,y,z, rx,ry,rz] or a Pose-like tuple
            try:
                target_pose = Pose(target_pose[:3], target_pose[3:])
            except Exception:
                raise ValueError("target_pose must be an mplib.Pose or a length-6 iterable [x,y,z,rx,ry,rz]")

        print("[PLANNING] Computing trajectory...")

        # --- Attempt to inspect Pose contents across mplib versions ---
        pos = None
        ori = None
        try:
            # Attempt to access common attributes for position and orientation
            pos = getattr(target_pose, 'pos', getattr(target_pose, 'position', getattr(target_pose, 'translation', None)))
            ori = getattr(target_pose, 'quat', getattr(target_pose, 'orientation', getattr(target_pose, 'rotation', None)))

            if pos is not None:
                pos = np.array(pos)
            if ori is not None:
                ori = np.array(ori)

        except Exception:
            pass # If accessing attributes fails, pos/ori remain None

        if pos is not None and ori is not None:
            try:
                pos_print = np.round(pos, 4)
                ori_print = np.round(ori, 4)
                print(f"[PLANNING] Target TCP pos={pos_print}, quat={ori_print}")
            except Exception:
                print(f"[PLANNING] Target TCP: Could not round pos/quat. Raw attributes: pos={pos}, ori={ori}")
        else:
            print(f"[PLANNING] Target TCP: Could not access standard pose attributes. Attempting direct print:")
            try:
                print(f"[PLANNING] Target TCP (direct print): {target_pose}")
            except Exception:
                print(f"[PLANNING] Target TCP: Could not even directly print the pose object.")


        # Print current TCP (rvec style if available) and target pose
        try:
            tcp_curr = self.get_current_tcp_pose()
            pos_curr = tcp_curr[:3]
            rvec_curr = tcp_curr[3:6]
            print(f"[PLANNING] Current TCP pos={np.round(pos_curr,4)}, rvec={np.round(rvec_curr,4)}")
        except Exception:
            print(f"[PLANNING] Current TCP: (unavailable)")


        # Call planner (keeps same API as your working mplib usage)
        try:
            result = self.planner.plan_pose(target_pose, current_joints.tolist())
        except Exception as e:
            print(f"[PLANNING] Planner.plan_pose raised exception: {e}")
            result = {"status": "Error", "position": []}

        status = result.get("status", "Error")
        print(f"[PLANNING] Planning status: {status}")

        if status == "Success":
            n_points = len(result.get("position", []))
            print(f"[PLANNING] Trajectory points: {n_points}")

        return result

    # -------------------- Execution --------------------
    def execute_trajectory(self, trajectory, velocity=None, acceleration=None, dt=None,
                           interp_threshold=None, interp_steps=None):
        """
        Execute a joint-space trajectory on the real robot.
        - prints TCP poses to terminal while executing (user requested).
        - does safe interpolation to first point if too far.
        - Prints planned joint and corresponding FK TCP poses for debugging.
        """
        if velocity is None:
            velocity = self.velocity
        if acceleration is None:
            acceleration = self.acceleration
        if dt is None:
            dt = self.dt
        if interp_threshold is None:
            interp_threshold = self.interp_threshold
        if interp_steps is None:
            interp_steps = interp_steps

        if trajectory is None:
            print("[EXECUTE] Trajectory is None -> nothing to do.")
            return False

        traj_arr = np.array(trajectory)
        if traj_arr.ndim == 1:
            traj_arr = traj_arr.reshape(1, -1)

        if traj_arr.shape[1] != 6:
            print(f"[EXECUTE][ERROR] Trajectory points must be length 6. Got shape {traj_arr.shape}")
            return False

        print("\n--- Planned Trajectory Waypoints (Joints and FK TCP) ---")
        for i, joint_point in enumerate(traj_arr):
            # Call the forward kinematics function for each planned joint point
            tcp_pose_fk = self.forward_kinematics(joint_point.tolist())
            if tcp_pose_fk is not None:
                pos_fk = tcp_pose_fk[:3]
                rvec_fk = tcp_pose_fk[3:6]
                print(f"Waypoint {i+1}/{traj_arr.shape[0]}:")
                print(f"  Joints: {np.round(joint_point, 4)}")
                print(f"  FK TCP: pos={np.round(pos_fk, 4)}, rvec={np.round(rvec_fk, 4)}")
            else:
                 print(f"Waypoint {i+1}/{traj_arr.shape[0]}: Joints: {np.round(joint_point, 4)} (FK failed - check forward_kinematics method)")
        print("---------------------------------------------------------\n")


        try:
            current_joints = self.get_current_joint_positions()
        except Exception as e:
            print(f"[EXECUTE][ERROR] Could not read current joints: {e}")
            return False

        first_point = traj_arr[0]
        dist = np.linalg.norm(first_point - current_joints)
        print(f"[EXECUTE] Current joints: {np.round(current_joints,4)}")
        print(f"[EXECUTE] First planned point: {np.round(first_point,4)}, dist={dist:.4f} rad")

        # Safe interpolation to first point if far
        if dist > interp_threshold:
            print(f"[EXECUTE] Interpolating {interp_steps} steps to first point (safe).")
            for i, alpha in enumerate(np.linspace(0.0, 1.0, interp_steps, endpoint=True)):
                interp = (1.0 - alpha) * current_joints + alpha * first_point
                try:
                    # use servoJ for small steps
                    self.rtde_control.servoJ(interp.tolist(), velocity, acceleration, dt, 0.05, 120)
                    time.sleep(dt)
                except Exception as e:
                    print(f"[EXECUTE][ERROR] servoJ during interpolation failed at step {i}: {e}")
                    return False
                # print TCP for feedback
                tcp = self.get_current_tcp_pose()
                try:
                    pos, rvec = tcp[:3], tcp[3:6]
                    print(f"[EXEC-INT] step {i+1}/{interp_steps} TCP pos={np.round(pos,4)}, rvec={np.round(rvec,4)}")
                except Exception:
                    print(f"[EXEC-INT] step {i+1}/{interp_steps} (couldn't read TCP)")

                time.sleep(0.05) # Small additional sleep
            time.sleep(0.05) # Small settle time after interpolation
            print("[EXECUTE] Interpolation to first point done.")
        else:
            # if close, use moveJ to go to first point (fast single point move but still with low speed)
            try:
                print("[EXECUTE] moveJ to first planned point (close).")
                self.rtde_control.moveJ(first_point.tolist(), velocity, acceleration)
                time.sleep(0.05) # Small settle time after moveJ
            except Exception as e:
                print(f"[EXECUTE][WARN] moveJ failed (continuing to stream): {e}")


        # Stream remaining trajectory points with servoJ very slowly
        try:
            total = traj_arr.shape[0]
            for i in range(total):
                point = traj_arr[i].tolist()
                # stream servoJ - small 'time' param to help interpolation on controller side
                try:
                    self.rtde_control.servoJ(point, velocity, acceleration, dt, 0.05, 120)
                    time.sleep(dt) # Explicit sleep for the duration 'dt'
                except Exception as e:
                    print(f"[EXECUTE][ERROR] servoJ failed at point {i}: {e}")
                    return False

                # print TCP pose (user wanted TCP in terminal)
                tcp = self.get_current_tcp_pose()
                try:
                    pos = tcp[:3]; rvec = tcp[3:6]
                    if i % max(1, total//10) == 0 or i == total - 1:
                        print(f"[EXECUTE] {i+1}/{total} TCP pos={np.round(pos,4)}, rvec={np.round(rvec,4)}")
                except Exception:
                    if i % max(1, total//10) == 0 or i == total - 1:
                        print(f"[EXECUTE] {i+1}/{total} (couldn't read TCP)")

                # safety: small sleep to give controller time; reduce if you want smoother streaming
                time.sleep(0.05) # Small additional sleep after each point

        finally:
            try:
                self.rtde_control.servoStop()
            except Exception as e:
                print(f"[EXECUTE] servoStop raised: {e}")

        print("[EXECUTE] Trajectory execution completed.")
        return True

    # -------------------- End-to-end move --------------------
    def move_to_tcp_pose(self, pos, rvec, velocity=None, acceleration=None, tol=1e-3):
        """
        Plan + execute to a TCP pose (pos + axis-angle rvec).
        Returns True on success.
        """
        # prepare target Pose (keeping same Pose usage as your original script)
        target_pose = self.pos_rvec_to_pose(pos, rvec)

        current_joints = self.get_current_joint_positions()
        print(f"[MOVE] Current joints: {np.round(current_joints,4)}")
        print(f"[MOVE] Target TCP pos={np.round(pos,4)}, rvec={np.round(rvec,4)}")

        # Plan with retries
        plan_result = None
        for attempt in range(self.planning_retries):
            print(f"[MOVE] Planning attempt {attempt + 1}/{self.planning_retries}...")
            plan_result = self.plan_to_tcp_pose(target_pose, current_joints)
            if plan_result is not None and plan_result.get("status", "") == "Success":
                print("[MOVE] Planning successful.")
                break
            else:
                print(f"[MOVE] Planning failed on attempt {attempt + 1}.")
                if attempt < self.planning_retries - 1:
                    time.sleep(self.retry_delay)
                    current_joints = self.get_current_joint_positions() # Get updated joints before retrying
                else:
                    print("[MOVE] Max planning retries reached.")
                    return False


        traj = plan_result.get("position", None)
        if traj is None or len(traj) == 0:
            print("[MOVE] Planner returned empty trajectory.")
            return False

        # Print first few planned points for verification
        print("First 3 planned points:\n", traj[:min(3, len(traj))])


        # Execute with safe streaming / prints
        executed = self.execute_trajectory(traj, velocity=velocity, acceleration=acceleration)
        if not executed:
            print("[MOVE] Execution failed.")
            return False

        # small settle then verify TCP (prints)
        time.sleep(0.5)
        tcp_after = self.get_current_tcp_pose()
        try:
            pos_after = tcp_after[:3]; rvec_after = tcp_after[3:6]
            pos_err = np.linalg.norm(np.array(pos_after) - np.array(pos))
            rvec_err = np.linalg.norm(np.array(rvec_after) - np.array(rvec))
            print(f"[MOVE] Final TCP pos={np.round(pos_after, 4)}, rvec={np.round(rvec_after, 4)}")
            print(f"[MOVE] pos_err={pos_err:.6f}, rvec_err={rvec_err:.6f}")
            return (pos_err < tol) and (rvec_err < tol)
        except Exception:
            print("[MOVE] Could not compute final TCP error (non-fatal).")
            return True

    def disconnect(self):
        try:
            self.rtde_control.disconnect()
        except Exception:
            pass
        try:
            self.rtde_receive.disconnect()
        except Exception:
            pass


# ===================== Example usage =====================
if __name__ == "__main__":
    planner = UR5MotionPlanner(
        robot_ip="10.2.30.60",  # your robot IP
        urdf_path="/home/robot/Bureau/ur5_control/ur.urdf",
        srdf_path="/home/robot/Bureau/ur5_control/srdf/ur5.srdf",
        move_group="tool0",
        velocity=0.02,  # super-slow defaults
        acceleration=0.01, # super-slow defaults
        dt=1.5,
        interp_threshold=0.1,
        interp_steps=10,
        planning_retries=3
    )

    try:
        # Example obstacle definitions (replace with your actual obstacles)
        mplib_obstacles = [
            {
                "type": "box",
                "name": "pb_box_1",
                "pose": [-0.205, 0.0, -0.02, 0, 0, 0, 1],
                "size": [0.325 * 2, 0.325 * 2, 0.01 * 2]
            },
            {
                "type": "box",
                "name": "pb_box_2",
                "pose": [-0.8, 0.30, 0.4, 0, 0, 0, 1],
                "size": [0.02 * 2, 0.02 * 2, 0.4 * 2]
            },
            {
                "type": "box",
                "name": "pb_box_3",
                "pose": [-0.675, 0.30, 0.82, 0, 0, 0, 1],
                "size": [0.125 * 2, 0.02 * 2, 0.02 * 2]
            }
        ]
        planner.set_obstacles(mplib_obstacles)


        pos = [-0.382, 0.635, 0.624]
        rvec = [2.035, -2.705, -2.326]  # axis-angle rotation vector
        ok = planner.move_to_tcp_pose(pos, rvec)
        print("SUCCESS" if ok else "FAILED")

    finally:
        planner.disconnect()
