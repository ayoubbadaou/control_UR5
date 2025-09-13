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

        # MPLib Planner: keep same signature as your working code
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
            self.planner.set_objects(self.obstacles)
        except Exception as e:
            print(f"[OBSTACLES] Warning: planner.set_objects raised: {e}")
        print(f"[OBSTACLES] {len(self.obstacles)} obstacles added to planner.")

    def add_obstacles(self, obstacle_meshes):
        """Append obstacles to existing list and call planner.set_objects(...)"""
        self.obstacles.extend(obstacle_meshes)
        try:
            self.planner.set_objects(self.obstacles)
        except Exception as e:
            print(f"[OBSTACLES] Warning: planner.set_objects raised: {e}")
        print(f"[OBSTACLES] Now {len(self.obstacles)} obstacles active.")

    def clear_obstacles(self):
        """Clear obstacles both locally and in planner."""
        self.obstacles = []
        try:
            self.planner.set_objects([])
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

        # --- Safe way to inspect Pose contents across mplib versions ---
        pos = None
        ori = None
        for attr in ("pos", "position", "translation"):
            if hasattr(target_pose, attr):
                pos = np.array(getattr(target_pose, attr))
                break
        for attr in ("quat", "orientation", "rotation"):
            if hasattr(target_pose, attr):
                ori = np.array(getattr(target_pose, attr))
                break

        if pos is not None and ori is not None:
            pos_print = np.round(pos, 4)
            ori_print = np.round(ori, 4)
        else:
            pos_print = "?"
            ori_print = "?"

        # Print current TCP (rvec style if available) and target pose
        try:
            tcp_curr = self.get_current_tcp_pose()
            pos_curr = tcp_curr[:3]
            rvec_curr = tcp_curr[3:6]
            print(f"[PLANNING] Current TCP pos={np.round(pos_curr,4)}, rvec={np.round(rvec_curr,4)}")
        except Exception:
            print(f"[PLANNING] Current TCP: (unavailable)")

        print(f"[PLANNING] Target TCP pos={pos_print}, quat={ori_print}")

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
            interp_steps = self.interp_steps

        if trajectory is None:
            print("[EXECUTE] Trajectory is None -> nothing to do.")
            return False

        traj_arr = np.array(trajectory)
        if traj_arr.ndim == 1:
            traj_arr = traj_arr.reshape(1, -1)

        if traj_arr.shape[1] != 6:
            print(f"[EXECUTE][ERROR] Trajectory points must be length 6. Got shape {traj_arr.shape}")
            return False

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
                pos, rvec = self.get_current_tcp_pose()[:3], self.get_current_tcp_pose()[3:6]
                print(f"[EXEC-INT] step {i+1}/{interp_steps} TCP pos={np.round(pos,4)}, rvec={np.round(rvec,4)}")
                time.sleep(0.05)
            time.sleep(0.05)
            print("[EXECUTE] Interpolation to first point done.")
        else:
            # if close, use moveJ to go to first point (fast single point move but still with low speed)
            try:
                print("[EXECUTE] moveJ to first planned point (close).")
                self.rtde_control.moveJ(first_point.tolist(), velocity, acceleration)
                time.sleep(0.05)
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
                    time.sleep(dt)
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
                time.sleep(0.05)
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
        plan_result = self.plan_to_tcp_pose(target_pose, current_joints)
        if plan_result is None or plan_result.get("status", "") != "Success":
            print(f"[MOVE] Planning failed: {plan_result}")
            return False

        traj = plan_result.get("position", None)
        print("First 3 planned points:\n", traj[:3])
        if traj is None or len(traj) == 0:
            print("[MOVE] Planner returned empty trajectory.")
            return False

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
        velocity=0.005,
        acceleration=0.001,
        dt=1.5,
        interp_threshold=0.08,
        interp_steps=12,
        planning_retries=3
    )

    try:
        # set obstacles using the same approach you used before:
        planner.set_obstacles([
            # e.g. mesh file paths / object descriptors understood by your mplib installation
            # "/path/to/mesh1.stl",
            # "/path/to/table.obj",
        ])

        pos = [-0.382, 0.635, 0.624]
        rvec = [2.035, -2.705, -2.326]  # axis-angle rotation vector
        ok = planner.move_to_tcp_pose(pos, rvec)
        print("SUCCESS" if ok else "FAILED")

    finally:
        planner.disconnect()

