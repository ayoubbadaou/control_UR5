import pybullet as p
import pybullet_data
import numpy as np
import time
from scipy.spatial.transform import Rotation as R
from itertools import combinations
from scipy.spatial.transform import Slerp


class UR5PyBulletPlanner:
    def __init__(self, urdf_path, use_gui=True, self_collision=True):
        """
        Initialize the UR5 PyBullet motion planner

        Args:
            urdf_path (str): Path to the UR5 URDF file
            use_gui (bool): Whether to launch PyBullet with GUI
            self_collision (bool): Enable self-collision in physics (keeps URDF flags)
        """
        self.cid = p.connect(p.GUI if use_gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1.0/240.0)

        # World
        self.plane_id = p.loadURDF("plane.urdf")

        flags = 0
        if self_collision:
            flags |= p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT

        self.robot_id = p.loadURDF(urdf_path, useFixedBase=True, flags=flags)

        # ---- Build joint/link maps ----
        self.joint_indices = []              # revolute only (active DOFs)
        self.joint_names = []
        self.lower = []
        self.upper = []
        self.ranges = []
        self.rest = []
        self.link_name_to_index = {}

        n_j = p.getNumJoints(self.robot_id)
        for i in range(n_j):
            ji = p.getJointInfo(self.robot_id, i)
            jtype = ji[2]
            jname = ji[1].decode("utf-8")
            link_name = ji[12].decode("utf-8")
            self.link_name_to_index[link_name] = i
            if jtype == p.JOINT_REVOLUTE or jtype == p.JOINT_PRISMATIC:
                self.joint_indices.append(i)
                self.joint_names.append(jname)
                lo, hi = ji[8], ji[9]
                if not np.isfinite(lo) or not np.isfinite(hi):
                    lo, hi = -2*np.pi, 2*np.pi
                self.lower.append(lo)
                self.upper.append(hi)
                self.ranges.append(hi - lo if np.isfinite(hi - lo) else 4*np.pi)
                self.rest.append(0.0)

        # Choose end-effector link explicitly by name from your URDF/SRDF
        self.ee_link_name_candidates = ["tool0", "flange", "wrist_3_link"]
        self.ee_link_index = None
        for nm in self.ee_link_name_candidates:
            if nm in self.link_name_to_index:
                self.ee_link_index = self.link_name_to_index[nm]
                break
        if self.ee_link_index is None and len(self.joint_indices) > 0:
            self.ee_link_index = self.joint_indices[-1]
        print(f"[INIT] EE link index: {self.ee_link_index}")

        # compute tool offset if both flange and tool0 exist
        self.tool_offset = None
        if 'flange' in self.link_name_to_index and 'tool0' in self.link_name_to_index:
            # store current zero joint state to compute relative transform
            zero_q = [0.0] * len(self.joint_indices)
            for j, idx in enumerate(self.joint_indices):
                p.resetJointState(self.robot_id, idx, zero_q[j])
            p.stepSimulation()
            ls_flange = p.getLinkState(self.robot_id, self.link_name_to_index['flange'], computeForwardKinematics=True)
            ls_tool = p.getLinkState(self.robot_id, self.link_name_to_index['tool0'], computeForwardKinematics=True)
            pos_f, orn_f = np.array(ls_flange[4]), np.array(ls_flange[5])
            pos_t, orn_t = np.array(ls_tool[4]), np.array(ls_tool[5])
            R_f = R.from_quat([orn_f[0], orn_f[1], orn_f[2], orn_f[3]]).as_matrix()
            R_t = R.from_quat([orn_t[0], orn_t[1], orn_t[2], orn_t[3]]).as_matrix()
            # transform from flange to tool: T_f_to_t
            R_f_to_t = R_f.T.dot(R_t)
            t_f_to_t = R_f.T.dot(pos_t - pos_f)
            self.tool_offset = (t_f_to_t, R.from_matrix(R_f_to_t).as_quat())  # quat xyzw
            print(f"[INIT] Computed tool offset from flange->tool0: trans={t_f_to_t}, quat={self.tool_offset[1]}")

        # Obstacles (body unique ids)
        self.obstacles = []

    # -------------------- Utilities --------------------
    def set_obstacles(self, obstacle_urdfs_or_boxes):
        """
        Load external obstacles.
        Each item can be either:
          - a string path to a URDF file, or
          - a dict like { 'type':'box', 'halfExtents':[x,y,z], 'basePosition':[x,y,z], 'baseOrientation':[x,y,z,w] }
        """
        for item in obstacle_urdfs_or_boxes:
            if isinstance(item, str):
                oid = p.loadURDF(item, useFixedBase=True)
            elif isinstance(item, dict) and item.get('type') == 'box':
                he = item.get('halfExtents', [0.1, 0.1, 0.1])
                pos = item.get('basePosition', [0, 0, 0])
                orn = item.get('baseOrientation', [0, 0, 0, 1])
                col = p.createCollisionShape(p.GEOM_BOX, halfExtents=he)
                vis = p.createVisualShape(p.GEOM_BOX, halfExtents=he)
                oid = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
                                        basePosition=pos, baseOrientation=orn)
            else:
                continue
            self.obstacles.append(oid)
        print(f"[OBSTACLES] {len(self.obstacles)} obstacles loaded.")

    def get_current_joint_positions(self):
        return np.array([p.getJointState(self.robot_id, j)[0] for j in self.joint_indices])

    def get_current_tcp_pose(self):
        ls = p.getLinkState(self.robot_id, self.ee_link_index, computeForwardKinematics=True)
        pos = ls[4]
        orn = ls[5]  # xyzw
        rvec = R.from_quat([orn[0], orn[1], orn[2], orn[3]]).as_rotvec()
        return np.array(list(pos) + list(rvec))

    @staticmethod
    def rvec_to_quat(rvec):
        q = R.from_rotvec(rvec).as_quat()  # [x, y, z, w]
        return [q[3], q[0], q[1], q[2]]  # [w, x, y, z]

    @staticmethod
    def pos_rvec_to_pose(pos, rvec):
        return pos, UR5PyBulletPlanner.rvec_to_quat(rvec)

    # -------------------- Planning helpers --------------------
    def _apply_tool_offset_to_target(self, pos, quat_xyzw):
        """
        If tool offset exists (flange->tool0), compute flange target pose that produces desired tool pose.
        Input quat_xyzw is [x,y,z,w]
        Returns (pos_flange, quat_flange_xyzw)
        """
        if self.tool_offset is None:
            return pos, quat_xyzw
        t_f_to_t, q_xyzw = self.tool_offset
        # convert to matrices
        R_t = R.from_quat([quat_xyzw[0], quat_xyzw[1], quat_xyzw[2], quat_xyzw[3]]).as_matrix()
        R_f_to_t = R.from_quat([q_xyzw[0], q_xyzw[1], q_xyzw[2], q_xyzw[3]]).as_matrix()
        R_f = R_t.dot(R_f_to_t.T)
        pos = np.array(pos)
        pos_f = pos - R_f.dot(t_f_to_t)
        quat_f = R.from_matrix(R_f).as_quat()  # xyzw
        return pos_f.tolist(), quat_f.tolist()

    def _ik(self, pos, quat_xyzw, rest_pose=None):
        if rest_pose is None:
            rest_pose = self.get_current_joint_positions()
        try:
            sol = p.calculateInverseKinematics(
                bodyUniqueId=self.robot_id,
                endEffectorLinkIndex=self.ee_link_index,
                targetPosition=pos,
                targetOrientation=quat_xyzw,
                lowerLimits=self.lower,
                upperLimits=self.upper,
                jointRanges=self.ranges,
                restPoses=rest_pose.tolist(),
                maxNumIterations=512,
                residualThreshold=1e-4
            )
        except TypeError:
            sol = p.calculateInverseKinematics(self.robot_id, self.ee_link_index, pos, quat_xyzw,
                                               maxNumIterations=256, residualThreshold=1e-4)
        return np.array(sol)[:len(self.joint_indices)]

    def _set_joint_state(self, q):
        for j, idx in enumerate(self.joint_indices):
            p.resetJointState(self.robot_id, idx, float(q[j]))

    def _in_collision(self, q, margin=0.0):
        """Check collisions of the robot configuration q against external obstacles only.
        Self-collision checks are intentionally skipped to avoid false positives from the URDF frames.
        """
        cur = self.get_current_joint_positions()
        self._set_joint_state(q)
        p.stepSimulation()

        # Check collision only with external obstacles (ignore self-collision)
        for obs_id in self.obstacles:
            pts = p.getClosestPoints(self.robot_id, obs_id, margin)
            if len(pts) > 0:
                self._set_joint_state(cur)
                return True

        # Restore state
        self._set_joint_state(cur)
        p.stepSimulation()
        return False

    # -------------------- Cartesian planning --------------------
    def plan_cartesian_line(self, target_pos, target_rvec, steps=200, check_collision=True):
        """
        Plan a Cartesian straight-line in task space from current TCP to target.
        For each interpolated waypoint we solve IK and collision-check.
        Returns {'status': 'Success', 'position': traj} or status 'IKFail'/'InCollision'.
        """
        # current TCP
        current_tcp = self.get_current_tcp_pose()
        curr_pos = current_tcp[:3]
        curr_rvec = current_tcp[3:]

        # convert rvec->quat xyzw
        target_quat = R.from_rotvec(target_rvec).as_quat()  # xyz,w
        curr_quat = R.from_rotvec(curr_rvec).as_quat()

        # prepare SLERP once
        key_times = [0, 1]
        key_rots = R.from_quat([curr_quat, target_quat])
        slerp = Slerp(key_times, key_rots)

        traj_js = []
        for i, alpha in enumerate(np.linspace(0.0, 1.0, steps)):
            pos_i = (1 - alpha) * curr_pos + alpha * np.array(target_pos)
            quat_i = slerp([alpha])[0].as_quat()

            # compute flange target if needed
            pos_flange, quat_flange = self._apply_tool_offset_to_target(pos_i, quat_i.tolist())
            # IK for this waypoint
            q_sol = self._ik(pos_flange, quat_flange)
            if q_sol is None or len(q_sol) != len(self.joint_indices):
                print(f"[PLANNING] IK failed at waypoint {i}/{steps}")
                return {"status": "IKFail"}
            if check_collision and self._in_collision(q_sol):
                print(f"[PLANNING] Collision at waypoint {i}/{steps}")
                return {"status": "InCollision"}
            traj_js.append(q_sol.tolist())
        print("[PLANNING] Cartesian trajectory planned.")
        return {"status": "Success", "position": traj_js}

    # -------------------- Execution --------------------
    def execute_trajectory(self, trajectory, slow_dt=0.02, max_force=200):
        if trajectory is None or len(trajectory) == 0:
            print("[EXECUTE] Empty trajectory.")
            return False
        traj = np.array(trajectory)
        n = len(traj)
        for i in range(n):
            q = traj[i]
            for j, idx in enumerate(self.joint_indices):
                p.setJointMotorControl2(self.robot_id, idx, controlMode=p.POSITION_CONTROL,
                                        targetPosition=float(q[j]), force=max_force)
            p.stepSimulation()
            time.sleep(slow_dt)
            if i % 20 == 0 or i >= n - 1:
                print(f"[EXECUTE] {i+1}/{n}")
        print("[EXECUTE] Trajectory execution finished.")
        return True

    # -------------------- High-level --------------------
    def move_to_tcp_pose(self, pos, rvec, tol=2e-3, steps=200):
        print("[MOVE] Planning Cartesian line to target...")
        plan = self.plan_cartesian_line(pos, rvec, steps=steps, check_collision=True)
        if plan["status"] != "Success":
            print(f"[MOVE] Planning failed: {plan['status']}")
            return False
        executed = self.execute_trajectory(plan["position"], slow_dt=0.02)

        actual = self.get_current_tcp_pose()
        pos_error = np.linalg.norm(np.array(pos) - actual[:3])
        orient_error = np.linalg.norm(np.array(rvec) - actual[3:])
        print(f"[MOVE] pos_error={pos_error:.6f}, orient_error={orient_error:.6f}")
        return executed and (pos_error < tol)

    def disconnect(self):
        p.disconnect(self.cid)


if __name__ == "__main__":
    planner = UR5PyBulletPlanner(
        urdf_path="/home/robot/Bureau/ur5_control/ur.urdf",
        use_gui=True,
        self_collision=True,
    )

    
    planner.set_obstacles([
        {"type": "box", "halfExtents": [0.325, 0.325, 0.01], "basePosition": [-0.205, 0.0, -0.02]},
        {"type": "box", "halfExtents": [0.02, 0.02, 0.4], "basePosition": [-0.8, 0.30, 0.4]},
        {"type": "box", "halfExtents": [0.125, 0.02, 0.02], "basePosition": [-0.675, 0.30, 0.82]},
    ])
    

    try:
        pos = [-0.3, 0.2, 0.4]
        rvec = [3.5, -0.4, -2.1]
        ok = planner.move_to_tcp_pose(pos, rvec, steps=200)
        print("SUCCESS" if ok else "FAILED")

        # keep GUI open until user interrupts
        print("[SIM] Execution finished. Keep GUI open. Press Ctrl+C to exit.")
        while True:
            p.stepSimulation()
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("[SIM] Interrupted by user.")
    finally:
        planner.disconnect()

