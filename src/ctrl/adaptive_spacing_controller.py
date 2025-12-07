import numpy as np
from typing import List
import cvxpy as cp

import os
import sys

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_ROOT = os.path.dirname(THIS_DIR)

if SRC_ROOT not in sys.path:
    sys.path.append(SRC_ROOT)

from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from env.warehouse_convoy_env import WarehouseConvoyEnv
from env.warehouse_convoy_cfg import WarehouseConvoyCfg


class AdaptiveSpacingClfCbfController:
    """
    CLF-CBF controller with ADAPTIVE SAFETY MARGINS based on corridor width.
    
    Key Features:
    - Detects available space around robot
    - Wide corridors → Conservative (d_safe = 0.8m, spacious navigation)
    - Narrow passages → Aggressive (d_safe = 0.4m, tight navigation)
    - Smooth transitions between modes
    - Visual indicators in console
    """

    def __init__(self, env: WarehouseConvoyEnv, cfg: WarehouseConvoyCfg):
        self.env = env
        self.cfg = cfg

        self.robots: List[WheeledRobot] = env.robots
        self.formation_offsets = env.formation_offsets

        self.goal_xy = np.array(cfg.goal_xy, dtype=float)

        # CLF-CBF parameters
        self.gamma_clf = 2.0
        self.alpha_cbf = 3.0
        self.d_robot = 0.5
        self.slack_penalty = 1000.0

        # ADAPTIVE SAFETY MARGINS
        self.d_safe_wide = 0.8      # Wide corridors (conservative)
        self.d_safe_narrow = 0.4    # Narrow passages (aggressive)
        self.d_safe_current = {}    # Per-robot current safety margin
        
        # Space detection parameters
        self.wide_threshold = 2.5   # >2.5m clearance = wide
        self.narrow_threshold = 1.5 # <1.5m clearance = narrow
        
        # Tracking
        self.last_qp_status = {}
        self.cbf_active_count = {}
        self.step_count = 0
        self.spacing_mode = {}  # Per-robot: 'wide', 'narrow', 'transition'
        
        # Initialize all robots in wide mode
        for i in range(len(self.robots)):
            self.d_safe_current[i] = self.d_safe_wide
            self.spacing_mode[i] = 'wide'
        
        # Detect working solver
        self.working_solver = self._detect_working_solver()
        
        print(f"[ADAPTIVE-CBF] Controller initialized:")
        print(f"[ADAPTIVE-CBF]   Using solver: {self.working_solver}")
        print(f"[ADAPTIVE-CBF]   ADAPTIVE SPACING ENABLED")
        print(f"[ADAPTIVE-CBF]     Wide mode:   d_safe = {self.d_safe_wide}m (conservative)")
        print(f"[ADAPTIVE-CBF]     Narrow mode: d_safe = {self.d_safe_narrow}m (aggressive)")
        print(f"[ADAPTIVE-CBF]   Initial obstacles: {len(self.env.obstacle_centers)}")

    def _detect_working_solver(self):
        """Test which solver works."""
        u = cp.Variable(2)
        obj = cp.Minimize(cp.sum_squares(u))
        constraints = [u[0] <= 1, u[0] >= -1, u[1] <= 1, u[1] >= -1]
        prob = cp.Problem(obj, constraints)
        
        solvers_to_try = [
            ('CLARABEL', cp.CLARABEL),
            ('SCS', cp.SCS),
            ('ECOS', cp.ECOS),
            ('OSQP', cp.OSQP),
        ]
        
        for name, solver in solvers_to_try:
            try:
                prob.solve(solver=solver, verbose=False)
                if prob.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
                    return solver
            except:
                continue
        
        return cp.OSQP

    def _get_xy_z_quat(self, robot: WheeledRobot):
        pos, quat = robot.get_world_pose()
        xy = np.array([pos[0], pos[1]])
        z = pos[2]
        return xy, z, quat

    def _clamp_vec(self, v: np.ndarray, max_norm: float):
        n = np.linalg.norm(v)
        if n > max_norm:
            return v * (max_norm / (n + 1e-9))
        return v

    def _detect_corridor_width(self, robot_pos: np.ndarray) -> float:
        """
        Detect available space around robot.
        
        Returns: estimated corridor width in meters
        """
        obstacle_centers = self.env.obstacle_centers
        
        if len(obstacle_centers) == 0:
            return 10.0  # No obstacles = wide open
        
        # Find obstacles in lateral vicinity (perpendicular to goal direction)
        goal_dir = self.goal_xy - robot_pos
        goal_dir = goal_dir / (np.linalg.norm(goal_dir) + 1e-9)
        
        # Perpendicular direction (left/right)
        perp_dir = np.array([-goal_dir[1], goal_dir[0]])
        
        # Project obstacles onto perpendicular axis
        left_distances = []
        right_distances = []
        
        for obs in obstacle_centers:
            to_obs = obs - robot_pos
            forward_dist = np.dot(to_obs, goal_dir)
            
            # Only consider obstacles in front (±3m forward)
            if abs(forward_dist) < 3.0:
                lateral_dist = np.dot(to_obs, perp_dir)
                
                if lateral_dist > 0:
                    left_distances.append(lateral_dist)
                else:
                    right_distances.append(abs(lateral_dist))
        
        # Estimate corridor width
        if left_distances and right_distances:
            clearance_left = min(left_distances)
            clearance_right = min(right_distances)
            corridor_width = clearance_left + clearance_right
        elif left_distances:
            corridor_width = min(left_distances) * 2
        elif right_distances:
            corridor_width = min(right_distances) * 2
        else:
            corridor_width = 10.0  # Wide open
        
        return corridor_width

    def _update_adaptive_safety_margin(self, robot_idx: int, robot_pos: np.ndarray):
        """Update safety margin based on detected corridor width."""
        corridor_width = self._detect_corridor_width(robot_pos)
        
        # Determine mode
        if corridor_width > self.wide_threshold:
            target_mode = 'wide'
            target_d_safe = self.d_safe_wide
        elif corridor_width < self.narrow_threshold:
            target_mode = 'narrow'
            target_d_safe = self.d_safe_narrow
        else:
            # Transition zone: interpolate
            target_mode = 'transition'
            t = (corridor_width - self.narrow_threshold) / (self.wide_threshold - self.narrow_threshold)
            target_d_safe = self.d_safe_narrow + t * (self.d_safe_wide - self.d_safe_narrow)
        
        # Smooth transition (exponential moving average)
        alpha = 0.1  # Smoothing factor
        current_d_safe = self.d_safe_current[robot_idx]
        new_d_safe = (1 - alpha) * current_d_safe + alpha * target_d_safe
        
        self.d_safe_current[robot_idx] = new_d_safe
        self.spacing_mode[robot_idx] = target_mode
        
        return corridor_width

    def _compute_clf_constraint(self, pos: np.ndarray, ref: np.ndarray, 
                                 u: cp.Variable, slack: cp.Variable):
        error = pos - ref
        V = np.dot(error, error)
        dV_dx = 2 * error
        V_dot = dV_dx @ u
        return V_dot + self.gamma_clf * V <= slack

    def _compute_cbf_obstacle_constraints(self, pos: np.ndarray, u: cp.Variable, robot_idx: int):
        """CBF with ADAPTIVE safety margin."""
        constraints = []
        active_obstacles = []
        
        obstacle_centers = self.env.obstacle_centers
        d_safe = self.d_safe_current[robot_idx]  # Use adaptive margin!
        
        for obs_idx, obs in enumerate(obstacle_centers):
            diff = pos - obs
            dist = np.linalg.norm(diff)
            dist_sq = np.dot(diff, diff)
            h = dist_sq - d_safe**2
            
            if dist < 3.0:
                if robot_idx not in self.cbf_active_count:
                    self.cbf_active_count[robot_idx] = {}
                self.cbf_active_count[robot_idx][obs_idx] = dist
            
            if h < 4.0:
                dh_dx = 2 * diff
                h_dot = dh_dx @ u
                constraints.append(h_dot + self.alpha_cbf * h >= 0)
                active_obstacles.append((obs_idx, dist))
        
        return constraints, active_obstacles

    def _compute_cbf_robot_constraints(self, pos: np.ndarray, u: cp.Variable, robot_idx: int):
        constraints = []
        
        for j, other_robot in enumerate(self.robots):
            if j == robot_idx:
                continue
            
            other_pos, _, _ = self._get_xy_z_quat(other_robot)
            diff = pos - other_pos
            dist_sq = np.dot(diff, diff)
            h = dist_sq - self.d_robot**2
            
            if h < 2.0:
                dh_dx = 2 * diff
                h_dot = dh_dx @ u
                constraints.append(h_dot + self.alpha_cbf * h >= 0)
        
        return constraints

    def _solve_qp(self, u_nom: np.ndarray, pos: np.ndarray, ref: np.ndarray, 
                  robot_idx: int, use_clf: bool = True):
        u = cp.Variable(2)
        slack = cp.Variable(1)
        
        objective = cp.Minimize(
            cp.sum_squares(u - u_nom) + self.slack_penalty * cp.sum_squares(slack)
        )
        
        constraints = []
        
        if use_clf:
            constraints.append(self._compute_clf_constraint(pos, ref, u, slack))
            constraints.append(slack >= 0)
        
        cbf_constraints, active_obstacles = self._compute_cbf_obstacle_constraints(pos, u, robot_idx)
        constraints.extend(cbf_constraints)
        constraints.extend(self._compute_cbf_robot_constraints(pos, u, robot_idx))
        
        constraints.append(u[0] <= self.cfg.v_max)
        constraints.append(u[0] >= -self.cfg.v_max)
        constraints.append(u[1] <= self.cfg.v_max)
        constraints.append(u[1] >= -self.cfg.v_max)
        
        problem = cp.Problem(objective, constraints)
        try:
            problem.solve(solver=self.working_solver, verbose=False)
            
            if problem.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
                u_safe = u.value
                self.last_qp_status[robot_idx] = "optimal"
                return u_safe
            else:
                self.last_qp_status[robot_idx] = f"failed_{problem.status}"
                return self._clamp_vec(u_nom, self.cfg.v_max)
        except Exception as e:
            self.last_qp_status[robot_idx] = f"error"
            return self._clamp_vec(u_nom, self.cfg.v_max)

    def physics_step(self, step_size: float):
        self.step_count += 1
        cfg = self.cfg
        leader = self.robots[0]

        # Leader
        leader_xy, leader_z, leader_quat = self._get_xy_z_quat(leader)
        
        # Update adaptive safety margin for leader
        corridor_width = self._update_adaptive_safety_margin(0, leader_xy)
        
        to_goal = self.goal_xy - leader_xy
        dist_to_goal = np.linalg.norm(to_goal)

        if dist_to_goal > 0.2:
            u_nom_leader = cfg.leader_speed * to_goal / (dist_to_goal + 1e-6)
        else:
            u_nom_leader = np.zeros(2)

        u_safe_leader = self._solve_qp(
            u_nom=u_nom_leader,
            pos=leader_xy,
            ref=self.goal_xy,
            robot_idx=0,
            use_clf=False
        )

        new_leader_xy = leader_xy + u_safe_leader * step_size
        leader.set_world_pose(
            position=np.array([new_leader_xy[0], new_leader_xy[1], leader_z]),
            orientation=leader_quat,
        )

        leader_xy, leader_z, _ = self._get_xy_z_quat(leader)

        # Followers
        for i in range(1, len(self.robots)):
            follower = self.robots[i]
            follower_xy, follower_z, follower_quat = self._get_xy_z_quat(follower)

            # Update adaptive safety margin for follower
            self._update_adaptive_safety_margin(i, follower_xy)

            offset = self.formation_offsets[i - 1]
            ref_xy = leader_xy + offset

            error = follower_xy - ref_xy
            u_nom_follower = -cfg.follower_k * error

            u_safe_follower = self._solve_qp(
                u_nom=u_nom_follower,
                pos=follower_xy,
                ref=ref_xy,
                robot_idx=i,
                use_clf=True
            )

            new_follower_xy = follower_xy + u_safe_follower * step_size
            follower.set_world_pose(
                position=np.array([new_follower_xy[0], new_follower_xy[1], follower_z]),
                orientation=follower_quat,
            )

    def get_qp_status(self):
        return self.last_qp_status

    def print_safety_margins(self):
        """Print current safety margins with ADAPTIVE MODE indicators."""
        print("\n=== Adaptive Safety Margins ===")
        
        obstacle_centers = self.env.obstacle_centers
        
        for i, robot in enumerate(self.robots):
            pos, _, _ = self._get_xy_z_quat(robot)
            
            d_safe = self.d_safe_current[i]
            mode = self.spacing_mode[i]
            
            # Mode indicator
            if mode == 'wide':
                mode_marker = "🟦 WIDE"
            elif mode == 'narrow':
                mode_marker = "🟥 NARROW"
            else:
                mode_marker = "🟨 TRANSITION"
            
            if len(obstacle_centers) > 0:
                obs_dists = [np.linalg.norm(pos - obs) for obs in obstacle_centers]
                min_obs_dist = min(obs_dists)
                min_obs_idx = obs_dists.index(min_obs_dist)
                
                # Safety indicator
                if min_obs_dist < d_safe:
                    marker = "🔴"
                elif min_obs_dist < d_safe + 0.2:
                    marker = "🟡"
                else:
                    marker = "🟢"
                
                print(f"{marker} Robot {i} [{mode_marker}]: dist={min_obs_dist:.3f}m, d_safe={d_safe:.3f}m (obs {min_obs_idx})")
