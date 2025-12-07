import numpy as np
from typing import List
import cvxpy as cp

import os
import sys

# Add project src/ to sys.path so we can import env.*
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_ROOT = os.path.dirname(THIS_DIR)  # /home/teja/safe_convoy_isaac/src

if SRC_ROOT not in sys.path:
    sys.path.append(SRC_ROOT)

from isaacsim.robot.wheeled_robots.robots import WheeledRobot

from env.warehouse_convoy_env import WarehouseConvoyEnv
from env.warehouse_convoy_cfg import WarehouseConvoyCfg


class ClfCbfConvoyControllerDebug:
    """
    CLF-CBF controller with extensive debugging and robust solver fallback.
    
    CRITICAL FIX: Always references env.obstacle_centers dynamically so it
    sees moving obstacles in real-time!
    """

    def __init__(self, env: WarehouseConvoyEnv, cfg: WarehouseConvoyCfg):
        self.env = env
        self.cfg = cfg

        self.robots: List[WheeledRobot] = env.robots
        self.formation_offsets = env.formation_offsets
        
        # ❌ DON'T DO THIS: self.obstacle_centers = env.obstacle_centers
        # That creates a STALE copy! Moving obstacles won't be seen!
        # ✅ Instead: Always reference env.obstacle_centers directly

        self.goal_xy = np.array(cfg.goal_xy, dtype=float)

        # CLF-CBF parameters (INCREASED SAFETY MARGINS)
        self.gamma_clf = 2.0
        self.alpha_cbf = 3.0
        self.d_safe = 0.6    # INCREASED from 0.35 to 0.6
        self.d_robot = 0.5   # INCREASED from 0.3 to 0.5
        self.slack_penalty = 1000.0

        self.last_qp_status = {}
        self.cbf_active_count = {}
        self.step_count = 0
        
        # Detect working solver
        self.working_solver = self._detect_working_solver()
        
        print(f"[CBF-DEBUG] Controller initialized:")
        print(f"[CBF-DEBUG]   Using solver: {self.working_solver}")
        print(f"[CBF-DEBUG]   d_safe (obstacles) = {self.d_safe}m")
        print(f"[CBF-DEBUG]   d_robot (robots) = {self.d_robot}m")
        print(f"[CBF-DEBUG]   alpha_cbf = {self.alpha_cbf}")
        print(f"[CBF-DEBUG]   Initial obstacles: {len(self.env.obstacle_centers)}")
        for i, obs in enumerate(self.env.obstacle_centers):
            print(f"[CBF-DEBUG]     Obstacle {i}: {obs}")

    def _detect_working_solver(self):
        """Test which solver works with this cvxpy/OSQP setup."""
        # Create tiny test problem
        u = cp.Variable(2)
        obj = cp.Minimize(cp.sum_squares(u))
        constraints = [u[0] <= 1, u[0] >= -1, u[1] <= 1, u[1] >= -1]
        prob = cp.Problem(obj, constraints)
        
        # Try solvers in order of preference
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
                    print(f"[CBF-DEBUG] ✅ {name} solver works!")
                    return solver
            except Exception as e:
                print(f"[CBF-DEBUG] ❌ {name} failed: {str(e)[:50]}")
                continue
        
        # If all fail, default to OSQP and hope for the best
        print(f"[CBF-DEBUG] ⚠️  All solvers failed, using OSQP anyway")
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

    def _compute_clf_constraint(self, pos: np.ndarray, ref: np.ndarray, 
                                 u: cp.Variable, slack: cp.Variable):
        error = pos - ref
        V = np.dot(error, error)
        dV_dx = 2 * error
        V_dot = dV_dx @ u
        return V_dot + self.gamma_clf * V <= slack

    def _compute_cbf_obstacle_constraints(self, pos: np.ndarray, u: cp.Variable, robot_idx: int):
        """
        CRITICAL: Always get FRESH obstacle list from env!
        """
        constraints = []
        active_obstacles = []
        
        # ✅ Get fresh obstacle list (includes moving obstacles!)
        obstacle_centers = self.env.obstacle_centers
        
        for obs_idx, obs in enumerate(obstacle_centers):
            diff = pos - obs
            dist = np.linalg.norm(diff)
            dist_sq = np.dot(diff, diff)
            h = dist_sq - self.d_safe**2
            
            # Log obstacle proximity
            if dist < 3.0:  # Within 3m
                if robot_idx not in self.cbf_active_count:
                    self.cbf_active_count[robot_idx] = {}
                self.cbf_active_count[robot_idx][obs_idx] = dist
            
            # Enforce CBF if close
            if h < 4.0:  # INCREASED from 2.0 to 4.0 (enforce earlier!)
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
        # Decision variables
        u = cp.Variable(2)
        slack = cp.Variable(1)
        
        # Objective
        objective = cp.Minimize(
            cp.sum_squares(u - u_nom) + self.slack_penalty * cp.sum_squares(slack)
        )
        
        # Constraints
        constraints = []
        
        # CLF
        if use_clf:
            constraints.append(self._compute_clf_constraint(pos, ref, u, slack))
            constraints.append(slack >= 0)
        
        # CBF obstacles (will get FRESH list from env!)
        cbf_constraints, active_obstacles = self._compute_cbf_obstacle_constraints(pos, u, robot_idx)
        constraints.extend(cbf_constraints)
        
        # CBF robots
        constraints.extend(self._compute_cbf_robot_constraints(pos, u, robot_idx))
        
        # Velocity bounds (box constraint for QP compatibility)
        constraints.append(u[0] <= self.cfg.v_max)
        constraints.append(u[0] >= -self.cfg.v_max)
        constraints.append(u[1] <= self.cfg.v_max)
        constraints.append(u[1] >= -self.cfg.v_max)
        
        # DEBUG: Print when CBF is active
        if active_obstacles and self.step_count % 50 == 0:
            print(f"\n[CBF-DEBUG] Robot {robot_idx} at {pos}, nominal u = {u_nom}")
            print(f"[CBF-DEBUG]   Active CBF obstacles: {len(active_obstacles)}")
            for obs_idx, dist in active_obstacles:
                print(f"[CBF-DEBUG]     Obstacle {obs_idx}: dist = {dist:.3f}m")
        
        # Solve with working solver
        problem = cp.Problem(objective, constraints)
        try:
            problem.solve(solver=self.working_solver, verbose=False)
            
            if problem.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
                u_safe = u.value
                
                # DEBUG: Check if control was modified
                deviation = np.linalg.norm(u_safe - u_nom)
                if deviation > 0.01 and self.step_count % 50 == 0:
                    print(f"[CBF-DEBUG]   QP modified control! deviation = {deviation:.3f}")
                    print(f"[CBF-DEBUG]   u_nom = {u_nom}, u_safe = {u_safe}")
                
                self.last_qp_status[robot_idx] = "optimal"
                return u_safe
            else:
                # QP failed
                if self.step_count % 100 == 0:  # Reduce spam
                    print(f"[CBF-DEBUG] Robot {robot_idx} QP FAILED: {problem.status}")
                self.last_qp_status[robot_idx] = f"failed_{problem.status}"
                return self._clamp_vec(u_nom, self.cfg.v_max)
        except Exception as e:
            if self.step_count % 100 == 0:  # Reduce spam
                print(f"[CBF-DEBUG] Robot {robot_idx} QP ERROR: {str(e)[:80]}")
            self.last_qp_status[robot_idx] = f"error"
            return self._clamp_vec(u_nom, self.cfg.v_max)

    def physics_step(self, step_size: float):
        self.step_count += 1
        cfg = self.cfg
        leader = self.robots[0]

        # Leader
        leader_xy, leader_z, leader_quat = self._get_xy_z_quat(leader)
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
        """Print current safety margins with FRESH obstacle data."""
        print("\n=== Safety Margins ===")
        
        # Get fresh obstacle list
        obstacle_centers = self.env.obstacle_centers
        
        for i, robot in enumerate(self.robots):
            pos, _, _ = self._get_xy_z_quat(robot)
            
            if len(obstacle_centers) > 0:
                obs_dists = [np.linalg.norm(pos - obs) for obs in obstacle_centers]
                min_obs_dist = min(obs_dists)
                min_obs_idx = obs_dists.index(min_obs_dist)
                
                # Color code the output
                if min_obs_dist < self.d_safe:
                    marker = "🔴"  # DANGER!
                elif min_obs_dist < self.d_safe + 0.2:
                    marker = "🟡"  # CBF should be active
                else:
                    marker = "🟢"  # Safe
                
                print(f"{marker} Robot {i}: min_obs_dist = {min_obs_dist:.3f}m (to obs {min_obs_idx}, d_safe={self.d_safe}m)")
            
            other_dists = []
            for j, other in enumerate(self.robots):
                if i != j:
                    other_pos, _, _ = self._get_xy_z_quat(other)
                    other_dists.append(np.linalg.norm(pos - other_pos))
            if other_dists:
                min_robot_dist = min(other_dists)
                print(f"   Robot {i}: min_robot_dist = {min_robot_dist:.3f}m")