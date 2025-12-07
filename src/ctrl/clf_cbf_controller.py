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


class ClfCbfConvoyController:
    """
    CLF-CBF based safety-critical convoy controller.
    
    Control Lyapunov Function (CLF): Ensures formation convergence
    Control Barrier Function (CBF): Guarantees collision avoidance
    
    Architecture:
    - Leader: Go-to-goal with CBF constraints (obstacles only)
    - Followers: Track formation with CLF + CBF (obstacles + other robots)
    
    Each robot solves a QP at each timestep:
        minimize    ||u - u_nominal||^2 + slack_penalty * slack^2
        subject to  CLF_constraint: V̇ + γ*V ≤ slack        (formation)
                    CBF_constraints: ḣ + α*h ≥ 0           (safety)
                    u bounds: ||u|| ≤ v_max
                    slack ≥ 0
    
    where:
    - u = [u_x, u_y] is the control input (velocity in world frame)
    - u_nominal is the desired control (e.g., go-to-goal or formation tracking)
    - V is the CLF (distance to formation reference)
    - h is the CBF (distance to obstacle - safety margin)
    """

    def __init__(self, env: WarehouseConvoyEnv, cfg: WarehouseConvoyCfg):
        self.env = env
        self.cfg = cfg

        self.robots: List[WheeledRobot] = env.robots
        self.formation_offsets = env.formation_offsets
        self.obstacle_centers = env.obstacle_centers  # static obstacles from env

        self.goal_xy = np.array(cfg.goal_xy, dtype=float)

        # CLF-CBF parameters
        self.gamma_clf = 2.0  # CLF convergence rate
        self.alpha_cbf = 3.0  # CBF class-K function parameter
        self.d_safe = 0.35    # Safety margin (m) for obstacles
        self.d_robot = 0.3    # Safety margin (m) for robot-robot
        self.slack_penalty = 1000.0  # High penalty to minimize CLF slack

        # For debugging/visualization (optional)
        self.last_qp_status = {}

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #

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

    # ------------------------------------------------------------------ #
    # CLF (Control Lyapunov Function)
    # ------------------------------------------------------------------ #

    def _compute_clf_constraint(self, pos: np.ndarray, ref: np.ndarray, 
                                 u: cp.Variable, slack: cp.Variable):
        """
        CLF: V(x) = ||x - x_ref||^2
        CLF derivative: V̇ = 2*(x - x_ref)^T * u
        
        Constraint: V̇ + γ*V ≤ slack
                    2*(x - x_ref)^T * u + γ*||x - x_ref||^2 ≤ slack
        
        Returns: constraint for cvxpy
        """
        error = pos - ref
        V = np.dot(error, error)
        
        # Linear approximation of V̇
        dV_dx = 2 * error
        V_dot = dV_dx @ u
        
        return V_dot + self.gamma_clf * V <= slack

    # ------------------------------------------------------------------ #
    # CBF (Control Barrier Function)
    # ------------------------------------------------------------------ #

    def _compute_cbf_obstacle_constraints(self, pos: np.ndarray, u: cp.Variable):
        """
        CBF for obstacles: h(x) = ||x - obs||^2 - d_safe^2
        
        ḣ = 2*(x - obs)^T * u
        
        Constraint: ḣ + α*h ≥ 0
                    2*(x - obs)^T * u + α*(||x - obs||^2 - d_safe^2) ≥ 0
        
        Returns: list of constraints
        """
        constraints = []
        
        for obs in self.obstacle_centers:
            diff = pos - obs
            dist_sq = np.dot(diff, diff)
            h = dist_sq - self.d_safe**2
            
            # Only enforce if we're close to the obstacle
            if h < 2.0:  # Only enforce within 2m vicinity
                dh_dx = 2 * diff
                h_dot = dh_dx @ u
                constraints.append(h_dot + self.alpha_cbf * h >= 0)
        
        return constraints

    def _compute_cbf_robot_constraints(self, pos: np.ndarray, u: cp.Variable, 
                                        robot_idx: int):
        """
        CBF for robot-robot collision avoidance.
        Same form as obstacle CBF but with d_robot margin.
        
        Returns: list of constraints
        """
        constraints = []
        
        for j, other_robot in enumerate(self.robots):
            if j == robot_idx:
                continue
            
            other_pos, _, _ = self._get_xy_z_quat(other_robot)
            diff = pos - other_pos
            dist_sq = np.dot(diff, diff)
            h = dist_sq - self.d_robot**2
            
            # Only enforce if robots are close
            if h < 1.5:  # Within 1.5m vicinity
                dh_dx = 2 * diff
                h_dot = dh_dx @ u
                constraints.append(h_dot + self.alpha_cbf * h >= 0)
        
        return constraints

    # ------------------------------------------------------------------ #
    # QP Solver
    # ------------------------------------------------------------------ #

    def _solve_qp(self, u_nom: np.ndarray, pos: np.ndarray, ref: np.ndarray, 
                  robot_idx: int, use_clf: bool = True):
        """
        Solve the CLF-CBF QP for a single robot.
        
        Args:
            u_nom: nominal control (2D vector)
            pos: current position (2D)
            ref: reference position (2D) - only used if use_clf=True
            robot_idx: index of robot in self.robots
            use_clf: whether to include CLF constraint (False for leader)
        
        Returns:
            u_safe: safe control (2D numpy array)
        """
        # Decision variables
        u = cp.Variable(2)
        slack = cp.Variable(1)
        
        # Objective: minimize deviation from nominal + slack
        objective = cp.Minimize(
            cp.sum_squares(u - u_nom) + self.slack_penalty * cp.sum_squares(slack)
        )
        
        # Constraints
        constraints = []
        
        # 1) CLF constraint (formation tracking) - only for followers
        if use_clf:
            constraints.append(self._compute_clf_constraint(pos, ref, u, slack))
            constraints.append(slack >= 0)
        
        # 2) CBF constraints (obstacle avoidance)
        constraints.extend(self._compute_cbf_obstacle_constraints(pos, u))
        
        # 3) CBF constraints (robot-robot avoidance)
        constraints.extend(self._compute_cbf_robot_constraints(pos, u, robot_idx))
        
        # 4) Velocity bounds
        constraints.append(cp.norm(u, 2) <= self.cfg.v_max)
        
        # Solve
        problem = cp.Problem(objective, constraints)
        try:
            problem.solve(solver=cp.OSQP, verbose=False)
            
            if problem.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
                u_safe = u.value
                self.last_qp_status[robot_idx] = "optimal"
                return u_safe
            else:
                # QP failed, return nominal (clamped)
                self.last_qp_status[robot_idx] = f"failed_{problem.status}"
                return self._clamp_vec(u_nom, self.cfg.v_max)
        except Exception as e:
            # Solver error, return nominal
            self.last_qp_status[robot_idx] = f"error_{str(e)[:20]}"
            return self._clamp_vec(u_nom, self.cfg.v_max)

    # ------------------------------------------------------------------ #
    # Physics callback
    # ------------------------------------------------------------------ #

    def physics_step(self, step_size: float):
        """
        Called every physics step by the World.
        
        For each robot:
        1. Compute nominal control (go-to-goal or formation tracking)
        2. Solve CLF-CBF QP to get safe control
        3. Apply control by updating position: x_{k+1} = x_k + u_safe * dt
        """
        cfg = self.cfg
        leader = self.robots[0]

        # ================================================================
        # LEADER: Go-to-goal with CBF (no CLF)
        # ================================================================
        leader_xy, leader_z, leader_quat = self._get_xy_z_quat(leader)
        to_goal = self.goal_xy - leader_xy
        dist_to_goal = np.linalg.norm(to_goal)

        if dist_to_goal > 0.2:
            u_nom_leader = cfg.leader_speed * to_goal / (dist_to_goal + 1e-6)
        else:
            u_nom_leader = np.zeros(2)

        # Solve QP (CBF only, no CLF for leader)
        u_safe_leader = self._solve_qp(
            u_nom=u_nom_leader,
            pos=leader_xy,
            ref=self.goal_xy,  # not used since use_clf=False
            robot_idx=0,
            use_clf=False
        )

        # Apply control
        new_leader_xy = leader_xy + u_safe_leader * step_size
        leader.set_world_pose(
            position=np.array([new_leader_xy[0], new_leader_xy[1], leader_z]),
            orientation=leader_quat,
        )

        # Re-read leader position after moving
        leader_xy, leader_z, _ = self._get_xy_z_quat(leader)

        # ================================================================
        # FOLLOWERS: Formation tracking with CLF + CBF
        # ================================================================
        for i in range(1, len(self.robots)):
            follower = self.robots[i]
            follower_xy, follower_z, follower_quat = self._get_xy_z_quat(follower)

            # Formation reference
            offset = self.formation_offsets[i - 1]
            ref_xy = leader_xy + offset

            # Nominal control (proportional)
            error = follower_xy - ref_xy
            u_nom_follower = -cfg.follower_k * error

            # Solve CLF-CBF QP
            u_safe_follower = self._solve_qp(
                u_nom=u_nom_follower,
                pos=follower_xy,
                ref=ref_xy,
                robot_idx=i,
                use_clf=True
            )

            # Apply control
            new_follower_xy = follower_xy + u_safe_follower * step_size
            follower.set_world_pose(
                position=np.array([new_follower_xy[0], new_follower_xy[1], follower_z]),
                orientation=follower_quat,
            )

    # ------------------------------------------------------------------ #
    # Optional: Debug/Visualization helpers
    # ------------------------------------------------------------------ #

    def get_qp_status(self):
        """Returns dict of QP solve status for each robot (for debugging)."""
        return self.last_qp_status

    def print_safety_margins(self):
        """Print current safety margins (for debugging)."""
        print("\n=== Safety Margins ===")
        for i, robot in enumerate(self.robots):
            pos, _, _ = self._get_xy_z_quat(robot)
            
            # Min distance to obstacles
            if len(self.obstacle_centers) > 0:
                obs_dists = [np.linalg.norm(pos - obs) for obs in self.obstacle_centers]
                min_obs_dist = min(obs_dists)
                print(f"Robot {i}: min_obs_dist = {min_obs_dist:.3f}m")
            
            # Min distance to other robots
            other_dists = []
            for j, other in enumerate(self.robots):
                if i != j:
                    other_pos, _, _ = self._get_xy_z_quat(other)
                    other_dists.append(np.linalg.norm(pos - other_pos))
            if other_dists:
                min_robot_dist = min(other_dists)
                print(f"Robot {i}: min_robot_dist = {min_robot_dist:.3f}m")
