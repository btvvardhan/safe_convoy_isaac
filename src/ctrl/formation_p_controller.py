import numpy as np
from typing import List

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


class FormationPController:
    """
    Simple leader–follower convoy controller (kinematic, single-integrator):

      - Leader: go-to-goal in (x, y) with bounded speed.
      - Followers: track formation offsets behind leader in (x, y).

    We treat the robots as holonomic agents and directly update their world pose
    using set_world_pose(). This matches the Robotarium single-integrator model
    and avoids articulation-controller issues for our control experiments.
    """

    def __init__(self, env: WarehouseConvoyEnv, cfg: WarehouseConvoyCfg):
        self.env = env
        self.cfg = cfg

        self.robots: List[WheeledRobot] = env.robots
        self.formation_offsets = env.formation_offsets

        self.goal_xy = np.array(cfg.goal_xy, dtype=float)

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
    # Physics callback
    # ------------------------------------------------------------------ #

    def physics_step(self, step_size: float):
        """
        Called every physics step by the World.

        We treat velocities as:
            x_{k+1} = x_k + u * dt
        where u is in world frame (m/s), dt = step_size.
        """
        cfg = self.cfg
        leader = self.robots[0]

        # --- Leader: go-to-goal in (x, y) ---
        leader_xy, leader_z, leader_quat = self._get_xy_z_quat(leader)
        to_goal = self.goal_xy - leader_xy
        dist_to_goal = np.linalg.norm(to_goal)

        if dist_to_goal > 0.2:
            u_leader = cfg.leader_speed * to_goal / (dist_to_goal + 1e-6)
        else:
            u_leader = np.zeros(2)

        u_leader = self._clamp_vec(u_leader, cfg.v_max)

        new_leader_xy = leader_xy + u_leader * step_size

        leader.set_world_pose(
            position=np.array([new_leader_xy[0], new_leader_xy[1], leader_z]),
            orientation=leader_quat,  # keep same orientation
        )

        # --- Followers: track formation behind leader ---
        # Re-read leader position after moving it
        leader_xy, leader_z, _ = self._get_xy_z_quat(leader)

        for i in range(1, len(self.robots)):
            follower = self.robots[i]
            follower_xy, follower_z, follower_quat = self._get_xy_z_quat(follower)

            offset = self.formation_offsets[i - 1]
            ref_xy = leader_xy + offset

            error = follower_xy - ref_xy
            u_f = -cfg.follower_k * error
            u_f = self._clamp_vec(u_f, cfg.v_max)

            new_follower_xy = follower_xy + u_f * step_size

            follower.set_world_pose(
                position=np.array(
                    [new_follower_xy[0], new_follower_xy[1], follower_z]
                ),
                orientation=follower_quat,
            )
