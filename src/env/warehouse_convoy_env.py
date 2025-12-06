import numpy as np

from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.api.objects import DynamicCuboid

from isaacsim.robot.wheeled_robots.robots import WheeledRobot

from .warehouse_convoy_cfg import WarehouseConvoyCfg


class WarehouseConvoyEnv:
    """
    Environment wrapper that:
      - creates a World
      - loads the warehouse
      - spawns a Jetbot convoy (as kinematic agents)
      - spawns extra obstacles (pallets)
      - exposes robots, formation offsets, obstacle centers
    """

    def __init__(self, cfg: WarehouseConvoyCfg):
        self.cfg = cfg

        # Create world and ground plane
        self.world = World()
        self.world.scene.add_default_ground_plane()

        # Locate assets root (Nucleus)
        assets_root = get_assets_root_path()
        if assets_root is None:
            raise RuntimeError(
                "Could not find Nucleus assets root. "
                "Open Isaac Sim GUI and connect to /Isaac assets."
            )
        self.assets_root = assets_root

        # Load warehouse
        env_usd_path = assets_root + cfg.warehouse_relpath
        add_reference_to_stage(usd_path=env_usd_path, prim_path="/World/Warehouse")
        print("[ENV] Loaded warehouse:", env_usd_path)

        # Spawn robots (Jetbots treated as kinematic agents)
        self.robots, self.formation_offsets = self._create_jetbot_convoy()

        # Spawn extra obstacles (pallets) to show CBF later
        self.extra_obstacle_prims, self.obstacle_centers = self._create_extra_obstacles()

    # --------------------------------------------------------------------- #
    # Internal helpers
    # --------------------------------------------------------------------- #

    def _create_jetbot_convoy(self):
        """
        Create Jetbot leader + followers, but we will move them kinematically
        by directly setting their world poses each physics step.
        """
        cfg = self.cfg

        robots = []

        jetbot_usd = self.assets_root + cfg.jetbot_relpath
        print("[ENV] Using Jetbot asset:", jetbot_usd)

        leader_start = np.array(
            [cfg.leader_start_xy[0], cfg.leader_start_xy[1], 0.0]
        )

        # Leader
        leader = self.world.scene.add(
            WheeledRobot(
                prim_path="/World/Jetbot_0",
                name="jetbot_0",
                wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
                create_robot=True,
                usd_path=jetbot_usd,
                position=leader_start,
            )
        )
        robots.append(leader)

        # Followers behind leader in x-direction
        for i in range(cfg.num_followers):
            x = leader_start[0] - (i + 1) * cfg.spacing
            y = leader_start[1]
            follower = self.world.scene.add(
                WheeledRobot(
                    prim_path=f"/World/Jetbot_{i+1}",
                    name=f"jetbot_{i+1}",
                    wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
                    create_robot=True,
                    usd_path=jetbot_usd,
                    position=np.array([x, y, leader_start[2]]),
                )
            )
            robots.append(follower)

        # Formation offsets (in world frame)
        formation_offsets = []
        for i in range(cfg.num_followers):
            formation_offsets.append(np.array([-(i + 1) * cfg.spacing, 0.0]))

        return robots, formation_offsets

    def _create_extra_obstacles(self):
        """
        Add visible pallet-like obstacles and store their centers.
        These will later be used as CBF obstacle points.
        """
        obstacle_prims = []
        obstacle_centers = []

        pallet_height = 0.5
        pallet_size_xy = (0.6, 0.4)  # width, depth

        for idx, (ox, oy) in enumerate(self.cfg.extra_obstacle_xy):
            pos = np.array([ox, oy, pallet_height / 2.0])
            prim_path = f"/World/ExtraPallet_{idx}"
            pallet = self.world.scene.add(
                DynamicCuboid(
                    prim_path=prim_path,
                    name=f"extra_pallet_{idx}",
                    position=pos,
                    scale=np.array([pallet_size_xy[0], pallet_size_xy[1], pallet_height]),
                    color=np.array([0.8, 0.5, 0.1]),  # brown-ish
                )
            )
            obstacle_prims.append(pallet)
            obstacle_centers.append(np.array([ox, oy]))

        print(f"[ENV] Created {len(obstacle_prims)} extra pallet obstacles.")

        return obstacle_prims, obstacle_centers

    # --------------------------------------------------------------------- #
    # Public API
    # --------------------------------------------------------------------- #

    def reset(self):
        """Call world.reset() once everything is created."""
        self.world.reset()

    def add_physics_callback(self, name: str, callback):
        """Forward physics callback registration to world."""
        self.world.add_physics_callback(name, callback)

    def step(self, render: bool = True):
        """Advance the world by one physics step."""
        self.world.step(render=render)
