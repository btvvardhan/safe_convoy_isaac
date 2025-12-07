import numpy as np
import sys
import os

# Add parent directory to path for imports
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.append(THIS_DIR)

from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid  # ADDED FixedCuboid!
from isaacsim.robot.wheeled_robots.robots import WheeledRobot

from warehouse_convoy_cfg_visual import WarehouseConvoyCfgVisual
from moving_obstacle_manager import MovingObstacleManager


class WarehouseConvoyEnvVisual:
    """
    Enhanced environment with:
    - Moving obstacles (humans, carts) - NOW USING FIXED CUBOIDS SO THEY CAN'T BE PUSHED!
    - Visual effects support
    - Real-time obstacle list updates
    """

    def __init__(self, cfg: WarehouseConvoyCfgVisual):
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

        # Spawn static obstacles (pallets)
        self.static_obstacle_prims, self.static_obstacle_centers = self._create_static_obstacles()

        # Initialize moving obstacles
        self.moving_obstacle_manager = None
        self.moving_obstacle_prims = []
        if cfg.enable_moving_obstacles:
            self._create_moving_obstacles()

        # Combined obstacle list (updated each frame)
        self.obstacle_centers = []
        self._update_obstacle_centers()

        # Time tracking
        self.current_time = 0.0

        print(f"[ENV] Visual environment ready!")
        print(f"[ENV] - Static obstacles: {len(self.static_obstacle_centers)}")
        print(f"[ENV] - Moving obstacles: {len(self.moving_obstacle_prims) if self.moving_obstacle_manager else 0}")

    # --------------------------------------------------------------------- #
    # Robot creation
    # --------------------------------------------------------------------- #

    def _create_jetbot_convoy(self):
        """Create Jetbot convoy."""
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

        # Followers
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

        # Formation offsets
        formation_offsets = []
        for i in range(cfg.num_followers):
            formation_offsets.append(np.array([-(i + 1) * cfg.spacing, 0.0]))

        return robots, formation_offsets

    # --------------------------------------------------------------------- #
    # Static obstacles
    # --------------------------------------------------------------------- #

    def _create_static_obstacles(self):
        """Create static pallet obstacles."""
        obstacle_prims = []
        obstacle_centers = []

        pallet_height = 0.5
        pallet_size_xy = (0.6, 0.4)

        for idx, (ox, oy) in enumerate(self.cfg.static_obstacle_xy):
            pos = np.array([ox, oy, pallet_height / 2.0])
            prim_path = f"/World/StaticPallet_{idx}"
            
            # Use FixedCuboid so they can't be pushed
            pallet = self.world.scene.add(
                FixedCuboid(
                    prim_path=prim_path,
                    name=f"static_pallet_{idx}",
                    position=pos,
                    scale=np.array([pallet_size_xy[0], pallet_size_xy[1], pallet_height]),
                    color=np.array([0.6, 0.4, 0.1]),  # Brown
                )
            )
            obstacle_prims.append(pallet)
            obstacle_centers.append(np.array([ox, oy]))

        return obstacle_prims, obstacle_centers

    # --------------------------------------------------------------------- #
    # Moving obstacles
    # --------------------------------------------------------------------- #

    def _create_moving_obstacles(self):
        """
        Create moving obstacles (humans, carts, etc.).
        
        CRITICAL FIX: Use FixedCuboid instead of DynamicCuboid!
        - FixedCuboid = kinematic, won't be pushed by robots
        - We manually update position via set_world_pose()
        - Robots will avoid them but can't push them
        """
        # Create manager
        self.moving_obstacle_manager = MovingObstacleManager(
            self.cfg.moving_obstacles_config
        )

        # Create visual prims for each obstacle
        for obs_config in self.cfg.moving_obstacles_config:
            width, depth, height = obs_config['size']
            pos = np.array([
                obs_config['start_xy'][0],
                obs_config['start_xy'][1],
                height / 2.0
            ])
            
            prim_path = f"/World/MovingObstacle_{obs_config['id']}"
            
            # CRITICAL: Use FixedCuboid so robots can't push them!
            prim = self.world.scene.add(
                FixedCuboid(  # CHANGED from DynamicCuboid!
                    prim_path=prim_path,
                    name=f"moving_{obs_config['id']}",
                    position=pos,
                    scale=np.array([width, depth, height]),
                    color=np.array(obs_config['color']),
                )
            )
            self.moving_obstacle_prims.append(prim)
            
        print(f"[ENV] Created {len(self.moving_obstacle_prims)} FIXED (non-pushable) moving obstacles")

    def _update_moving_obstacle_positions(self):
        """Update visual prims to match obstacle manager positions."""
        if self.moving_obstacle_manager is None:
            return
        
        obstacle_info = self.moving_obstacle_manager.get_obstacle_info()
        
        for prim, info in zip(self.moving_obstacle_prims, obstacle_info):
            # Update position (keep z at half height)
            new_pos = np.array([
                info['position'][0],
                info['position'][1],
                info['size'][2] / 2.0  # height / 2
            ])
            # FixedCuboid can still be moved via set_world_pose!
            prim.set_world_pose(position=new_pos)

    # --------------------------------------------------------------------- #
    # Obstacle list management
    # --------------------------------------------------------------------- #

    def _update_obstacle_centers(self):
        """
        Update combined obstacle list (static + moving).
        This is what the CLF-CBF controller uses!
        """
        self.obstacle_centers = []
        
        # Add static obstacles
        self.obstacle_centers.extend(self.static_obstacle_centers)
        
        # Add moving obstacles
        if self.moving_obstacle_manager is not None:
            moving_positions = self.moving_obstacle_manager.get_positions()
            for pos in moving_positions:
                self.obstacle_centers.append(pos.copy())

    # --------------------------------------------------------------------- #
    # Update loop
    # --------------------------------------------------------------------- #

    def update_dynamic_elements(self, dt: float):
        """
        Call this each physics step to update moving obstacles.
        """
        self.current_time += dt
        
        # Update moving obstacle logic
        if self.moving_obstacle_manager is not None:
            self.moving_obstacle_manager.update(dt)
            self._update_moving_obstacle_positions()
        
        # Update combined obstacle list for controller
        self._update_obstacle_centers()

    # --------------------------------------------------------------------- #
    # Public API
    # --------------------------------------------------------------------- #

    def reset(self):
        """Reset environment."""
        self.world.reset()
        self.current_time = 0.0
        
        if self.moving_obstacle_manager is not None:
            self.moving_obstacle_manager.reset()
            self._update_moving_obstacle_positions()
        
        self._update_obstacle_centers()

    def add_physics_callback(self, name: str, callback):
        """Register physics callback."""
        self.world.add_physics_callback(name, callback)

    def step(self, render: bool = True):
        """Advance simulation by one step."""
        self.world.step(render=render)

    def get_obstacle_count(self):
        """Get obstacle statistics."""
        return {
            "static": len(self.static_obstacle_centers),
            "moving": len(self.moving_obstacle_prims) if self.moving_obstacle_manager else 0,
            "total": len(self.obstacle_centers)
        }
    
    def get_moving_obstacle_info(self):
        """Get detailed info about moving obstacles."""
        if self.moving_obstacle_manager is None:
            return []
        return self.moving_obstacle_manager.get_obstacle_info()