import numpy as np
import random

from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.api.objects import DynamicCuboid

from isaacsim.robot.wheeled_robots.robots import WheeledRobot

from .warehouse_convoy_cfg_enhanced import WarehouseConvoyCfgEnhanced


class WarehouseConvoyEnvEnhanced:
    """
    Enhanced environment with:
    - Static obstacles IN the path
    - Dynamic randomly spawning obstacles
    - Moving "human" agent crossing the aisle
    - Real-time obstacle list updates for CLF-CBF controller
    """

    def __init__(self, cfg: WarehouseConvoyCfgEnhanced):
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

        # Spawn STATIC obstacles (pallets IN the path!)
        self.static_obstacle_prims, self.static_obstacle_centers = self._create_static_obstacles()

        # Dynamic obstacles (spawn randomly during simulation)
        self.dynamic_obstacle_prims = []
        self.dynamic_obstacle_centers = []
        self.dynamic_obstacle_spawn_times = []  # track when spawned
        self.current_time = 0.0

        # Moving human agent
        self.human_agent = None
        self.human_position = np.array([cfg.human_x_position, cfg.human_path_y_range[0]])
        self.human_direction = 1  # 1 = moving up, -1 = moving down
        if cfg.enable_moving_agent:
            self._create_moving_human()

        # Combined obstacle list (updated each step)
        self.obstacle_centers = []
        self._update_obstacle_centers()

        print(f"[ENV] Enhanced environment ready!")
        print(f"[ENV] - Static obstacles: {len(self.static_obstacle_centers)}")
        print(f"[ENV] - Dynamic spawning: {cfg.enable_dynamic_obstacles}")
        print(f"[ENV] - Moving agent: {cfg.enable_moving_agent}")

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
        """Create static pallet obstacles IN the convoy path."""
        obstacle_prims = []
        obstacle_centers = []

        pallet_height = 0.5
        pallet_size_xy = (0.6, 0.4)

        for idx, (ox, oy) in enumerate(self.cfg.extra_obstacle_xy):
            pos = np.array([ox, oy, pallet_height / 2.0])
            prim_path = f"/World/StaticPallet_{idx}"
            
            pallet = self.world.scene.add(
                DynamicCuboid(
                    prim_path=prim_path,
                    name=f"static_pallet_{idx}",
                    position=pos,
                    scale=np.array([pallet_size_xy[0], pallet_size_xy[1], pallet_height]),
                    color=np.array(self.cfg.static_obstacle_color),
                )
            )
            obstacle_prims.append(pallet)
            obstacle_centers.append(np.array([ox, oy]))
            print(f"[ENV] Static obstacle {idx} at ({ox:.2f}, {oy:.2f})")

        return obstacle_prims, obstacle_centers

    # --------------------------------------------------------------------- #
    # Dynamic obstacles (random spawning)
    # --------------------------------------------------------------------- #

    def _try_spawn_dynamic_obstacle(self):
        """Randomly spawn a dynamic obstacle."""
        if not self.cfg.enable_dynamic_obstacles:
            return
        
        # Check if we've reached max
        if len(self.dynamic_obstacle_prims) >= self.cfg.max_dynamic_obstacles:
            return
        
        # Random spawn check
        if random.random() > self.cfg.dynamic_obstacle_spawn_rate:
            return
        
        # Random position in spawn region
        x_range = self.cfg.dynamic_obstacle_spawn_region[0]
        y_range = self.cfg.dynamic_obstacle_spawn_region[1]
        
        ox = random.uniform(x_range[0], x_range[1])
        oy = random.uniform(y_range[0], y_range[1])
        
        pallet_height = 0.5
        pallet_size_xy = (0.5, 0.5)  # slightly smaller
        pos = np.array([ox, oy, pallet_height / 2.0])
        
        idx = len(self.dynamic_obstacle_prims)
        prim_path = f"/World/DynamicPallet_{idx}_{int(self.current_time * 1000)}"
        
        try:
            pallet = self.world.scene.add(
                DynamicCuboid(
                    prim_path=prim_path,
                    name=f"dynamic_pallet_{idx}",
                    position=pos,
                    scale=np.array([pallet_size_xy[0], pallet_size_xy[1], pallet_height]),
                    color=np.array(self.cfg.dynamic_obstacle_color),  # Bright orange!
                )
            )
            
            self.dynamic_obstacle_prims.append(pallet)
            self.dynamic_obstacle_centers.append(np.array([ox, oy]))
            self.dynamic_obstacle_spawn_times.append(self.current_time)
            
            print(f"[ENV] 🟠 Spawned dynamic obstacle at ({ox:.2f}, {oy:.2f})")
        except Exception as e:
            print(f"[ENV] Failed to spawn dynamic obstacle: {e}")

    def _cleanup_old_dynamic_obstacles(self):
        """Remove dynamic obstacles that have exceeded their lifetime."""
        if not self.cfg.enable_dynamic_obstacles:
            return
        
        indices_to_remove = []
        
        for i, spawn_time in enumerate(self.dynamic_obstacle_spawn_times):
            age = self.current_time - spawn_time
            if age > self.cfg.dynamic_obstacle_lifetime:
                indices_to_remove.append(i)
        
        # Remove in reverse order to avoid index issues
        for i in reversed(indices_to_remove):
            try:
                # Remove from scene
                prim = self.dynamic_obstacle_prims[i]
                self.world.scene.remove_object(prim.name)
                
                print(f"[ENV] 💀 Despawned dynamic obstacle (age: {self.cfg.dynamic_obstacle_lifetime:.1f}s)")
            except:
                pass
            
            # Remove from lists
            del self.dynamic_obstacle_prims[i]
            del self.dynamic_obstacle_centers[i]
            del self.dynamic_obstacle_spawn_times[i]

    # --------------------------------------------------------------------- #
    # Moving human agent
    # --------------------------------------------------------------------- #

    def _create_moving_human(self):
        """Create a moving 'human' agent (tall box) that crosses the aisle."""
        cfg = self.cfg
        
        human_height = 1.8
        human_size = (0.4, 0.3)  # width, depth
        
        pos = np.array([
            self.human_position[0],
            self.human_position[1],
            human_height / 2.0
        ])
        
        self.human_agent = self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/HumanAgent",
                name="human_agent",
                position=pos,
                scale=np.array([human_size[0], human_size[1], human_height]),
                color=np.array(cfg.human_agent_color),  # Blue/cyan
            )
        )
        
        print(f"[ENV] 🚶 Created moving human agent")

    def _update_moving_human(self, dt: float):
        """Update human position (oscillates back and forth)."""
        if not self.cfg.enable_moving_agent or self.human_agent is None:
            return
        
        cfg = self.cfg
        
        # Move human
        self.human_position[1] += self.human_direction * cfg.human_crossing_speed * dt
        
        # Reverse direction if at boundaries
        if self.human_position[1] >= cfg.human_path_y_range[1]:
            self.human_direction = -1
        elif self.human_position[1] <= cfg.human_path_y_range[0]:
            self.human_direction = 1
        
        # Update position in simulation
        human_height = 1.8
        new_pos = np.array([
            self.human_position[0],
            self.human_position[1],
            human_height / 2.0
        ])
        
        self.human_agent.set_world_pose(position=new_pos)

    # --------------------------------------------------------------------- #
    # Obstacle list management
    # --------------------------------------------------------------------- #

    def _update_obstacle_centers(self):
        """
        Combine all obstacle centers (static + dynamic + human) into one list.
        This is what the CLF-CBF controller will use!
        """
        self.obstacle_centers = []
        
        # Static obstacles
        self.obstacle_centers.extend(self.static_obstacle_centers)
        
        # Dynamic obstacles
        self.obstacle_centers.extend(self.dynamic_obstacle_centers)
        
        # Moving human
        if self.cfg.enable_moving_agent and self.human_agent is not None:
            self.obstacle_centers.append(self.human_position.copy())

    # --------------------------------------------------------------------- #
    # Public API
    # --------------------------------------------------------------------- #

    def update_dynamic_elements(self, dt: float):
        """
        Call this each physics step to:
        - Spawn new dynamic obstacles
        - Remove old dynamic obstacles
        - Update moving human position
        - Update combined obstacle list
        """
        self.current_time += dt
        
        # Dynamic obstacle management
        self._try_spawn_dynamic_obstacle()
        self._cleanup_old_dynamic_obstacles()
        
        # Moving human
        self._update_moving_human(dt)
        
        # Update combined obstacle list (for controller)
        self._update_obstacle_centers()

    def reset(self):
        """Call world.reset() once everything is created."""
        self.world.reset()

    def add_physics_callback(self, name: str, callback):
        """Forward physics callback registration to world."""
        self.world.add_physics_callback(name, callback)

    def step(self, render: bool = True):
        """Advance the world by one physics step."""
        self.world.step(render=render)

    def get_obstacle_count(self):
        """Get current obstacle counts (for debugging/display)."""
        return {
            "static": len(self.static_obstacle_centers),
            "dynamic": len(self.dynamic_obstacle_centers),
            "human": 1 if (self.cfg.enable_moving_agent and self.human_agent) else 0,
            "total": len(self.obstacle_centers)
        }
