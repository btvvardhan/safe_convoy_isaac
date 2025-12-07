from dataclasses import dataclass
from typing import Tuple


@dataclass
class WarehouseConvoyCfgEnhanced:
    """
    Enhanced config with:
    - Static obstacles IN the convoy path
    - Dynamic obstacle spawning
    - Moving "human" agent options
    - Visual enhancements
    """
    
    # Robots
    num_followers: int = 3
    leader_start_xy: Tuple[float, float] = (-4.0, 0.0)
    goal_xy: Tuple[float, float] = (4.0, 0.0)

    # Formation
    spacing: float = 0.7  # distance between robots along x

    # Control gains
    leader_speed: float = 0.6
    follower_k: float = 1.5
    v_max: float = 0.7
    w_max: float = 2.0

    # Assets (we will use get_assets_root_path and append these)
    warehouse_relpath: str = (
        "/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd"
    )
    jetbot_relpath: str = "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"

    # ================================================================
    # STATIC OBSTACLES - Now actually IN the path!
    # ================================================================
    # These are RIGHT in the convoy's path (y=0) so CBF will activate!
    extra_obstacle_xy: Tuple[Tuple[float, float], ...] = (
        (-2.0, 0.0),   # Left side of path
        (0.0, 0.15),   # Center-right (slight offset)
        (2.0, -0.15),  # Right side (slight offset)
    )

    # ================================================================
    # DYNAMIC OBSTACLES - Random spawning!
    # ================================================================
    enable_dynamic_obstacles: bool = True
    dynamic_obstacle_spawn_rate: float = 0.01  # probability per physics step
    dynamic_obstacle_spawn_region: Tuple[Tuple[float, float], Tuple[float, float]] = (
        (-3.0, 3.0),   # x range
        (-0.5, 0.5),   # y range (near convoy path)
    )
    max_dynamic_obstacles: int = 5  # max number at once
    dynamic_obstacle_lifetime: float = 10.0  # seconds before despawn

    # ================================================================
    # MOVING "HUMAN" AGENT - Crosses the aisle!
    # ================================================================
    enable_moving_agent: bool = True
    human_crossing_speed: float = 0.3  # m/s
    human_path_y_range: Tuple[float, float] = (-2.0, 2.0)  # crosses from -2 to +2
    human_x_position: float = 1.0  # crosses at x = 1.0
    
    # ================================================================
    # VISUAL ENHANCEMENTS
    # ================================================================
    # Color robots based on CBF activation
    enable_robot_coloring: bool = True
    # Static obstacles - brown
    static_obstacle_color: Tuple[float, float, float] = (0.6, 0.4, 0.1)
    # Dynamic obstacles - bright orange (more visible!)
    dynamic_obstacle_color: Tuple[float, float, float] = (1.0, 0.5, 0.0)
    # Moving human - blue/cyan
    human_agent_color: Tuple[float, float, float] = (0.0, 0.7, 1.0)
