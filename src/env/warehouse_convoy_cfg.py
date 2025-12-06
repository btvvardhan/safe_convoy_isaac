from dataclasses import dataclass
from typing import Tuple


@dataclass
class WarehouseConvoyCfg:
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

    # Extra pallet obstacles (for CBF later)
    # Positions are in (x, y); z will be handled in env
    extra_obstacle_xy: Tuple[Tuple[float, float], ...] = (
        (0.0, 0.8),
        (0.0, -0.8),
    )
