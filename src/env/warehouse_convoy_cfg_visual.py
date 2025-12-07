from dataclasses import dataclass
from typing import Tuple, List


@dataclass
class WarehouseConvoyCfgVisual:
    """
    Enhanced configuration for visual presentation demo.
    
    Features:
    - Moving obstacles (humans, carts, forklifts)
    - Visual effects (trails, bubbles, colors)
    - Multiple scenarios
    - Recording-ready settings
    """
    
    # ================================================================
    # ROBOTS
    # ================================================================
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
    
    # ================================================================
    # ASSETS
    # ================================================================
    warehouse_relpath: str = (
        "/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd"
    )
    jetbot_relpath: str = "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"
    
    # ================================================================
    # STATIC OBSTACLES (pallets that don't move)
    # ================================================================
    static_obstacle_xy: Tuple[Tuple[float, float], ...] = (
        (-3.0, 0.5),   # Left side
        (3.0, -0.5),   # Right side
    )
    
    # ================================================================
    # MOVING OBSTACLES (humans, carts, etc.)
    # ================================================================
    enable_moving_obstacles: bool = True
    
    # Moving obstacle configurations
    # Each entry: (start_x, start_y, motion_type, speed, params)
    moving_obstacles_config: List[dict] = None  # Will be set in __post_init__
    
    def __post_init__(self):
        if self.moving_obstacles_config is None:
            self.moving_obstacles_config = [
                # Human crossing from left to right
                {
                    'id': 'human_1',
                    'type': 'human',
                    'start_xy': (0.0, -3.0),
                    'motion': 'linear',
                    'target_xy': (0.0, 3.0),
                    'speed': 0.4,
                    'size': (0.4, 0.3, 1.8),  # width, depth, height
                    'color': (0.2, 0.4, 0.8),  # Blue
                },
                # Cart going in circles
                {
                    'id': 'cart_1',
                    'type': 'cart',
                    'start_xy': (-1.5, 0.0),
                    'motion': 'circular',
                    'center_xy': (-1.5, 0.0),
                    'radius': 1.0,
                    'speed': 0.3,
                    'size': (0.6, 0.6, 1.0),
                    'color': (0.8, 0.6, 0.0),  # Orange
                },
                # Human zigzagging
                {
                    'id': 'human_2',
                    'type': 'human',
                    'start_xy': (2.0, -2.0),
                    'motion': 'zigzag',
                    'amplitude': 0.8,
                    'frequency': 0.5,
                    'direction': (0.0, 1.0),  # Moving up
                    'speed': 0.35,
                    'size': (0.4, 0.3, 1.8),
                    'color': (0.8, 0.2, 0.2),  # Red
                },
            ]
    
    # ================================================================
    # VISUAL EFFECTS
    # ================================================================
    enable_visual_effects: bool = True
    
    # Robot trails
    enable_robot_trails: bool = True
    trail_length: int = 50  # number of points in trail
    trail_fade: bool = True  # fade older points
    
    # Safety bubbles (show CBF activation zones)
    enable_safety_bubbles: bool = True
    safety_bubble_alpha: float = 0.3  # transparency
    
    # Robot danger coloring
    enable_danger_coloring: bool = True
    safe_color: Tuple[float, float, float] = (0.2, 0.8, 0.2)  # Green
    warning_color: Tuple[float, float, float] = (0.8, 0.8, 0.0)  # Yellow
    danger_color: Tuple[float, float, float] = (0.8, 0.2, 0.2)  # Red
    
    # Velocity vectors (arrows showing motion)
    enable_velocity_vectors: bool = True
    vector_scale: float = 0.5  # arrow length multiplier
    
    # Labels
    enable_labels: bool = True
    label_offset_z: float = 1.0  # height above object
    
    # ================================================================
    # DATA LOGGING
    # ================================================================
    enable_logging: bool = True
    log_frequency: int = 10  # log every N steps
    log_directory: str = "./logs"
    
    # ================================================================
    # SCENARIO SETTINGS
    # ================================================================
    scenario: str = "default"  # Options: default, corridor, intersection, zigzag, emergency, rush_hour
    
    # Scenario-specific configs (can be overridden)
    scenario_configs: dict = None
    
    def __post_init__(self):
        if self.scenario_configs is None:
            self.scenario_configs = {
                'corridor': {
                    'leader_start_xy': (-4.0, 0.0),
                    'goal_xy': (4.0, 0.0),
                    'moving_obstacles_config': [
                        {
                            'id': 'human_corridor',
                            'type': 'human',
                            'start_xy': (2.0, 0.0),
                            'motion': 'linear',
                            'target_xy': (-2.0, 0.0),
                            'speed': 0.5,
                            'size': (0.4, 0.3, 1.8),
                            'color': (0.2, 0.4, 0.8),
                        }
                    ]
                },
                'intersection': {
                    'leader_start_xy': (-4.0, 0.0),
                    'goal_xy': (4.0, 0.0),
                    'moving_obstacles_config': [
                        # # North
                        # {
                        #     'id': 'human_north',
                        #     'type': 'human',
                        #     'start_xy': (0.0, 3.0),
                        #     'motion': 'linear',
                        #     'target_xy': (0.0, -3.0),
                        #     'speed': 0.4,
                        #     'size': (0.4, 0.3, 1.8),
                        #     'color': (0.2, 0.4, 0.8),
                        # },
                        # # South
                        # {
                        #     'id': 'human_south',
                        #     'type': 'human',
                        #     'start_xy': (0.0, -3.0),
                        #     'motion': 'linear',
                        #     'target_xy': (0.0, 3.0),
                        #     'speed': 0.4,
                        #     'size': (0.4, 0.3, 1.8),
                        #     'color': (0.8, 0.2, 0.2),
                        # },
                        # East
                        {
                            'id': 'cart_east',
                            'type': 'cart',
                            'start_xy': (3.0, -0.3),
                            'motion': 'linear',
                            'target_xy': (-3.0, -0.3),
                            'speed': 0.35,
                            'size': (0.6, 0.6, 1.0),
                            'color': (0.8, 0.6, 0.0),
                        },
                        # West
                        # {
                        #     'id': 'cart_west',
                        #     'type': 'cart',
                        #     'start_xy': (-3.0, 0.3),
                        #     'motion': 'linear',
                        #     'target_xy': (3.0, 0.3),
                        #     'speed': 0.35,
                        #     'size': (0.6, 0.6, 1.0),
                        #     'color': (0.2, 0.8, 0.2),
                        # },
                    ]
                },
                'zigzag': {
                    'leader_start_xy': (-4.0, 0.0),
                    'goal_xy': (4.0, 0.0),
                    'moving_obstacles_config': [
                        {
                            'id': 'human_zigzag',
                            'type': 'human',
                            'start_xy': (-1.0, -2.0),
                            'motion': 'zigzag',
                            'amplitude': 1.2,
                            'frequency': 0.7,
                            'direction': (0.0, 1.0),
                            'speed': 0.4,
                            'size': (0.4, 0.3, 1.8),
                            'color': (0.8, 0.2, 0.8),
                        }
                    ]
                },
                'emergency': {
                    'leader_start_xy': (-4.0, 0.0),
                    'goal_xy': (4.0, 0.0),
                    'moving_obstacles_config': [
                        {
                            'id': 'human_sudden',
                            'type': 'human',
                            'start_xy': (1.0, -3.0),
                            'motion': 'sudden_stop',
                            'target_xy': (1.0, 0.0),
                            'speed': 0.8,  # Fast!
                            'stop_time': 5.0,  # Stop at t=5s
                            'size': (0.4, 0.3, 1.8),
                            'color': (0.9, 0.1, 0.1),
                        }
                    ]
                },
                'rush_hour': {
                    'leader_start_xy': (-4.0, 0.0),
                    'goal_xy': (4.0, 0.0),
                    'moving_obstacles_config': [
                        # Multiple humans going different directions
                        {
                            'id': 'human_1',
                            'type': 'human',
                            'start_xy': (-2.0, -2.5),
                            'motion': 'linear',
                            'target_xy': (-2.0, 2.5),
                            'speed': 0.45,
                            'size': (0.4, 0.3, 1.8),
                            'color': (0.3, 0.3, 0.8),
                        },
                        {
                            'id': 'human_2',
                            'type': 'human',
                            'start_xy': (0.0, 2.5),
                            'motion': 'linear',
                            'target_xy': (0.0, -2.5),
                            'speed': 0.4,
                            'size': (0.4, 0.3, 1.8),
                            'color': (0.8, 0.3, 0.3),
                        },
                        {
                            'id': 'cart_1',
                            'type': 'cart',
                            'start_xy': (2.0, -2.0),
                            'motion': 'zigzag',
                            'amplitude': 0.5,
                            'frequency': 0.8,
                            'direction': (0.0, 1.0),
                            'speed': 0.35,
                            'size': (0.6, 0.6, 1.0),
                            'color': (0.7, 0.5, 0.0),
                        },
                        {
                            'id': 'human_3',
                            'type': 'human',
                            'start_xy': (2.5, 0.5),
                            'motion': 'circular',
                            'center_xy': (2.5, 0.5),
                            'radius': 0.8,
                            'speed': 0.3,
                            'size': (0.4, 0.3, 1.8),
                            'color': (0.3, 0.8, 0.3),
                        },
                    ]
                },
            }
    
    def load_scenario(self, scenario_name: str):
        """Load a specific scenario configuration."""
        if scenario_name in self.scenario_configs:
            self.scenario = scenario_name
            scenario_cfg = self.scenario_configs[scenario_name]
            
            # Update config with scenario-specific settings
            if 'leader_start_xy' in scenario_cfg:
                self.leader_start_xy = scenario_cfg['leader_start_xy']
            if 'goal_xy' in scenario_cfg:
                self.goal_xy = scenario_cfg['goal_xy']
            if 'moving_obstacles_config' in scenario_cfg:
                self.moving_obstacles_config = scenario_cfg['moving_obstacles_config']
            
            print(f"[CONFIG] Loaded scenario: {scenario_name}")
        else:
            print(f"[CONFIG] Unknown scenario: {scenario_name}")
