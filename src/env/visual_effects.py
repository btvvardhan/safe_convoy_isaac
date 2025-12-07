import numpy as np
from collections import deque
from typing import List, Dict
import sys
import os

# For Isaac Sim visualization primitives
try:
    from pxr import Usd, UsdGeom, Gf
    from isaacsim.core.utils.stage import get_current_stage
    ISAAC_SIM_AVAILABLE = True
except ImportError:
    ISAAC_SIM_AVAILABLE = False
    print("[VISUAL_FX] Warning: Isaac Sim not available, visual effects disabled")


class RobotTrail:
    """Manages trail visualization for a single robot."""
    
    def __init__(self, robot_id: int, max_length: int = 50, color=(0.2, 0.8, 0.2)):
        self.robot_id = robot_id
        self.max_length = max_length
        self.color = color
        self.positions = deque(maxlen=max_length)
    
    def add_position(self, position: np.ndarray):
        """Add a new position to the trail."""
        self.positions.append(position.copy())
    
    def get_positions(self) -> List[np.ndarray]:
        """Get all trail positions."""
        return list(self.positions)
    
    def clear(self):
        """Clear the trail."""
        self.positions.clear()


class VisualEffectsManager:
    """
    Manages all visual effects:
    - Robot trails
    - Safety bubbles
    - Danger coloring
    - Velocity vectors
    - Labels
    """
    
    def __init__(self, config):
        self.cfg = config
        
        # Robot trails
        self.trails: Dict[int, RobotTrail] = {}
        if self.cfg.enable_robot_trails:
            self._init_trails()
        
        # Safety bubble tracking
        self.safety_bubble_data = {}
        
        # Danger coloring tracking
        self.robot_danger_levels = {}
        
        print(f"[VISUAL_FX] Visual effects initialized")
        print(f"[VISUAL_FX]   - Trails: {self.cfg.enable_robot_trails}")
        print(f"[VISUAL_FX]   - Safety bubbles: {self.cfg.enable_safety_bubbles}")
        print(f"[VISUAL_FX]   - Danger coloring: {self.cfg.enable_danger_coloring}")
    
    def _init_trails(self):
        """Initialize trail trackers for each robot."""
        # We'll create trails dynamically as we see robots
        pass
    
    def update(self, robots, obstacle_centers):
        """Update all visual effects based on current state."""
        
        # Update trails
        if self.cfg.enable_robot_trails:
            self._update_trails(robots)
        
        # Update safety bubbles
        if self.cfg.enable_safety_bubbles:
            self._update_safety_bubbles(robots, obstacle_centers)
        
        # Update danger coloring
        if self.cfg.enable_danger_coloring:
            self._update_danger_coloring(robots, obstacle_centers)
    
    def _update_trails(self, robots):
        """Update robot trails."""
        for i, robot in enumerate(robots):
            # Get or create trail
            if i not in self.trails:
                # Color trails differently per robot
                colors = [
                    (0.2, 0.8, 0.2),  # Green
                    (0.2, 0.2, 0.8),  # Blue
                    (0.8, 0.2, 0.8),  # Magenta
                    (0.8, 0.8, 0.2),  # Yellow
                ]
                color = colors[i % len(colors)]
                self.trails[i] = RobotTrail(
                    i, 
                    max_length=self.cfg.trail_length,
                    color=color
                )
            
            # Add current position
            pos, _ = robot.get_world_pose()
            self.trails[i].add_position(np.array([pos[0], pos[1]]))
    
    def _update_safety_bubbles(self, robots, obstacle_centers):
        """Update safety bubble visualization data."""
        self.safety_bubble_data = {}
        
        for i, robot in enumerate(robots):
            pos, _ = robot.get_world_pose()
            robot_xy = np.array([pos[0], pos[1]])
            
            # Find closest obstacle
            if len(obstacle_centers) > 0:
                dists = [np.linalg.norm(robot_xy - obs) for obs in obstacle_centers]
                min_dist = min(dists)
                
                # Store bubble data (will be visualized externally)
                self.safety_bubble_data[i] = {
                    'position': robot_xy.copy(),
                    'min_distance': min_dist,
                    'radius': 0.6,  # d_safe from controller
                    'active': min_dist < 1.0,  # Show bubble if within 1m
                }
    
    def _update_danger_coloring(self, robots, obstacle_centers):
        """Update danger level for each robot."""
        self.robot_danger_levels = {}
        
        for i, robot in enumerate(robots):
            pos, _ = robot.get_world_pose()
            robot_xy = np.array([pos[0], pos[1]])
            
            # Calculate danger level based on closest obstacle
            danger_level = 0.0  # 0 = safe, 1 = danger
            
            if len(obstacle_centers) > 0:
                dists = [np.linalg.norm(robot_xy - obs) for obs in obstacle_centers]
                min_dist = min(dists)
                
                # Compute danger level
                d_safe = 0.6  # Should match controller
                d_danger = 0.3  # Critically close
                
                if min_dist < d_danger:
                    danger_level = 1.0
                elif min_dist < d_safe:
                    danger_level = 0.5
                else:
                    # Gradual increase as approaching safety zone
                    danger_level = max(0.0, 1.0 - (min_dist - d_safe) / 1.0)
            
            self.robot_danger_levels[i] = danger_level
    
    def get_robot_color(self, robot_id: int) -> tuple:
        """Get color for robot based on danger level."""
        if not self.cfg.enable_danger_coloring:
            return (0.8, 0.8, 0.8)  # Default gray
        
        danger_level = self.robot_danger_levels.get(robot_id, 0.0)
        
        safe_color = np.array(self.cfg.safe_color)
        warning_color = np.array(self.cfg.warning_color)
        danger_color = np.array(self.cfg.danger_color)
        
        if danger_level < 0.5:
            # Interpolate between safe and warning
            t = danger_level / 0.5
            color = (1 - t) * safe_color + t * warning_color
        else:
            # Interpolate between warning and danger
            t = (danger_level - 0.5) / 0.5
            color = (1 - t) * warning_color + t * danger_color
        
        return tuple(color)
    
    def get_trail_positions(self, robot_id: int) -> List[np.ndarray]:
        """Get trail positions for a robot."""
        if robot_id in self.trails:
            return self.trails[robot_id].get_positions()
        return []
    
    def get_safety_bubble_data(self):
        """Get all safety bubble data."""
        return self.safety_bubble_data
    
    def reset(self):
        """Reset all visual effects."""
        for trail in self.trails.values():
            trail.clear()
        self.safety_bubble_data = {}
        self.robot_danger_levels = {}
        print("[VISUAL_FX] Reset visual effects")
    
    def print_status(self):
        """Print current visual status (for debugging)."""
        print("\n=== Visual Effects Status ===")
        
        # Trails
        if self.cfg.enable_robot_trails:
            for robot_id, trail in self.trails.items():
                print(f"Robot {robot_id} trail: {len(trail.positions)} points")
        
        # Safety bubbles
        if self.cfg.enable_safety_bubbles:
            active_bubbles = sum(1 for data in self.safety_bubble_data.values() if data['active'])
            print(f"Active safety bubbles: {active_bubbles}/{len(self.safety_bubble_data)}")
        
        # Danger levels
        if self.cfg.enable_danger_coloring:
            for robot_id, level in self.robot_danger_levels.items():
                if level > 0.5:
                    marker = "🔴"
                elif level > 0.2:
                    marker = "🟡"
                else:
                    marker = "🟢"
                print(f"{marker} Robot {robot_id}: danger_level = {level:.2f}")


class SimpleVisualizer:
    """
    Simple console-based visualizer (doesn't require Isaac Sim primitives).
    Prints ASCII art representation of the scene.
    """
    
    def __init__(self, width=80, height=30, world_bounds=(-5, 5, -3, 3)):
        self.width = width
        self.height = height
        self.world_bounds = world_bounds  # (x_min, x_max, y_min, y_max)
    
    def render(self, robots, obstacles):
        """Render ASCII visualization."""
        # Create grid
        grid = [[' ' for _ in range(self.width)] for _ in range(self.height)]
        
        # Helper to convert world coords to grid coords
        def world_to_grid(x, y):
            x_min, x_max, y_min, y_max = self.world_bounds
            grid_x = int((x - x_min) / (x_max - x_min) * (self.width - 1))
            grid_y = int((y - y_min) / (y_max - y_min) * (self.height - 1))
            grid_x = max(0, min(self.width - 1, grid_x))
            grid_y = max(0, min(self.height - 1, grid_y))
            return grid_x, self.height - 1 - grid_y  # Flip y
        
        # Draw obstacles
        for obs in obstacles:
            gx, gy = world_to_grid(obs[0], obs[1])
            grid[gy][gx] = '█'
        
        # Draw robots
        for i, robot in enumerate(robots):
            pos, _ = robot.get_world_pose()
            gx, gy = world_to_grid(pos[0], pos[1])
            grid[gy][gx] = str(i)
        
        # Print grid
        print("\n" + "="*self.width)
        for row in grid:
            print("".join(row))
        print("="*self.width)
        print("Legend: 0-3=Robots, █=Obstacles")
