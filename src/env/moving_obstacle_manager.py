import numpy as np
from typing import List, Dict


class MovingObstacle:
    """Represents a single moving obstacle (human, cart, etc.)."""
    
    def __init__(self, config: dict):
        self.id = config['id']
        self.type = config['type']
        self.motion = config['motion']
        self.speed = config['speed']
        self.size = config['size']
        self.color = config['color']
        
        # Current state
        self.position = np.array(config['start_xy'], dtype=float)
        self.velocity = np.zeros(2, dtype=float)
        self.time = 0.0
        
        # Motion-specific parameters
        self.config = config
        
        # For linear motion
        if self.motion == 'linear':
            self.target = np.array(config['target_xy'], dtype=float)
            direction = self.target - self.position
            dist = np.linalg.norm(direction)
            if dist > 1e-6:
                self.velocity = (direction / dist) * self.speed
        
        # For circular motion
        elif self.motion == 'circular':
            self.center = np.array(config['center_xy'], dtype=float)
            self.radius = config['radius']
            self.angle = 0.0
            self.angular_velocity = self.speed / self.radius
        
        # For zigzag motion
        elif self.motion == 'zigzag':
            self.amplitude = config['amplitude']
            self.frequency = config['frequency']
            self.direction = np.array(config['direction'], dtype=float)
            self.direction = self.direction / (np.linalg.norm(self.direction) + 1e-9)
            self.base_velocity = self.direction * self.speed
        
        # For sudden stop
        elif self.motion == 'sudden_stop':
            self.target = np.array(config['target_xy'], dtype=float)
            self.stop_time = config.get('stop_time', 5.0)
            self.stopped = False
            direction = self.target - self.position
            dist = np.linalg.norm(direction)
            if dist > 1e-6:
                self.velocity = (direction / dist) * self.speed
    
    def update(self, dt: float):
        """Update obstacle position based on motion pattern."""
        self.time += dt
        
        if self.motion == 'linear':
            self._update_linear(dt)
        elif self.motion == 'circular':
            self._update_circular(dt)
        elif self.motion == 'zigzag':
            self._update_zigzag(dt)
        elif self.motion == 'sudden_stop':
            self._update_sudden_stop(dt)
    
    def _update_linear(self, dt: float):
        """Linear motion toward target, then reverse."""
        # Move toward target
        self.position += self.velocity * dt
        
        # Check if reached target (within 0.1m)
        to_target = self.target - self.position
        if np.linalg.norm(to_target) < 0.1:
            # Reverse direction
            self.target = np.array(self.config['start_xy'], dtype=float)
            direction = self.target - self.position
            dist = np.linalg.norm(direction)
            if dist > 1e-6:
                self.velocity = (direction / dist) * self.speed
            
            # Swap target and start for next reversal
            self.config['start_xy'] = tuple(self.config['target_xy'])
            self.config['target_xy'] = tuple(self.position)
    
    def _update_circular(self, dt: float):
        """Circular motion around center."""
        self.angle += self.angular_velocity * dt
        
        # Update position
        self.position[0] = self.center[0] + self.radius * np.cos(self.angle)
        self.position[1] = self.center[1] + self.radius * np.sin(self.angle)
        
        # Update velocity (tangent to circle)
        self.velocity[0] = -self.radius * self.angular_velocity * np.sin(self.angle)
        self.velocity[1] = self.radius * self.angular_velocity * np.cos(self.angle)
    
    def _update_zigzag(self, dt: float):
        """Zigzag motion (sinusoidal perpendicular to main direction)."""
        # Main direction movement
        forward_displacement = self.base_velocity * dt
        
        # Perpendicular sinusoidal motion
        perpendicular = np.array([-self.direction[1], self.direction[0]])
        lateral_offset = self.amplitude * np.sin(2 * np.pi * self.frequency * self.time)
        
        # Update position
        self.position += forward_displacement
        
        # Velocity includes both forward and lateral components
        lateral_velocity = (2 * np.pi * self.frequency * self.amplitude * 
                           np.cos(2 * np.pi * self.frequency * self.time))
        self.velocity = self.base_velocity + perpendicular * lateral_velocity
    
    def _update_sudden_stop(self, dt: float):
        """Move toward target, then suddenly stop."""
        if not self.stopped and self.time >= self.stop_time:
            # STOP!
            self.velocity = np.zeros(2)
            self.stopped = True
        
        if not self.stopped:
            # Move toward target
            self.position += self.velocity * dt
            
            # Check if reached target
            to_target = self.target - self.position
            if np.linalg.norm(to_target) < 0.1:
                self.position = self.target.copy()
                self.velocity = np.zeros(2)
                self.stopped = True


class MovingObstacleManager:
    """Manages all moving obstacles in the environment."""
    
    def __init__(self, obstacle_configs: List[dict]):
        self.obstacles: List[MovingObstacle] = []
        
        for config in obstacle_configs:
            self.obstacles.append(MovingObstacle(config))
        
        print(f"[MOVING_OBS] Created {len(self.obstacles)} moving obstacles")
    
    def update(self, dt: float):
        """Update all obstacles."""
        for obstacle in self.obstacles:
            obstacle.update(dt)
    
    def get_positions(self) -> np.ndarray:
        """Get current positions of all obstacles."""
        if len(self.obstacles) == 0:
            return np.empty((0, 2))
        return np.array([obs.position for obs in self.obstacles])
    
    def get_velocities(self) -> np.ndarray:
        """Get current velocities of all obstacles."""
        if len(self.obstacles) == 0:
            return np.empty((0, 2))
        return np.array([obs.velocity for obs in self.obstacles])
    
    def get_obstacle_info(self) -> List[Dict]:
        """Get detailed info about each obstacle."""
        info = []
        for obs in self.obstacles:
            info.append({
                'id': obs.id,
                'type': obs.type,
                'position': obs.position.copy(),
                'velocity': obs.velocity.copy(),
                'size': obs.size,
                'color': obs.color,
            })
        return info
    
    def reset(self):
        """Reset all obstacles to initial positions."""
        for obstacle in self.obstacles:
            obstacle.position = np.array(obstacle.config['start_xy'], dtype=float)
            obstacle.time = 0.0
            obstacle.velocity = np.zeros(2)
            
            # Re-initialize velocity based on motion type
            if obstacle.motion == 'linear':
                obstacle.target = np.array(obstacle.config['target_xy'], dtype=float)
                direction = obstacle.target - obstacle.position
                dist = np.linalg.norm(direction)
                if dist > 1e-6:
                    obstacle.velocity = (direction / dist) * obstacle.speed
            
            elif obstacle.motion == 'sudden_stop':
                obstacle.stopped = False
                obstacle.target = np.array(obstacle.config['target_xy'], dtype=float)
                direction = obstacle.target - obstacle.position
                dist = np.linalg.norm(direction)
                if dist > 1e-6:
                    obstacle.velocity = (direction / dist) * obstacle.speed
        
        print("[MOVING_OBS] Reset all obstacles to initial state")
