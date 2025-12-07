import numpy as np
import os
import json
from datetime import datetime
from collections import defaultdict


class DataLogger:
    """
    Logs simulation data for analysis and plotting.
    
    Tracks:
    - Robot positions over time
    - Obstacle distances
    - CBF activations
    - QP solve times
    - Safety violations
    - Formation errors
    """
    
    def __init__(self, config, log_dir=None):
        self.cfg = config
        
        # Create log directory
        if log_dir is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            scenario = config.scenario if hasattr(config, 'scenario') else 'default'
            log_dir = os.path.join(config.log_directory, f"{scenario}_{timestamp}")
        
        self.log_dir = log_dir
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Data storage
        self.data = {
            'time': [],
            'robot_positions': defaultdict(list),  # {robot_id: [(x,y), ...]}
            'robot_velocities': defaultdict(list),
            'obstacle_distances': defaultdict(list),  # {robot_id: [dist, ...]}
            'min_obstacle_distance': [],  # Global minimum
            'cbf_activations': defaultdict(list),  # {robot_id: [bool, ...]}
            'qp_solve_status': defaultdict(list),  # {robot_id: [status, ...]}
            'formation_errors': defaultdict(list),  # {robot_id: [error, ...]}
            'safety_violations': [],  # List of (time, robot_id, distance)
        }
        
        # Statistics
        self.stats = {
            'total_steps': 0,
            'total_cbf_activations': 0,
            'qp_failures': 0,
            'safety_violations': 0,
            'min_distance_ever': float('inf'),
        }
        
        # Configuration
        self.log_frequency = config.log_frequency
        self.step_count = 0
        
        print(f"[DATA_LOGGER] Initialized")
        print(f"[DATA_LOGGER]   Log directory: {self.log_dir}")
        print(f"[DATA_LOGGER]   Log frequency: every {self.log_frequency} steps")
    
    def log_step(self, time, robots, obstacle_centers, controller=None):
        """Log data for current timestep."""
        self.step_count += 1
        
        # Only log at specified frequency
        if self.step_count % self.log_frequency != 0:
            return
        
        self.stats['total_steps'] += 1
        
        # Log time
        self.data['time'].append(time)
        
        # Log robot data
        min_dist_this_step = float('inf')
        
        for i, robot in enumerate(robots):
            pos, _ = robot.get_world_pose()
            robot_xy = np.array([pos[0], pos[1]])
            
            # Position
            self.data['robot_positions'][i].append(robot_xy.copy())
            
            # Compute distance to obstacles
            if len(obstacle_centers) > 0:
                dists = [np.linalg.norm(robot_xy - obs) for obs in obstacle_centers]
                min_dist = min(dists)
                self.data['obstacle_distances'][i].append(min_dist)
                
                min_dist_this_step = min(min_dist_this_step, min_dist)
                
                # Check for safety violation (< 0.3m is very dangerous)
                if min_dist < 0.3:
                    self.data['safety_violations'].append((time, i, min_dist))
                    self.stats['safety_violations'] += 1
            else:
                self.data['obstacle_distances'][i].append(float('inf'))
        
        # Log global minimum distance
        self.data['min_obstacle_distance'].append(min_dist_this_step)
        if min_dist_this_step < self.stats['min_distance_ever']:
            self.stats['min_distance_ever'] = min_dist_this_step
        
        # Log controller data if available
        if controller is not None:
            qp_status = controller.get_qp_status()
            for robot_id, status in qp_status.items():
                self.data['qp_solve_status'][robot_id].append(status)
                if status != 'optimal':
                    self.stats['qp_failures'] += 1
    
    def save(self):
        """Save logged data to disk."""
        # Save raw data as numpy arrays
        data_file = os.path.join(self.log_dir, 'data.npz')
        
        # Convert to numpy arrays
        save_dict = {
            'time': np.array(self.data['time']),
        }
        
        for robot_id in self.data['robot_positions'].keys():
            save_dict[f'robot_{robot_id}_positions'] = np.array(
                self.data['robot_positions'][robot_id]
            )
            save_dict[f'robot_{robot_id}_obstacle_dists'] = np.array(
                self.data['obstacle_distances'][robot_id]
            )
        
        save_dict['min_obstacle_distance'] = np.array(
            self.data['min_obstacle_distance']
        )
        
        np.savez(data_file, **save_dict)
        print(f"[DATA_LOGGER] Saved data to {data_file}")
        
        # Save statistics as JSON
        stats_file = os.path.join(self.log_dir, 'statistics.json')
        with open(stats_file, 'w') as f:
            json.dump(self.stats, f, indent=2)
        print(f"[DATA_LOGGER] Saved statistics to {stats_file}")
        
        # Save configuration
        config_file = os.path.join(self.log_dir, 'config.json')
        config_dict = {
            'scenario': self.cfg.scenario if hasattr(self.cfg, 'scenario') else 'default',
            'num_robots': 1 + self.cfg.num_followers,
            'leader_speed': self.cfg.leader_speed,
            'v_max': self.cfg.v_max,
        }
        with open(config_file, 'w') as f:
            json.dump(config_dict, f, indent=2)
        print(f"[DATA_LOGGER] Saved config to {config_file}")
    
    def print_summary(self):
        """Print summary statistics."""
        print("\n" + "="*60)
        print("SIMULATION SUMMARY")
        print("="*60)
        print(f"Total steps logged: {self.stats['total_steps']}")
        print(f"Minimum distance achieved: {self.stats['min_distance_ever']:.3f}m")
        print(f"Safety violations (<0.3m): {self.stats['safety_violations']}")
        print(f"QP failures: {self.stats['qp_failures']}")
        
        # Success rate
        if len(self.data['min_obstacle_distance']) > 0:
            safe_steps = sum(1 for d in self.data['min_obstacle_distance'] if d >= 0.3)
            success_rate = 100.0 * safe_steps / len(self.data['min_obstacle_distance'])
            print(f"Safety success rate: {success_rate:.1f}%")
        
        print("="*60 + "\n")


def generate_plots(log_dir):
    """
    Generate plots from logged data.
    
    This function is separate so it can be run without Isaac Sim.
    Requires: matplotlib
    """
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[PLOT] matplotlib not installed, skipping plots")
        return
    
    # Load data
    data_file = os.path.join(log_dir, 'data.npz')
    if not os.path.exists(data_file):
        print(f"[PLOT] Data file not found: {data_file}")
        return
    
    data = np.load(data_file)
    time = data['time']
    
    # Determine number of robots
    robot_ids = []
    for key in data.keys():
        if key.startswith('robot_') and key.endswith('_positions'):
            robot_id = int(key.split('_')[1])
            robot_ids.append(robot_id)
    robot_ids.sort()
    
    print(f"[PLOT] Generating plots for {len(robot_ids)} robots...")
    
    # ================================================================
    # PLOT 1: Minimum obstacle distance over time
    # ================================================================
    plt.figure(figsize=(10, 6))
    min_dist = data['min_obstacle_distance']
    plt.plot(time, min_dist, 'b-', linewidth=2, label='Min Distance')
    plt.axhline(y=0.6, color='g', linestyle='--', label='Target Safety (0.6m)')
    plt.axhline(y=0.3, color='r', linestyle='--', label='Critical (0.3m)')
    plt.fill_between(time, 0, 0.3, color='red', alpha=0.1)
    plt.fill_between(time, 0.3, 0.6, color='yellow', alpha=0.1)
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('Distance (m)', fontsize=12)
    plt.title('Minimum Obstacle Distance Over Time', fontsize=14, fontweight='bold')
    plt.legend(fontsize=10)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(log_dir, 'plot_min_distance.png'), dpi=150)
    print(f"[PLOT] Saved: plot_min_distance.png")
    
    # ================================================================
    # PLOT 2: Individual robot distances
    # ================================================================
    plt.figure(figsize=(12, 6))
    colors = ['b', 'r', 'g', 'm']
    for i, robot_id in enumerate(robot_ids):
        dists = data[f'robot_{robot_id}_obstacle_dists']
        plt.plot(time, dists, color=colors[i % len(colors)], 
                linewidth=2, label=f'Robot {robot_id}', alpha=0.7)
    plt.axhline(y=0.6, color='k', linestyle='--', linewidth=1, label='Safety Threshold')
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('Distance to Closest Obstacle (m)', fontsize=12)
    plt.title('Per-Robot Obstacle Distances', fontsize=14, fontweight='bold')
    plt.legend(fontsize=10)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(log_dir, 'plot_robot_distances.png'), dpi=150)
    print(f"[PLOT] Saved: plot_robot_distances.png")
    
    # ================================================================
    # PLOT 3: 2D Trajectory plot
    # ================================================================
    plt.figure(figsize=(12, 8))
    for i, robot_id in enumerate(robot_ids):
        positions = data[f'robot_{robot_id}_positions']
        plt.plot(positions[:, 0], positions[:, 1], 
                color=colors[i % len(colors)], linewidth=2, 
                label=f'Robot {robot_id}', alpha=0.7)
        # Mark start and end
        plt.plot(positions[0, 0], positions[0, 1], 'o', 
                color=colors[i % len(colors)], markersize=10, label=f'R{robot_id} Start')
        plt.plot(positions[-1, 0], positions[-1, 1], 's', 
                color=colors[i % len(colors)], markersize=10, label=f'R{robot_id} End')
    
    plt.xlabel('X Position (m)', fontsize=12)
    plt.ylabel('Y Position (m)', fontsize=12)
    plt.title('Robot Trajectories (Top-Down View)', fontsize=14, fontweight='bold')
    plt.legend(fontsize=9, ncol=2)
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.tight_layout()
    plt.savefig(os.path.join(log_dir, 'plot_trajectories.png'), dpi=150)
    print(f"[PLOT] Saved: plot_trajectories.png")
    
    # ================================================================
    # PLOT 4: Distance histogram
    # ================================================================
    plt.figure(figsize=(10, 6))
    plt.hist(min_dist, bins=50, color='skyblue', edgecolor='black', alpha=0.7)
    plt.axvline(x=0.6, color='g', linestyle='--', linewidth=2, label='Target Safety (0.6m)')
    plt.axvline(x=0.3, color='r', linestyle='--', linewidth=2, label='Critical (0.3m)')
    plt.xlabel('Distance (m)', fontsize=12)
    plt.ylabel('Frequency', fontsize=12)
    plt.title('Distribution of Minimum Distances', fontsize=14, fontweight='bold')
    plt.legend(fontsize=10)
    plt.grid(True, alpha=0.3, axis='y')
    plt.tight_layout()
    plt.savefig(os.path.join(log_dir, 'plot_distance_histogram.png'), dpi=150)
    print(f"[PLOT] Saved: plot_distance_histogram.png")
    
    print(f"[PLOT] All plots saved to: {log_dir}")
    plt.close('all')


if __name__ == '__main__':
    # Example: Generate plots from existing log directory
    import sys
    if len(sys.argv) > 1:
        log_dir = sys.argv[1]
        generate_plots(log_dir)
    else:
        print("Usage: python data_logger.py <log_directory>")
