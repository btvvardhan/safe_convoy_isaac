from isaacsim import SimulationApp

import os
import sys

THIS_FILE = os.path.abspath(__file__)
SRC_ROOT = os.path.dirname(THIS_FILE)

if SRC_ROOT not in sys.path:
    sys.path.append(SRC_ROOT)

simulation_app = SimulationApp({"headless": False})

import argparse
from env.warehouse_convoy_cfg_visual import WarehouseConvoyCfgVisual
from env.warehouse_convoy_env_visual import WarehouseConvoyEnvVisual
from ctrl.clf_cbf_controller_debug import ClfCbfConvoyControllerDebug
from env.visual_effects import VisualEffectsManager, SimpleVisualizer
from env.data_logger import DataLogger, generate_plots


def main():
    # ================================================================
    # PARSE ARGUMENTS
    # ================================================================
    parser = argparse.ArgumentParser(description='Visual Demo for Safe Convoy')
    parser.add_argument('--scenario', type=str, default='default',
                       choices=['default', 'corridor', 'intersection', 'zigzag', 'emergency', 'rush_hour'],
                       help='Scenario to run')
    parser.add_argument('--duration', type=float, default=30.0,
                       help='Simulation duration in seconds')
    parser.add_argument('--no-visual', action='store_true',
                       help='Disable visual effects')
    parser.add_argument('--no-logging', action='store_true',
                       help='Disable data logging')
    parser.add_argument('--ascii-viz', action='store_true',
                       help='Enable ASCII visualization in console')
    
    args = parser.parse_args()
    
    # ================================================================
    # CONFIGURATION
    # ================================================================
    cfg = WarehouseConvoyCfgVisual()
    
    # Load scenario
    if args.scenario != 'default':
        cfg.load_scenario(args.scenario)
    
    # Override visual settings if requested
    if args.no_visual:
        cfg.enable_visual_effects = False
        cfg.enable_robot_trails = False
        cfg.enable_safety_bubbles = False
        cfg.enable_danger_coloring = False
    
    if args.no_logging:
        cfg.enable_logging = False
    
    # ================================================================
    # CREATE ENVIRONMENT
    # ================================================================
    print("\n" + "="*60)
    print(f"🚀 VISUAL CONVOY DEMO - SCENARIO: {args.scenario.upper()}")
    print("="*60 + "\n")
    
    env = WarehouseConvoyEnvVisual(cfg)
    
    # ================================================================
    # CREATE CONTROLLER
    # ================================================================
    controller = ClfCbfConvoyControllerDebug(env, cfg)
    
    # ================================================================
    # CREATE VISUAL EFFECTS
    # ================================================================
    visual_fx = None
    if cfg.enable_visual_effects:
        visual_fx = VisualEffectsManager(cfg)
        print("[MAIN] Visual effects enabled")
    
    # ================================================================
    # CREATE DATA LOGGER
    # ================================================================
    data_logger = None
    if cfg.enable_logging:
        data_logger = DataLogger(cfg)
        print("[MAIN] Data logging enabled")
    
    # ================================================================
    # ASCII VISUALIZER (optional)
    # ================================================================
    ascii_viz = None
    if args.ascii_viz:
        ascii_viz = SimpleVisualizer()
        print("[MAIN] ASCII visualization enabled")
    
    # ================================================================
    # PHYSICS CALLBACK
    # ================================================================
    def physics_callback(step_size: float):
        """Combined callback that updates everything."""
        # 1. Update moving obstacles
        env.update_dynamic_elements(step_size)
        
        # 2. Run controller
        controller.physics_step(step_size)
        
        # 3. Update visual effects
        if visual_fx is not None:
            visual_fx.update(env.robots, env.obstacle_centers)
        
        # 4. Log data
        if data_logger is not None:
            data_logger.log_step(
                env.current_time,
                env.robots,
                env.obstacle_centers,
                controller
            )
    
    # ================================================================
    # REGISTER CALLBACK AND RESET
    # ================================================================
    env.add_physics_callback("visual_convoy_controller", physics_callback)
    env.reset()
    
    # ================================================================
    # MAIN LOOP
    # ================================================================
    print("\n" + "="*60)
    print("▶️  SIMULATION STARTED")
    print("="*60)
    print(f"Scenario: {args.scenario}")
    print(f"Duration: {args.duration}s")
    print(f"Moving obstacles: {env.get_obstacle_count()['moving']}")
    print(f"Visual effects: {cfg.enable_visual_effects}")
    print(f"Data logging: {cfg.enable_logging}")
    print("="*60 + "\n")
    
    step_count = 0
    last_print_time = 0.0
    last_visual_update = 0.0
    
    while simulation_app.is_running():
        # Step simulation
        env.step(render=True)
        step_count += 1
        
        current_time = step_count * 0.01667  # Approximate time
        
        # Check if we've reached duration
        if current_time >= args.duration:
            print(f"\n⏱️  Reached target duration ({args.duration}s)")
            break
        
        # Print status every 2 seconds
        if current_time - last_print_time >= 2.0:
            last_print_time = current_time
            
            print(f"\n[t={current_time:.1f}s, step={step_count}]")
            
            # Obstacle count
            obs_count = env.get_obstacle_count()
            print(f"Obstacles: Static={obs_count['static']}, Moving={obs_count['moving']}, Total={obs_count['total']}")
            
            # Safety margins
            controller.print_safety_margins()
            
            # QP status
            qp_status = controller.get_qp_status()
            optimal_count = sum(1 for s in qp_status.values() if s == "optimal")
            failed = [f"R{i}:{s}" for i, s in qp_status.items() if s != "optimal"]
            
            print(f"QP solves: {optimal_count}/{len(qp_status)} optimal", end="")
            if failed:
                print(f" (failed: {', '.join(failed)})")
            else:
                print()
            
            # Visual effects status
            if visual_fx is not None and current_time - last_visual_update >= 5.0:
                last_visual_update = current_time
                visual_fx.print_status()
        
        # ASCII visualization (if enabled)
        if ascii_viz is not None and step_count % 100 == 0:
            ascii_viz.render(env.robots, env.obstacle_centers)
    
    # ================================================================
    # CLEANUP AND SAVE
    # ================================================================
    print("\n" + "="*60)
    print("🏁 SIMULATION COMPLETE")
    print("="*60 + "\n")
    
    # Save data
    if data_logger is not None:
        data_logger.save()
        data_logger.print_summary()
        
        # Generate plots
        print("[MAIN] Generating plots...")
        generate_plots(data_logger.log_dir)
    
    # Reset visual effects
    if visual_fx is not None:
        visual_fx.reset()
    
    print("\n✅ Demo complete! Press Ctrl+C to exit.\n")
    
    simulation_app.close()


if __name__ == "__main__":
    main()
