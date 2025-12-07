from isaacsim import SimulationApp
import os
import sys

THIS_FILE = os.path.abspath(__file__)
SRC_ROOT = os.path.dirname(THIS_FILE)
if SRC_ROOT not in sys.path:
    sys.path.append(SRC_ROOT)

simulation_app = SimulationApp({"headless": False})

import argparse
from env.warehouse_convoy_cfg_enhanced import WarehouseConvoyCfgEnhanced
from env.warehouse_convoy_env_visual import WarehouseConvoyEnvVisual
from ctrl.adaptive_spacing_controller import AdaptiveSpacingClfCbfController
from env.visual_effects import VisualEffectsManager
from env.data_logger import DataLogger, generate_plots


def main():
    # ================================================================
    # PARSE ARGUMENTS
    # ================================================================
    parser = argparse.ArgumentParser(
        description='Adaptive Spacing Demo - Robot adjusts spacing based on corridor width'
    )
    parser.add_argument('--scenario', type=str, default='adaptive_demo',
                       choices=['adaptive_demo', 'narrow_corridor', 'wide_warehouse', 'busy_warehouse'],
                       help='Scenario to run (default: adaptive_demo)')
    parser.add_argument('--duration', type=float, default=40.0,
                       help='Simulation duration in seconds (default: 40)')
    parser.add_argument('--no-visual', action='store_true',
                       help='Disable visual effects')
    parser.add_argument('--no-logging', action='store_true',
                       help='Disable data logging')
    
    args = parser.parse_args()
    
    # ================================================================
    # CONFIGURATION
    # ================================================================
    cfg = WarehouseConvoyCfgEnhanced()
    cfg.load_scenario(args.scenario)
    
    if args.no_visual:
        cfg.enable_visual_effects = False
    if args.no_logging:
        cfg.enable_logging = False
    
    # ================================================================
    # PRINT INTRO
    # ================================================================
    print("\n" + "="*70)
    print("🚀 ADAPTIVE SPACING DEMO - " + args.scenario.upper())
    print("="*70)
    print("\nThis demo shows INTELLIGENT robot navigation:")
    print("  • Robot detects corridor width in real-time")
    print("  • Adjusts safety margins automatically:")
    print()
    print("    🟦 WIDE mode       → d_safe = 0.8m (conservative, lots of space)")
    print("    🟨 TRANSITION mode → d_safe = 0.4-0.8m (adapting)")
    print("    🟥 NARROW mode     → d_safe = 0.4m (aggressive, tight space)")
    print()
    print("Watch the console for mode changes as the robot navigates!")
    print("="*70 + "\n")
    
    # ================================================================
    # CREATE ENVIRONMENT
    # ================================================================
    print("[MAIN] Creating environment...")
    env = WarehouseConvoyEnvVisual(cfg)
    
    # ================================================================
    # CREATE ADAPTIVE CONTROLLER
    # ================================================================
    print("[MAIN] Creating ADAPTIVE spacing controller...")
    controller = AdaptiveSpacingClfCbfController(env, cfg)
    
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
    # PHYSICS CALLBACK
    # ================================================================
    def physics_callback(step_size: float):
        """Combined callback that updates everything."""
        # 1. Update moving obstacles (with collision avoidance!)
        env.update_dynamic_elements(step_size)
        
        # 2. Run ADAPTIVE controller
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
    env.add_physics_callback("adaptive_convoy_controller", physics_callback)
    env.reset()
    
    # ================================================================
    # MAIN LOOP
    # ================================================================
    print("\n" + "="*70)
    print("▶️  SIMULATION STARTED")
    print("="*70)
    print(f"Scenario: {args.scenario}")
    print(f"Duration: {args.duration}s")
    print(f"Moving obstacles: {env.get_obstacle_count()['moving']}")
    print(f"Static obstacles: {env.get_obstacle_count()['static']}")
    print("="*70 + "\n")
    
    step_count = 0
    last_print_time = 0.0
    
    while simulation_app.is_running():
        # Step simulation
        env.step(render=True)
        step_count += 1
        current_time = step_count * 0.01667
        
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
            
            # ADAPTIVE SAFETY MARGINS - Shows WIDE/NARROW/TRANSITION modes!
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
    
    # ================================================================
    # CLEANUP AND SAVE
    # ================================================================
    print("\n" + "="*70)
    print("🏁 SIMULATION COMPLETE")
    print("="*70 + "\n")
    
    # Save data
    if data_logger is not None:
        print("[MAIN] Saving data...")
        data_logger.save()
        data_logger.print_summary()
        
        # Generate plots
        print("[MAIN] Generating plots...")
        generate_plots(data_logger.log_dir)
        print(f"\n📊 Results saved to: {data_logger.log_dir}")
    
    # Reset visual effects
    if visual_fx is not None:
        visual_fx.reset()
    
    print("\n" + "="*70)
    print("✅ DEMO COMPLETE!")
    print("="*70)
    print("\nKey Observations:")
    print("  • Did you see the mode changes? (🟦 → 🟨 → 🟥)")
    print("  • Notice how d_safe adjusted based on space?")
    print("  • All collisions avoided? Check the plots!")
    print("\nThis shows INTELLIGENT navigation - not just avoiding,")
    print("but adapting behavior based on environment!")
    print("="*70 + "\n")
    
    simulation_app.close()


if __name__ == "__main__":
    main()
