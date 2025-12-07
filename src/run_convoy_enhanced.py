from isaacsim import SimulationApp

import os
import sys

# Make sure ~/safe_convoy_isaac/src is on sys.path when this script runs
THIS_FILE = os.path.abspath(__file__)
SRC_ROOT = os.path.dirname(THIS_FILE)  # .../safe_convoy_isaac/src

if SRC_ROOT not in sys.path:
    sys.path.append(SRC_ROOT)
# ------------------------------------------------------


# Launch Isaac Sim
simulation_app = SimulationApp({"headless": False})

from env.warehouse_convoy_cfg_enhanced import WarehouseConvoyCfgEnhanced
from env.warehouse_convoy_env_enhanced import WarehouseConvoyEnvEnhanced

# ============================================================
# CONTROLLER SELECTION: Change this flag to switch controllers
# ============================================================
USE_CLF_CBF = True  # Set to False to use simple P-controller

if USE_CLF_CBF:
    from ctrl.clf_cbf_controller import ClfCbfConvoyController as Controller
    print("[MAIN] 🛡️  Using CLF-CBF safety-critical controller")
else:
    from ctrl.formation_p_controller import FormationPController as Controller
    print("[MAIN] 🎮 Using simple P-controller")
# ============================================================


def main():
    # 1) Config
    cfg = WarehouseConvoyCfgEnhanced()

    # 2) Environment (world, robots, obstacles)
    env = WarehouseConvoyEnvEnhanced(cfg)

    # 3) Controller (selected based on USE_CLF_CBF flag)
    controller = Controller(env, cfg)

    # 4) Create combined physics callback that updates both env and controller
    def physics_callback(step_size: float):
        """
        Combined callback that:
        1. Updates dynamic environment (obstacles, human)
        2. Runs controller
        """
        # Update environment first (spawn/despawn obstacles, move human)
        env.update_dynamic_elements(step_size)
        
        # Then run controller with updated obstacle list
        controller.physics_step(step_size)

    # 5) Register physics callback
    env.add_physics_callback("enhanced_convoy_controller", physics_callback)

    # 6) Reset and run
    env.reset()

    step_count = 0
    last_print_time = 0.0
    
    print("\n" + "="*60)
    print("🚀 ENHANCED CONVOY SIMULATION STARTED!")
    print("="*60)
    print(f"Static obstacles: {len(env.static_obstacle_centers)}")
    print(f"Dynamic spawning: {cfg.enable_dynamic_obstacles}")
    print(f"Moving human: {cfg.enable_moving_agent}")
    print(f"Controller: {'CLF-CBF' if USE_CLF_CBF else 'P-controller'}")
    print("="*60 + "\n")

    while simulation_app.is_running():
        env.step(render=True)
        step_count += 1
        
        # Print status every 2 seconds
        if USE_CLF_CBF and env.current_time - last_print_time >= 2.0:
            last_print_time = env.current_time
            
            obs_count = env.get_obstacle_count()
            print(f"\n[t={env.current_time:.1f}s] Obstacles: "
                  f"Static={obs_count['static']}, "
                  f"Dynamic={obs_count['dynamic']}, "
                  f"Human={obs_count['human']}, "
                  f"Total={obs_count['total']}")
            
            # Print safety margins
            controller.print_safety_margins()
            
            # Print QP status
            qp_status = controller.get_qp_status()
            optimal_count = sum(1 for s in qp_status.values() if s == "optimal")
            print(f"QP solves: {optimal_count}/{len(qp_status)} optimal")

    print("\n🏁 Simulation ended!")
    simulation_app.close()


if __name__ == "__main__":
    main()
