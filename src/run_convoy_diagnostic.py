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

# ============================================================
# DIAGNOSTICS: Check if cvxpy is available
# ============================================================
try:
    import cvxpy as cp
    print("[DIAGNOSTIC] ✅ cvxpy is installed!")
    CVXPY_AVAILABLE = True
except ImportError:
    print("[DIAGNOSTIC] ❌ cvxpy NOT installed!")
    print("[DIAGNOSTIC] Install with: cd ~/isaac-sim && ./python.sh -m pip install cvxpy --user")
    CVXPY_AVAILABLE = False

from env.warehouse_convoy_cfg import WarehouseConvoyCfg
from env.warehouse_convoy_env import WarehouseConvoyEnv

# ============================================================
# CONTROLLER SELECTION
# ============================================================
USE_CLF_CBF = True  # Set to False to use simple P-controller

if USE_CLF_CBF and CVXPY_AVAILABLE:
    try:
        from ctrl.clf_cbf_controller import ClfCbfConvoyController as Controller
        print("[MAIN] 🛡️  Using CLF-CBF safety-critical controller")
        USING_CLF_CBF = True
    except Exception as e:
        print(f"[MAIN] ❌ Failed to import CLF-CBF controller: {e}")
        print("[MAIN] 🎮 Falling back to P-controller")
        from ctrl.formation_p_controller import FormationPController as Controller
        USING_CLF_CBF = False
elif USE_CLF_CBF and not CVXPY_AVAILABLE:
    print("[MAIN] ⚠️  CLF-CBF requested but cvxpy not available!")
    print("[MAIN] 🎮 Using P-controller")
    from ctrl.formation_p_controller import FormationPController as Controller
    USING_CLF_CBF = False
else:
    from ctrl.formation_p_controller import FormationPController as Controller
    print("[MAIN] 🎮 Using simple P-controller")
    USING_CLF_CBF = False
# ============================================================


def main():
    # 1) Config
    cfg = WarehouseConvoyCfg()

    print("\n" + "="*60)
    print("🚀 CONVOY SIMULATION DIAGNOSTICS")
    print("="*60)
    print(f"Controller: {'CLF-CBF' if USING_CLF_CBF else 'P-controller'}")
    print(f"Leader: {cfg.leader_start_xy} → {cfg.goal_xy}")
    print(f"Followers: {cfg.num_followers}")
    print(f"Obstacles: {len(cfg.extra_obstacle_xy)} static")
    for i, (ox, oy) in enumerate(cfg.extra_obstacle_xy):
        print(f"  - Obstacle {i}: ({ox:.1f}, {oy:.1f})")
    print("="*60)
    
    # Check if obstacles are in the path
    path_y = cfg.leader_start_xy[1]  # robots travel at this y
    obstacles_in_path = []
    for ox, oy in cfg.extra_obstacle_xy:
        if abs(oy - path_y) < 0.5:  # Within 0.5m of path
            obstacles_in_path.append((ox, oy))
    
    if len(obstacles_in_path) == 0:
        print("⚠️  WARNING: No obstacles in convoy path!")
        print("⚠️  Robots will drive straight without avoidance.")
        print("⚠️  Edit cfg.extra_obstacle_xy to put obstacles at y ≈ 0")
    else:
        print(f"✅ {len(obstacles_in_path)} obstacles in convoy path")
    
    if not USING_CLF_CBF:
        print("\n⚠️  WARNING: P-controller will PUSH THROUGH obstacles!")
        print("⚠️  To use CLF-CBF, install cvxpy and set USE_CLF_CBF=True")
    
    print("="*60 + "\n")

    # 2) Environment (world, robots, obstacles)
    env = WarehouseConvoyEnv(cfg)

    # 3) Controller
    controller = Controller(env, cfg)

    # 4) Register physics callback
    env.add_physics_callback("convoy_controller", controller.physics_step)

    # 5) Reset and run
    env.reset()

    step_count = 0
    last_print_time = 0.0
    
    while simulation_app.is_running():
        env.step(render=True)
        step_count += 1
        
        # Print status every 2 seconds (only for CLF-CBF)
        if USING_CLF_CBF and hasattr(env, 'current_time'):
            current_time = step_count * 0.01667  # approximate time
            if current_time - last_print_time >= 2.0:
                last_print_time = current_time
                
                print(f"\n[t={current_time:.1f}s]")
                controller.print_safety_margins()
                
                qp_status = controller.get_qp_status()
                optimal_count = sum(1 for s in qp_status.values() if s == "optimal")
                print(f"QP solves: {optimal_count}/{len(qp_status)} optimal")

    print("\n🏁 Simulation ended!")
    simulation_app.close()


if __name__ == "__main__":
    main()
