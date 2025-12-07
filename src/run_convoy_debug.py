from isaacsim import SimulationApp

import os
import sys

THIS_FILE = os.path.abspath(__file__)
SRC_ROOT = os.path.dirname(THIS_FILE)

if SRC_ROOT not in sys.path:
    sys.path.append(SRC_ROOT)

simulation_app = SimulationApp({"headless": False})

from env.warehouse_convoy_cfg import WarehouseConvoyCfg
from env.warehouse_convoy_env import WarehouseConvoyEnv
from ctrl.clf_cbf_controller_debug import ClfCbfConvoyControllerDebug

def main():
    cfg = WarehouseConvoyCfg()
    env = WarehouseConvoyEnv(cfg)
    controller = ClfCbfConvoyControllerDebug(env, cfg)
    
    env.add_physics_callback("convoy_controller", controller.physics_step)
    env.reset()
    
    print("\n" + "="*60)
    print("🔍 CLF-CBF DEBUG MODE")
    print("="*60)
    print("Watch the console for CBF activation messages!")
    print("Look for:")
    print("  - 🟢 Green = Safe (far from obstacles)")
    print("  - 🟡 Yellow = CBF should be active")
    print("  - 🔴 Red = DANGER (too close!)")
    print("="*60 + "\n")
    
    step_count = 0
    last_print_time = 0.0
    
    while simulation_app.is_running():
        env.step(render=True)
        step_count += 1
        
        # Print every 2 seconds
        current_time = step_count * 0.01667
        if current_time - last_print_time >= 2.0:
            last_print_time = current_time
            
            print(f"\n[t={current_time:.1f}s, step={step_count}]")
            controller.print_safety_margins()
            
            qp_status = controller.get_qp_status()
            optimal_count = sum(1 for s in qp_status.values() if s == "optimal")
            failed = [f"R{i}:{s}" for i, s in qp_status.items() if s != "optimal"]
            
            print(f"QP solves: {optimal_count}/{len(qp_status)} optimal", end="")
            if failed:
                print(f" (failed: {', '.join(failed)})")
            else:
                print()

    simulation_app.close()

if __name__ == "__main__":
    main()
