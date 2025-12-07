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

from env.warehouse_convoy_cfg import WarehouseConvoyCfg
from env.warehouse_convoy_env import WarehouseConvoyEnv

# ============================================================
# CONTROLLER SELECTION: Change this flag to switch controllers
# ============================================================
USE_CLF_CBF = True  # Set to False to use simple P-controller

if USE_CLF_CBF:
    from ctrl.clf_cbf_controller import ClfCbfConvoyController as Controller
    print("[MAIN] Using CLF-CBF safety-critical controller")
else:
    from ctrl.formation_p_controller import FormationPController as Controller
    print("[MAIN] Using simple P-controller")
# ============================================================


def main():
    # 1) Config
    cfg = WarehouseConvoyCfg()

    # 2) Environment (world, robots, obstacles)
    env = WarehouseConvoyEnv(cfg)

    # 3) Controller (selected based on USE_CLF_CBF flag)
    controller = Controller(env, cfg)

    # 4) Register physics callback
    env.add_physics_callback("convoy_controller", controller.physics_step)

    # 5) Reset and run
    env.reset()

    step_count = 0
    while simulation_app.is_running():
        env.step(render=True)
        step_count += 1
        
        # Optional: Print safety margins every 100 steps (only for CLF-CBF)
        if USE_CLF_CBF and step_count % 100 == 0:
            controller.print_safety_margins()

    simulation_app.close()


if __name__ == "__main__":
    main()
