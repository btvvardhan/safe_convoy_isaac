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
from ctrl.formation_p_controller import FormationPController


def main():
    # 1) Config
    cfg = WarehouseConvoyCfg()

    # 2) Environment (world, robots, obstacles)
    env = WarehouseConvoyEnv(cfg)

    # 3) Controller
    controller = FormationPController(env, cfg)

    # 4) Register physics callback
    env.add_physics_callback("convoy_controller", controller.physics_step)

    # 5) Reset and run
    env.reset()

    while simulation_app.is_running():
        env.step(render=True)

    simulation_app.close()


if __name__ == "__main__":
    main()

