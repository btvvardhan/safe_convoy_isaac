# Safe Warehouse Convoy in Isaac Sim

This project implements a leader–follower convoy of mobile robots in a warehouse-like environment using **NVIDIA Isaac Sim**. The current version uses simple proportional control (no CLF–CBF yet) to validate the simulation setup, environment, and project structure.

> Later, this will be upgraded to a **CLF–CBF-based safety-critical convoy controller**.

---

## 1. Prerequisites

- **OS:** Linux (tested on Ubuntu)
- **Isaac Sim:** Installed under `~/isaac-sim`  
  - Follow NVIDIA’s official install docs for Isaac Sim. 
- **GPU drivers / CUDA:** Whatever Isaac Sim requires (already set up if Isaac Sim runs).

We assume you can run Isaac’s own examples, e.g.:

```bash
cd ~/isaac-sim
./python.sh standalone_examples/api/isaacsim.simulation_app/hello_world.py
