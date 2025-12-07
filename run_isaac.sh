#!/usr/bin/env bash

# Path to your Isaac Sim install
ISAAC_ROOT="$HOME/isaac-sim"

# Add our src/ folder to PYTHONPATH (so env/ ctrl/ utils/ are visible as packages)
export PYTHONPATH="$(pwd)/src:$PYTHONPATH"

# Run the main script using Isaac's Python
"$ISAAC_ROOT/python.sh" "$(pwd)/src/run_convoy_debug.py"
